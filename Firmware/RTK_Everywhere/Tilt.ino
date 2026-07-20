/*=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
Tilt.ino

  Once RTK Fix is achieved, and the tilt sensor is activated (ie rocked back and forth) the tilt sensor
  generates binary-encoded lat/lon/alt values that are tilt-compensated. To get these values to the
  GIS Data Collector software, we need to transmit corrected NMEA sentences over Bluetooth. The
  Data Collector does not know anything is being tilt-compensated. To do this we must intercept
  NMEA from the UM980 and splice in the values from the tilt sensor. See tiltApplyCompensationGGA()
  as an example.

  The tilt sensor reports + and - numbers for Latitude/Longitude. Whereas NMEA expects positive
  numbers with letters N/S and E/W. Since we are splicing into NMEA, the correct N/S and E/W letters
  are already set. We just need to be sure the tilt-compensated values are positive using abs().
  This could lead to problems if the unit is within ~1m of the Equator and Prime Meridian but
  we don't consider those edges cases here.

  It looks like the IM19 only supports 115200 baud...

  On Torch:
    The IM19 UART2 is fed by the UM980 UART2
    The IM19 gets BESTPOSB, PSRVELB, GPGGA at 5Hz at 115200 baud
    The IM19 outputs the binary NAVI message on UART1. This is connected to ESP32 UART2 (SerialForTilt)
    tiltSensor->update() checks ESP32 UART2 for the most recent incoming binary data

  On Facet FP:
    LG290P with Tilt:
      The IM19 UART2 is fed by the LG290P UART3
      The IM19 gets GGA, RMC and GST at >= 5Hz at 115200 baud. Messages are enabled by setMessagesNMEA()
    mosaic-X5 with Tilt:
      The IM19 UART2 is fed by the X5 UART4
      mosaic-X5 setTilt() creates a Stream and outputs GGA, RMC and GST at 5Hz at 115200 baud
    ZED-X20P with Tilt:
      The IM19 UART2 is fed by the X20P UART1 - which also feeds ESP32 UART1
      The message rates and baud rate need to be configured according to what the IM19 needs

=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=*/

// ---- MEMS BLE streaming + RTK correction injection + fix status ---------
// 32-byte frame: timestamp (double) + accel XYZ (float×3) + gyro XYZ (float×3)
#pragma pack(push, 1)
struct MemsFrame { double t; float ax, ay, az, gx, gy, gz; };
#pragma pack(pop)

#define MEMS_RING_SIZE 128  // power of 2; 1.28s at 100 Hz

static MemsFrame   memsRing[MEMS_RING_SIZE];
static volatile uint8_t memsHead = 0, memsTail = 0;

static bool memsBleInited = false;

#ifdef COMPILE_BT
#include "BleSerialServer.h"
#include <BLE2902.h>

// UUIDs — all on the same 4d360001 service so the phone connects once.
#define MEMS_SERVICE_UUID "4d360001-0000-1000-8000-004d36000000"
#define MEMS_CHAR_UUID    "4d360002-0000-1000-8000-004d36000000"
#define RTCM_WRITE_UUID   "4d360003-0000-1000-8000-004d36000000"  // phone → Torch RTCM3
#define STATUS_CHAR_UUID  "4d360004-0000-1000-8000-004d36000000"  // Torch → phone fix status
#define GNSS_CTRL_UUID    "4d360005-0000-1000-8000-004d36000000"  // phone → Torch: 0x01=send file, 0x00=abort
#define GNSS_DATA_UUID    "4d360006-0000-1000-8000-004d36000000"  // Torch → phone: raw obs stream

static BLECharacteristic *memsChar     = nullptr;
static BLECharacteristic *rtcmChar     = nullptr;
static BLECharacteristic *statusChar   = nullptr;
static BLECharacteristic *gnssCtrlChar = nullptr;
static BLECharacteristic *gnssDataChar = nullptr;

// Raw GNSS byte capture (RTCM MSM7 + NMEA) streamed to LittleFS for PPK post-processing.
// Written by gnssReadTask; served to the phone via 4d360006 on demand.
static File           gnssRawFile;
static char           gnssRawFileName[64] = {0};
static volatile bool  gnssRawLogging = false;
static volatile bool  gnssXferActive = false;

// Called from gnssReadTask (Tasks.ino) — shields it from the static internals.
void gnssRawWriteBytes(const uint8_t *buf, size_t len)
{
    if (gnssRawLogging && gnssRawFile && len > 0)
        gnssRawFile.write(buf, len);
}

static void gnssOpenRawFile()
{
    if (gnssRawFile) return;
    snprintf(gnssRawFileName, sizeof(gnssRawFileName),
             "/gnss_%02d%02d%02d_%02d%02d%02d.rtcm3",
             rtc.getYear() - 2000, rtc.getMonth() + 1, rtc.getDay(),
             rtc.getHour(true), rtc.getMinute(), rtc.getSecond());
    gnssRawFile = LittleFS.open(gnssRawFileName, FILE_WRITE);
    if (!gnssRawFile)
    {
        gnssRawFileName[0] = 0;
        systemPrintln("gnssRawFile: open failed");
        return;
    }
    gnssRawLogging = true;
    systemPrintf("gnssRawFile: logging to %s\r\n", gnssRawFileName);
}

// Forward declaration — defined after GnssCtrlCallback.
static void gnssXferTask(void *);

// Write callback: forward RTCM3 bytes received over BLE to the GNSS receiver.
// The UM980 accepts a continuous RTCM3 byte stream on its UART — no framing needed here.
class RtcmWriteCallback : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *c) override
    {
        String val = c->getValue();
        if (val.length() > 0 && gnss != nullptr)
            gnss->pushRawData((uint8_t *)val.c_str(), (int)val.length());
    }
};
static RtcmWriteCallback rtcmWriteCb;

// Write callback: 0x01 = transfer last raw obs file; 0x00 = abort transfer.
class GnssCtrlCallback : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *c) override
    {
        String val = c->getValue();
        if (val.length() == 0) return;
        uint8_t cmd = (uint8_t)val[0];

        if (cmd == 0x01) // send file
        {
            // Stop captures, flush and close current file, then stream it.
            gnssRawLogging = false;
            if (gnssRawFile)
            {
                gnssRawFile.close();
            }
            if (gnssDataChar == nullptr) return;
            if (gnssRawFileName[0] == 0)
            {
                // No file yet — send 4-byte zero header so the phone knows.
                uint8_t zero[4] = {0};
                gnssDataChar->setValue(zero, 4);
                gnssDataChar->notify();
                return;
            }
            if (!gnssXferActive)
                xTaskCreate(gnssXferTask, "gnssXfer", 8192, nullptr, 1, nullptr);
        }
        else if (cmd == 0x00) // abort
        {
            gnssXferActive = false;
        }
    }
};
static GnssCtrlCallback gnssCtrlCb;

// FreeRTOS task: streams the most recent raw obs file over BLE notify in 182-byte chunks.
// Protocol: first notification = [fileSize: u32 LE][data...]; subsequent = [data...].
// After transfer, opens a fresh log file for the next capture session.
static void gnssXferTask(void *e)
{
    gnssXferActive = true;

    File f = LittleFS.open(gnssRawFileName, FILE_READ);
    if (!f)
    {
        uint8_t zero[4] = {0};
        gnssDataChar->setValue(zero, 4);
        gnssDataChar->notify();
        gnssXferActive = false;
        vTaskDelete(nullptr);
        return;
    }

    uint32_t fileSize = (uint32_t)f.size();
    const uint16_t CHUNK = 182;
    uint8_t buf[182 + 4];

    // First packet includes the 4-byte file-size header.
    buf[0] = (uint8_t)(fileSize & 0xFF);
    buf[1] = (uint8_t)((fileSize >>  8) & 0xFF);
    buf[2] = (uint8_t)((fileSize >> 16) & 0xFF);
    buf[3] = (uint8_t)((fileSize >> 24) & 0xFF);
    int n = f.read(buf + 4, CHUNK);
    if (n > 0)
    {
        gnssDataChar->setValue(buf, (size_t)(4 + n));
        gnssDataChar->notify();
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    while (gnssXferActive)
    {
        n = f.read(buf, CHUNK);
        if (n <= 0) break;
        gnssDataChar->setValue(buf, (size_t)n);
        gnssDataChar->notify();
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    f.close();
    gnssXferActive = false;

    // Free the space and immediately start a fresh obs file for the next capture.
    LittleFS.remove(gnssRawFileName);
    gnssRawFileName[0] = 0;
    if (online.fs && online.rtc)
        gnssOpenRawFile();

    vTaskDelete(nullptr);
}

void memsBleInit()
{
    if (memsBleInited) return; // Already done — safe to call multiple times

    BLEServer *srv = BleSerialServer::getInstance().Server;
    if (srv == nullptr)
    {
        systemPrintln("MEMS BLE: server not ready yet");
        return;
    }

    // Handle space: 1 service + 3+2+3+2+3 chars/descriptors = 14 → use 24 for headroom.
    BLEService *svc = srv->createService(BLEUUID(MEMS_SERVICE_UUID), 24);

    // 4d360002 — IMU frame notify (existing)
    memsChar = svc->createCharacteristic(BLEUUID(MEMS_CHAR_UUID),
                                          BLECharacteristic::PROPERTY_NOTIFY);
    memsChar->addDescriptor(new BLE2902());

    // 4d360003 — RTCM3 write (phone relays corrections from NTRIP caster)
    rtcmChar = svc->createCharacteristic(BLEUUID(RTCM_WRITE_UUID),
                                          BLECharacteristic::PROPERTY_WRITE |
                                          BLECharacteristic::PROPERTY_WRITE_NR);
    rtcmChar->setCallbacks(&rtcmWriteCb);

    // 4d360004 — fix status notify (16 bytes, little-endian):
    //   [0]     fixCat  (0=none 1=single 2=RTK-float 3=RTK-fixed)
    //   [1]     sats
    //   [2-3]   hAccMm  (uint16)
    //   [4-7]   lat     (int32, degrees × 1e7)
    //   [8-11]  lon     (int32, degrees × 1e7)
    //   [12-15] t_utc   (float32, UTC seconds-of-day; 0.0 = not yet valid)
    statusChar = svc->createCharacteristic(BLEUUID(STATUS_CHAR_UUID),
                                            BLECharacteristic::PROPERTY_NOTIFY);
    statusChar->addDescriptor(new BLE2902());

    // 4d360005 — raw obs transfer control (write, phone → Torch)
    gnssCtrlChar = svc->createCharacteristic(BLEUUID(GNSS_CTRL_UUID),
                                              BLECharacteristic::PROPERTY_WRITE |
                                              BLECharacteristic::PROPERTY_WRITE_NR);
    gnssCtrlChar->setCallbacks(&gnssCtrlCb);

    // 4d360006 — raw obs data stream (notify, Torch → phone)
    gnssDataChar = svc->createCharacteristic(BLEUUID(GNSS_DATA_UUID),
                                              BLECharacteristic::PROPERTY_NOTIFY);
    gnssDataChar->addDescriptor(new BLE2902());

    svc->start();

    BLEAdvertising *adv = srv->getAdvertising();
    adv->addServiceUUID(BLEUUID(MEMS_SERVICE_UUID));
    adv->start();

    memsBleInited = true;
    systemPrintln("MEMS BLE ready: IMU(4d360002) RTCM(4d360003) Status(4d360004) GNSS(4d360005/06)");
}

void memsBleUpdate()
{
    if (!memsBleInited || memsChar == nullptr)
        return;

    // ── Raw obs log — open once LittleFS and RTC are both ready ─────────────
    if (!gnssRawFile && online.fs && online.rtc)
        gnssOpenRawFile();

    // ── IMU frames ──────────────────────────────────────────────────────────
    static unsigned long lastSend = 0;
    if (millis() - lastSend >= 100)
    {
        lastSend = millis();

        uint8_t buf[320];
        uint8_t count = 0;
        while (count < 10 && memsTail != memsHead)
        {
            memcpy(buf + (uint16_t)(count * 32), &memsRing[memsTail], 32);
            memsTail = (memsTail + 1) & (MEMS_RING_SIZE - 1);
            count++;
        }
        if (count > 0)
        {
            memsChar->setValue(buf, (uint16_t)(count * 32));
            memsChar->notify();
        }
    }

    // ── Fix status (1 Hz) ───────────────────────────────────────────────────
    if (statusChar == nullptr || gnss == nullptr) return;

    static unsigned long lastStatus = 0;
    if (millis() - lastStatus < 1000) return;
    lastStatus = millis();

    uint8_t fixCat = 0;
    if      (gnss->isRTKFix())                    fixCat = 3;
    else if (gnss->isRTKFloat())                   fixCat = 2;
    else if (gnss->getFixType() >= 16)             fixCat = 1; // single or better

    uint8_t  sats    = gnss->getSatellitesInView();
    float    hAccM   = gnss->getHorizontalAccuracy();
    uint16_t hAccMm  = (uint16_t)min(65535.0f, hAccM * 1000.0f);

    // lat/lon as int32, degrees × 1e7 (≈1 cm resolution), little-endian
    int32_t latI = (int32_t)(gnss->getLatitude()  * 1e7);
    int32_t lonI = (int32_t)(gnss->getLongitude() * 1e7);

    // UTC seconds-of-day (float32). 0.0 signals "not yet valid".
    float tUtc = 0.0f;
    if (gnss->isConfirmedTime())
        tUtc = gnss->getHour() * 3600.0f + gnss->getMinute() * 60.0f
               + gnss->getSecond() + gnss->getNanosecond() * 1e-9f;

    uint8_t status[16];
    status[0] = fixCat;
    status[1] = sats;
    status[2] = (uint8_t)(hAccMm & 0xFF);
    status[3] = (uint8_t)(hAccMm >> 8);
    status[4]  = (uint8_t)(latI         & 0xFF);
    status[5]  = (uint8_t)((latI >>  8) & 0xFF);
    status[6]  = (uint8_t)((latI >> 16) & 0xFF);
    status[7]  = (uint8_t)((latI >> 24) & 0xFF);
    status[8]  = (uint8_t)(lonI         & 0xFF);
    status[9]  = (uint8_t)((lonI >>  8) & 0xFF);
    status[10] = (uint8_t)((lonI >> 16) & 0xFF);
    status[11] = (uint8_t)((lonI >> 24) & 0xFF);
    memcpy(&status[12], &tUtc, 4); // float32 LE
    statusChar->setValue(status, 16);
    statusChar->notify();
}

#else
void memsBleInit()   {}
void memsBleUpdate() {}
#endif  // COMPILE_BT
// -----------------------------------------------------------------------

#ifdef COMPILE_IM19_IMU

typedef enum
{
    TILT_DISABLED = 0,
    TILT_OFFLINE,
    TILT_STARTED,
    TILT_INITIALIZED,
    TILT_CORRECTING,
    TILT_REQUEST_STOP,
} TiltState;
TiltState tiltState = TILT_DISABLED;

// Tilt compensation sensor state machine
void tiltUpdate()
{
    if (present.imu_im19 == false)
        return;

    if (settings.enableTiltCompensation == false && tiltState != TILT_DISABLED)
    {
        tiltStop(); // If the user has disabled the device, shut it down
        tiltState = TILT_DISABLED;
    }

    switch (tiltState)
    {
    default:
        systemPrintf("Unknown tiltState: %d\r\n", tiltState);
        break;

    case TILT_DISABLED:
        if (settings.enableTiltCompensation == true && tiltFailedBegin == false)
            tiltState = TILT_OFFLINE;
        break;

    case TILT_OFFLINE: {
        // Try multiple times to configure IM19
        uint8_t maxTries = 3;
        for (int x = 0; x < maxTries; x++)
        {
            beginTilt(); // Start IMU
            if (tiltState == TILT_STARTED)
                break;
        }

        if (tiltState != TILT_STARTED) // If we failed to begin, disable future attempts
        {
            systemPrintln("Tilt sensor failed to configure after multiple attempts.");
            tiltFailedBegin = true;
            tiltState = TILT_DISABLED;
        }
    }
    break;

    case TILT_STARTED:
        // Always drain UART and process MEMS — 100 Hz stream runs regardless of fix state.
        tiltSensor->update();
        tiltProcessMEMS();
        if (!memsBleInited) memsBleInit();
        memsBleUpdate();

        // RTK Fix required for isInitialized so don't check tilt until we have RTK Fix.
        if (gnss->isRTKFix() == false)
            break;

        // Check IMU state at 1Hz
        if (millis() - lastTiltCheck > 1000)
        {
            lastTiltCheck = millis();

            if (settings.antennaHeight_mm < 500)
                systemPrintf("Warning: Short pole length detected: %0.3fm\r\n", settings.antennaHeight_mm / 1000.0);

            if (settings.enableImuDebug == true)
                printTiltDebug();

            // Check to see if tilt sensor has been rocked
            if (tiltSensor->isInitialized())
            {
                beepDurationMs(1000); // Audibly indicate the init of tilt

                lastTiltBeepMs = millis();

                tiltState = TILT_INITIALIZED;
            }

            // Check to see if tilt compensation is active
            if (tiltSensor->isCorrecting())
            {
                beepMultiple(2, 500, 500); // Number of beeps, length of beep ms, length of quiet ms

                lastTiltBeepMs = millis();

                tiltState = TILT_CORRECTING;
            }
        }
        break;

    case TILT_INITIALIZED:
        // Waiting for user to rock unit back and forth
        tiltSensor->update(); // Check for the most recent incoming binary data
        tiltProcessMEMS();
        memsBleUpdate();

        // Check IMU state at 1Hz
        if ((millis() - lastTiltCheck) > 1000)
        {
            lastTiltCheck = millis();

            if (settings.antennaHeight_mm < 500)
                systemPrintf("Warning: Short pole length detected: %0.3fm\r\n", settings.antennaHeight_mm / 1000.0);

            if (settings.enableImuDebug == true)
                printTiltDebug();

            // Check to see if tilt compensation is active
            if (tiltSensor->isCorrecting())
            {
                beepDurationMs(2000); // Audibly indicate the start of tilt

                lastTiltBeepMs = millis();

                tiltState = TILT_CORRECTING;
            }
        }
        break;

    case TILT_CORRECTING:
        // Check to see if we've stopped correcting
        tiltSensor->update(); // Check for the most recent incoming binary data
        tiltProcessMEMS();
        memsBleUpdate();

        // Check IMU state at 1Hz
        if ((millis() - lastTiltCheck) > 1000)
        {
            lastTiltCheck = millis();

            if (settings.enableImuDebug == true)
                printTiltDebug();

            // Check to see if tilt compensation is active
            if (tiltSensor->isCorrecting() == false)
            {
                tiltState = TILT_STARTED;

                // Beep to indicating tilt went offline
                beepDurationMs(1000);
            }
        }

        // If tilt compensation is active, play a short beep every 10 seconds
        if ((millis() - lastTiltBeepMs) > 10000)
        {
            lastTiltBeepMs = millis();
            beepDurationMs(250);
        }

        break;

    case TILT_REQUEST_STOP:
        tiltStop(); // Changes state to TILT_OFFILINE
        break;
    }
}

/*Datasheet initialization steps:
    Step one: Rotate the receiver in hand, or shake it.

    Step two: If the heading angle becomes 0-180 degrees (or 0-(-180) degrees) it
    means step two has been entered. Wait for RTK to output the fixed solution.

    Step three: Some rocking is required to make accuracy meet the requirements. Rock rod back and
    forth for 5-6 seconds. Maintain the same speed when shaking. 1-2m/s is enough. Rotate the rod 90
    degrees and continue to rock until the init is complete. The status word becomes ready.
*/
void printTiltDebug()
{
    if (inMainMenu)
        return;

    uint32_t naviStatus = tiltSensor->getNaviStatus();
    // systemPrintf("NAVI timestamp: %0.0f lat: %0.4f lon: %0.4f alt: %0.2f\r\n", tiltSensor->getNaviTimestamp(),
    //              tiltSensor->getNaviLatitude(), tiltSensor->getNaviLongitude(), tiltSensor->getNaviAltitude());

    systemPrint("Tilt ");

    if (tiltState == TILT_STARTED)
        systemPrint("STARTED");
    else if (tiltState == TILT_INITIALIZED)
        systemPrint("INITIALIZED");
    else if (tiltState == TILT_CORRECTING)
        systemPrint("CORRECTING");

    systemPrintf(" Status: 0x%04X - ", naviStatus);

    // 0 = No fix, 1 = 3D, 4 = RTK Fix
    int solutionState = tiltSensor->getGnssSolutionState();
    if (solutionState == 4)
        systemPrint("RTK Fix");
    else if (solutionState == 3)
        systemPrint("RTK Float");
    else if (solutionState == 2)
        systemPrint("DGPS Fix");
    else if (solutionState == 1)
        systemPrint("3D Fix");
    else if (solutionState == 0)
        systemPrint("No Fix");
    else
        systemPrintf("solutionState %d", tiltSensor->getGnssSolutionState());

    systemPrintln();

    // if (naviStatus & (1 << 0))
    //     systemPrintln("Status: Filter uninitialized"); // Finit 0x1
    if (naviStatus & (1 << 1))
        systemPrintln("Status: Filter convergence complete"); // Ready 0x2
    if (naviStatus & (1 << 2))
        systemPrintln("Status: In filter convergence"); // Inaccurate 0x4
    if (naviStatus & (1 << 3))
        systemPrintln("Status: Excessive tilt angle"); // TiltReject 0x8

    if (naviStatus & (1 << 4))
        systemPrintln("Status: GNSS Positioning data difference"); // GnssReject 0x10
    if (naviStatus & (1 << 5))
        systemPrintln("Status: Filter Reset"); // FReset 0x20
    if (naviStatus & (1 << 6))
        systemPrintln("Status: Tilt estimation Phase 1"); // FixRlsStage1 0x40
    if (naviStatus & (1 << 7))
        systemPrintln("Status: Tilt estimation Phase 2"); // FixRlsStage2 0x80

    if (naviStatus & (1 << 8))
        systemPrintln("Status: Tilt estimation Phase 3"); // FixRlsStage3 0x100
    if (naviStatus & (1 << 9))
        systemPrintln("Status: Tilt estimation Phase 4"); // FixRlsStage4 0x200
    if (naviStatus & (1 << 10))
        systemPrintln("Status: Tilt estimation Complete"); // FixRlsOK 0x400

    if (naviStatus & (1 << 13))
        systemPrintln("Status: Initialize shaking direction 1"); // Direction1 0x2000
    if (naviStatus & (1 << 14))
        systemPrintln("Status: Initialize shaking direction 2"); // Direction2 0x4000

    if (naviStatus & (1 << 16))
        systemPrintln("Status: Filter determines GNSS data is invalid"); // GnssLost 0x10000
    if (naviStatus & (1 << 17))
        systemPrintln("Status: Initialization complete"); // FInitOk 0x20000
    // if (naviStatus & (1 << 18))
    //     systemPrintln("Status: PPS signal received"); // PPSReady 0x40000
    // if (naviStatus & (1 << 19))
    //     systemPrintln("Status: Module time synchronization successful"); // SyncReady 0x80000
    // if (naviStatus & (1 << 20)) //0x100000
    //     systemPrintln("Status: GNSS Connected"); //Module parses to RTK data "); // GnssConnect
    //     0x100000
    if (naviStatus > 0x1FFFFF)
    {
        // Clear all lower/known bits
        uint32_t bitsToShow = 0 ^ 0x1FFFFF;
        systemPrintf("Status: Unknown status bits set: 0x%04X\r\n", naviStatus & bitsToShow);
    }
}

// Called every main loop iteration — captures each new MEMS frame into the ring buffer
// and logs at 1 Hz so we can confirm data is flowing.
void tiltProcessMEMS()
{
    if (tiltSensor == nullptr || tiltSensor->packetMems == nullptr)
        return;
    if (tiltSensor->getMemsAge() > 20) // no frame in last 20 ms
        return;

    // Detect new frames by timestamp change
    static double lastSeen = -1.0;
    double t = tiltSensor->getMemsTimestamp();
    if (t == lastSeen)
        return;
    lastSeen = t;

    MemsFrame f = { t,
        tiltSensor->getMemsAccelX(), tiltSensor->getMemsAccelY(), tiltSensor->getMemsAccelZ(),
        tiltSensor->getMemsGyroX(),  tiltSensor->getMemsGyroY(),  tiltSensor->getMemsGyroZ() };
    // Push to ring buffer (inline to avoid Arduino prototype-injector issue with MemsFrame in sig)
    {
        uint8_t next = (memsHead + 1) & (MEMS_RING_SIZE - 1);
        if (next != memsTail) { memsRing[memsHead] = f; memsHead = next; }
    }

}

// Start communication with the IM19 IMU
void beginTilt()
{
    // Use UART2 on the ESP32 to receive IMU corrections
    // Shown as UART2 on these schematics: Torch, Facet FP
    tiltSensor = new IM19();
    if (SerialForTilt == nullptr)
        SerialForTilt = new HardwareSerial(2);

    SerialForTilt->setRxBufferSize(1024 * 4); // Extra room to absorb any MEMS stream

    // We must start the serial port before handing it over to the library
    SerialForTilt->begin(115200, SERIAL_8N1, pin_IMU_RX, pin_IMU_TX);

    // MEMS_OUTPUT=UART1,ON is saved to IM19 NVM and survives ESP32 reboot.
    // On second+ boot the IM19 blasts 100 Hz frames immediately, corrupting
    // the AT handshake in begin(). Send MEMS_OUTPUT=UART1,OFF first, then
    // drain the RX buffer before handing the port to the library.
    SerialForTilt->print("AT+MEMS_OUTPUT=UART1,OFF\r\n");
    delay(300);
    while (SerialForTilt->available()) SerialForTilt->read();

    if (settings.enableImuDebug == true)
        tiltSensor->enableDebugging(); // Print all debug to Serial

    if (tiltSensor->begin(*SerialForTilt) == false) // Give the serial port over to the library
    {
        tiltStop(); // Free memory
        return;
    }

    bool result = true;

    // The filter has a set of default parameters, which can be loaded when setting an error.
    result &= tiltSensor->sendCommand("LOAD_DEFAULT");

    // Use serial port 2 as the serial port for communication with GNSS
    result &= tiltSensor->sendCommand("GNSS_PORT=PHYSICAL_UART2");

    // Use serial port 1 as the main output with combined navigation data output
    result &= tiltSensor->sendCommand("NAVI_OUTPUT=UART1,ON");

    // If defined, set the IMU installation angle - before LEVER_ARM2
    // "the AT+INSTALL_ANGLE command must be sent firstly"
    if (strlen(variantHousingProperties->installAngle) > 0)
        result &= tiltSensor->sendCommand(variantHousingProperties->installAngle);

    // Set the LEVER_ARM(2) distance of the antenna ARP from the IMU
    result &= tiltSensor->sendCommand(variantHousingProperties->leverArm);

    // Set the overall length of the GNSS setup in meters: rod length 1800mm + internal length 96.45mm + antenna
    // POC 19.25mm = 1915.7mm
    char clubVector[strlen("CLUB_VECTOR=0,0,1.916") + 1];

    snprintf(clubVector, sizeof(clubVector), "CLUB_VECTOR=0,0,%0.3f",
             (settings.antennaHeight_mm + settings.antennaPhaseCenter_mm) / 1000.0);

    if (settings.enableImuCompensationDebug == true)
        systemPrintf("Setting club vector to: %s\r\n", clubVector);

    result &= tiltSensor->sendCommand(clubVector);

    // Configure interface type
    result &= tiltSensor->sendCommand(variantHousingProperties->gnssCard);

    // Configure as tilt measurement mode
    result &= tiltSensor->sendCommand("WORK_MODE=408"); // From stock firmware

    // AT+HIGH_RATE=[ENABLE | DISABLE] - try to slow down NAVI
    result &= tiltSensor->sendCommand("HIGH_RATE=DISABLE");

    // Unknown new command for v2
    result &= tiltSensor->sendCommand("CORRECT_HOLDER=ENABLE"); // From stock firmware

    // Trigger IMU on PPS from GNSS
    result &= tiltSensor->sendCommand("SET_PPS_EDGE=RISING");

    // Enable magnetic field mode
    // 'it is recommended to use the magnetic field initialization mode to speed up the initialization process'
    result &= tiltSensor->sendCommand("AHRS=ENABLE");

    result &= tiltSensor->sendCommand("MAG_AUTO_SAVE=ENABLE");

    if (result == true)
    {
        if (tiltSensor->saveConfiguration() == true)
        {
            // Enable raw MEMS output (100 Hz accel/gyro for IMU<->IMU time-sync with X5).
            // Done after saveConfiguration() so 100 Hz frames don't corrupt ACKs for other commands.
            bool memsOK = tiltSensor->sendCommand("MEMS_OUTPUT=UART1,ON");
            systemPrintf("MEMS_OUTPUT=UART1,ON: %s\r\n", memsOK ? "OK" : "FAILED");
            systemPrintln("Tilt sensor configuration complete");
            tiltState = TILT_STARTED;
            return; // Success
        }
    }

    tiltStop(); // Free memory
}

void tiltStop()
{
    // Gracefully stop the UART before freeing resources
    while (SerialForTilt->available())
        SerialForTilt->read();

    SerialForTilt->end();

    // Free the resources
    if (tiltSensor != nullptr)
    {
        delete tiltSensor;
        tiltSensor = nullptr;
    }

    if (SerialForTilt != nullptr)
    {
        delete SerialForTilt;
        SerialForTilt = nullptr;
    }

    if (tiltState == TILT_CORRECTING)
        beepDurationMs(1000); // Indicate we are going offline

    tiltState = TILT_OFFLINE;
}

// Called by other tasks. Prevents stopping serial port while within a library transaction.
void tiltRequestStop()
{
    tiltState = TILT_REQUEST_STOP;
}

bool tiltIsCorrecting()
{
    if (tiltState == TILT_CORRECTING)
        return (true);

    return (false);
}

// Restore the tilt sensor to factory settings
void tiltSensorFactoryReset()
{
    if (tiltState >= TILT_STARTED)
        tiltSensor->factoryReset();
}

// Given a NMEA sentence, modify the sentence to use the latest tilt-compensated lat/lon/alt
// Modifies the sentence directly. Updates sentence CRC.
// Auto-detects sentence type and will only modify sentences that have lat/lon/alt (ie GGA yes, GSV no)
// Which sentences have altitude? Yes: GGA, GNS No: RMC, GLL
// Which sentences have undulation? Yes: GGA, GNS No: RMC, GLL
// Four possible compensations:
// If tilt is active, and outputTipAltitude is enabled, then subtract undulation from IMU altitude, and apply LLA
// compensation. If tilt is active, and outputTipAltitude is disabled, then subtract undulation from IMU altitude, and
// add pole+ARP. If tilt is off, and outputTipAltitude is enabled, then subtract pole+ARP from altitude. If tilt is off,
// and outputTipAltitude is disabled, then pass GNSS data without modification. See issues:
//   https://github.com/sparkfun/SparkFun_RTK_Everywhere_Firmware/issues/334
//   https://github.com/sparkfun/SparkFun_RTK_Everywhere_Firmware/issues/343
void nmeaApplyCompensation(char *nmeaSentence, int sentenceLength)
{
    // If tilt is off, and outputTipAltitude is disabled, then pass GNSS data without modification
    if (tiltIsCorrecting() == false && settings.outputTipAltitude == false)
        return;

    // Identify sentence type
    char sentenceType[strlen("GGA") + 1] = {0};
    strncpy(sentenceType, &nmeaSentence[3],
            3); // Copy three letters, starting in spot 3. Null terminated from array initializer.

    // GGA and GNS sentences get modified in the same way
    if (strncmp(sentenceType, "GGA", sizeof(sentenceType)) == 0)
    {
        applyCompensationGGA(nmeaSentence, sentenceLength);
    }
    else if (strncmp(sentenceType, "GNS", sizeof(sentenceType)) == 0)
    {
        applyCompensationGNS(nmeaSentence, sentenceLength);
    }
    else if (strncmp(sentenceType, "RMC", sizeof(sentenceType)) == 0)
    {
        applyCompensationRMC(nmeaSentence, sentenceLength);
    }
    else if (strncmp(sentenceType, "GLL", sizeof(sentenceType)) == 0)
    {
        applyCompensationGLL(nmeaSentence, sentenceLength);
    }
    else
    {
        // This type of sentence does not have lat/lon/alt that needs modification
        return;
    }
}

// Modify a GNS sentence with tilt compensation
//$GNGNS,024034.00,4004.73854216,N,11614.19720023,E,ANAAA,28,0.8,1574.406,-8.4923,,,S*71 - Original
//$GNGNS,024034.00,4004.73854216,N,11614.19720023,E,ANAAA,28,0.8,1589.4793,-8.4923,,,S*7E - Modified
// 1580.987 is what is provided by the IMU and is the ellisoidal height
// 1580.987 is called 'ellipsoidal height' in SW Maps and includes the MSL + undulation
// To get mean sea level: 1580.987 - -8.4923 = 1589.4793
// 1589.4793 is the orthometric height in meters (MSL reference) that we need to insert into the NMEA sentence
// See issue: https://github.com/sparkfun/SparkFun_RTK_Everywhere_Firmware/issues/334
// https://support.virtual-surveyor.com/support/solutions/articles/1000261349-the-difference-between-ellipsoidal-geoid-and-orthometric-elevations-
void applyCompensationGNS(char *nmeaSentence, int sentenceLength)
{
    const int latitudeComma = 2;
    const int longitudeComma = 4;
    const int altitudeComma = 9;
    const int undulationComma = 10;

    uint8_t latitudeStart = 0;
    uint8_t latitudeStop = 0;
    uint8_t longitudeStart = 0;
    uint8_t longitudeStop = 0;
    uint8_t altitudeStart = 0;
    uint8_t altitudeStop = 0;
    uint8_t undulationStart = 0;
    uint8_t undulationStop = 0;
    uint8_t checksumStart = 0;

    if (settings.enableImuCompensationDebug == true && !inMainMenu)
        systemPrintf("Original GNGNS:\r\n%s\r\n", nmeaSentence);

    int commaCount = 0;
    for (int x = 0; x < strnlen(nmeaSentence, sentenceLength); x++) // Assumes sentence is null terminated
    {
        if (nmeaSentence[x] == ',')
        {
            commaCount++;
            if (commaCount == latitudeComma)
                latitudeStart = x + 1;
            if (commaCount == latitudeComma + 1)
                latitudeStop = x;
            if (commaCount == longitudeComma)
                longitudeStart = x + 1;
            if (commaCount == longitudeComma + 1)
                longitudeStop = x;
            if (commaCount == altitudeComma)
                altitudeStart = x + 1;
            if (commaCount == altitudeComma + 1)
                altitudeStop = x;
            if (commaCount == undulationComma)
                undulationStart = x + 1;
            if (commaCount == undulationComma + 1)
                undulationStop = x;
        }
        if (nmeaSentence[x] == '*')
        {
            checksumStart = x;
            break;
        }
    }

    if (latitudeStart == 0 || latitudeStop == 0 || longitudeStart == 0 || longitudeStop == 0 || altitudeStart == 0 ||
        altitudeStop == 0 || undulationStart == 0 || undulationStop == 0 || checksumStart == 0)
    {
        systemPrintln("Delineator not found");
        return;
    }

    // Extract the altitude
    char altitudeStr[strlen("-1602.3481") + 1]; // 4 decimals
    strncpy(altitudeStr, &nmeaSentence[altitudeStart], altitudeStop - altitudeStart);
    float altitude = (float)atof(altitudeStr);

    // Extract the undulation
    char undulationStr[strlen("-1602.3481") + 1]; // 4 decimals
    strncpy(undulationStr, &nmeaSentence[undulationStart], undulationStop - undulationStart);
    float undulation = (float)atof(undulationStr);

    char newSentence[150] = {0};

    if (sizeof(newSentence) < sentenceLength)
    {
        systemPrintln("newSentence not big enough!");
        return;
    }

    char coordinateStringDDMM[strlen("10511.12345678") + 1] = {0}; // UM980 outputs 8 decimals in GGA sentence

    // strncat terminates

    if (tiltIsCorrecting() == true)
    {
        // Add start of message up to latitude
        strncat(newSentence, nmeaSentence, latitudeStart);

        // Convert tilt-compensated latitude to DDMM
        coordinateConvertInput(abs(tiltSensor->getNaviLatitude()), COORDINATE_INPUT_TYPE_DDMM, coordinateStringDDMM,
                               sizeof(coordinateStringDDMM));

        // Check if latitude length has changed
        if (strlen(coordinateStringDDMM) != (latitudeStop - latitudeStart))
        {
            if (settings.enableImuCompensationDebug == true && !inMainMenu)
                systemPrintf("Compensated latitude length has changed! Orig: %d New: %d\r\n",
                             (latitudeStop - latitudeStart), strlen(coordinateStringDDMM));
        }

        // Add tilt-compensated Latitude
        strncat(newSentence, coordinateStringDDMM, sizeof(newSentence) - 1);

        // We can't allow the message length to change. Truncate if needed
        while (strlen(newSentence) > latitudeStop)
            *(newSentence + strlen(newSentence) - 1) = 0; // Move the NULL terminator

        // We can't allow the message length to change. Pad with zeros if needed
        while (strlen(newSentence) < latitudeStop)
            strncat(newSentence, "0", sizeof(newSentence) - 1);

        // Add interstitial between end of lat and beginning of lon
        strncat(newSentence, nmeaSentence + latitudeStop, longitudeStart - latitudeStop);

        // Convert tilt-compensated longitude to DDMM
        coordinateConvertInput(abs(tiltSensor->getNaviLongitude()), COORDINATE_INPUT_TYPE_DDMM, coordinateStringDDMM,
                               sizeof(coordinateStringDDMM));

        // Check if longitude length has changed
        if (strlen(coordinateStringDDMM) != (longitudeStop - longitudeStart))
        {
            if (settings.enableImuCompensationDebug == true && !inMainMenu)
                systemPrintf("Compensated longitude length has changed! Orig: %d New: %d\r\n",
                             (longitudeStop - longitudeStart), strlen(coordinateStringDDMM));
        }

        // Add tilt-compensated Longitude
        strncat(newSentence, coordinateStringDDMM, sizeof(newSentence) - 1);

        // We can't allow the message length to change. Truncate if needed
        while (strlen(newSentence) > longitudeStop)
            *(newSentence + strlen(newSentence) - 1) = 0; // Move the NULL terminator

        // We can't allow the message length to change. Pad with zeros if needed
        while (strlen(newSentence) < longitudeStop)
            strncat(newSentence, "0", sizeof(newSentence) - 1);

        // Add interstitial between end of lon and beginning of alt
        strncat(newSentence, nmeaSentence + longitudeStop, altitudeStart - longitudeStop);
    }
    else // No tilt compensation, no changes to the lat/lon
    {
        // Add start of message up to altitude
        strncat(newSentence, nmeaSentence, altitudeStart);
    }

    // Calculate newAltitude based on tilt mode and outputTipAltitude setting
    float newAltitude = 0;
    if (tiltIsCorrecting() == true)
    {
        // If tilt is active and outputTipAltitude is disabled, then subtract undulation from IMU altitude, and add
        // pole+ARP
        if (settings.outputTipAltitude == false)
            newAltitude = tiltSensor->getNaviAltitude() - undulation +
                          ((settings.antennaHeight_mm + settings.antennaPhaseCenter_mm) / 1000.0);

        // If tilt is active and outputTipAltitude is enabled, then subtract undulation from IMU altitude
        else if (settings.outputTipAltitude == true)
            newAltitude = tiltSensor->getNaviAltitude() - undulation;
    }
    else
    {
        // If tilt is off and outputTipAltitude is enabled, then subtract pole+ARP from altitude
        if (settings.outputTipAltitude == true)
            newAltitude = altitude - ((settings.antennaHeight_mm + settings.antennaPhaseCenter_mm) / 1000.0);

        // If tilt is off and outputTipAltitude is disabled, then we should not be here
    }

    // Convert altitude double to string
    snprintf(coordinateStringDDMM, sizeof(coordinateStringDDMM), "%0.3f", newAltitude);

    // Check if altitude length has changed
    if (strlen(coordinateStringDDMM) != (altitudeStop - altitudeStart))
    {
        if (settings.enableImuCompensationDebug == true && !inMainMenu)
            systemPrintf("Compensated altitude length has changed! Orig: %d New: %d\r\n",
                         (altitudeStop - altitudeStart), strlen(coordinateStringDDMM));
    }

    // Add tilt-compensated Altitude
    strncat(newSentence, coordinateStringDDMM, sizeof(newSentence) - 1);

    // We can't allow the message length to change. Truncate if needed
    // altitudeStop is the position of the comma.
    while (strlen(newSentence) > altitudeStop)
        *(newSentence + strlen(newSentence) - 1) = 0; // Move the NULL terminator

    // We can't allow the message length to change. Pad with zeros if needed
    while (strlen(newSentence) < altitudeStop)
        strncat(newSentence, "0", sizeof(newSentence) - 1);

    // Add remainder of the sentence up to checksum
    strncat(newSentence, nmeaSentence + altitudeStop, checksumStart - altitudeStop);

    // From: http://engineeringnotes.blogspot.com/2015/02/generate-crc-for-nmea-strings-arduino.html
    byte CRC = 0; // XOR chars between '$' and '*'
    for (byte x = 1; x < strlen(newSentence); x++)
        CRC = CRC ^ newSentence[x];

    // Convert CRC to string, add * and CR LF
    snprintf(coordinateStringDDMM, sizeof(coordinateStringDDMM), "*%02X\r\n", CRC);

    // Add CRC
    strncat(newSentence, coordinateStringDDMM, sizeof(newSentence) - 1);

    // Overwrite the original NMEA
    strncpy(nmeaSentence, newSentence, sentenceLength);

    if (settings.enableImuCompensationDebug == true && !inMainMenu)
        systemPrintf("Compensated GNGNS:\r\n%s\r\n", nmeaSentence);
}

// Modify a GLL sentence with tilt compensation
//$GNGLL,4005.4176871,N,10511.1034563,W,214210.00,A,A*68 - Original
//$GNGLL,4005.41769994,N,10507.40740734,W,214210.00,A,A*6D - Modified
void applyCompensationGLL(char *nmeaSentence, int sentenceLength)
{
    // GLL only needs to be changed in tilt mode
    if (tiltIsCorrecting() == false)
        return;

    if (settings.enableImuCompensationDebug == true && !inMainMenu)
        systemPrintf("Original GNGLL:\r\n%s\r\n", nmeaSentence);

    char coordinateStringDDMM[strlen("10511.12345678") + 1] = {0}; // UM980 outputs 8 decimals in GGA sentence

    const int latitudeComma = 1;
    const int longitudeComma = 3;

    uint8_t latitudeStart = 0;
    uint8_t latitudeStop = 0;
    uint8_t longitudeStart = 0;
    uint8_t longitudeStop = 0;
    uint8_t checksumStart = 0;

    int commaCount = 0;
    for (int x = 0; x < strnlen(nmeaSentence, sentenceLength); x++) // Assumes sentence is null terminated
    {
        if (nmeaSentence[x] == ',')
        {
            commaCount++;
            if (commaCount == latitudeComma)
                latitudeStart = x + 1;
            else if (commaCount == latitudeComma + 1)
                latitudeStop = x;
            else if (commaCount == longitudeComma)
                longitudeStart = x + 1;
            else if (commaCount == longitudeComma + 1)
                longitudeStop = x;
        }
        if (nmeaSentence[x] == '*')
        {
            checksumStart = x;
        }
    }

    if (latitudeStart == 0 || latitudeStop == 0 || longitudeStart == 0 || longitudeStop == 0 || checksumStart == 0)
    {
        systemPrintln("Delineator not found");
        return;
    }

    char newSentence[150] = {0};

    if (sizeof(newSentence) < sentenceLength)
    {
        systemPrintln("newSentence not big enough!");
        return;
    }

    // strncat terminates
    // Add start of message up to latitude
    strncat(newSentence, nmeaSentence, latitudeStart);

    // Convert tilt-compensated latitude to DDMM
    coordinateConvertInput(abs(tiltSensor->getNaviLatitude()), COORDINATE_INPUT_TYPE_DDMM, coordinateStringDDMM,
                           sizeof(coordinateStringDDMM));

    // Check if latitude length has changed
    if (strlen(coordinateStringDDMM) != (latitudeStop - latitudeStart))
    {
        if (settings.enableImuCompensationDebug == true && !inMainMenu)
            systemPrintf("Compensated latitude length has changed! Orig: %d New: %d\r\n",
                         (latitudeStop - latitudeStart), strlen(coordinateStringDDMM));
    }

    // Add tilt-compensated Latitude
    strncat(newSentence, coordinateStringDDMM, sizeof(newSentence) - 1);

    // We can't allow the message length to change. Truncate if needed
    while (strlen(newSentence) > latitudeStop)
        *(newSentence + strlen(newSentence) - 1) = 0; // Move the NULL terminator

    // We can't allow the message length to change. Pad with zeros if needed
    while (strlen(newSentence) < latitudeStop)
        strncat(newSentence, "0", sizeof(newSentence) - 1);

    // Add interstitial between end of lat and beginning of lon
    strncat(newSentence, nmeaSentence + latitudeStop, longitudeStart - latitudeStop);

    // Convert tilt-compensated longitude to DDMM
    coordinateConvertInput(abs(tiltSensor->getNaviLongitude()), COORDINATE_INPUT_TYPE_DDMM, coordinateStringDDMM,
                           sizeof(coordinateStringDDMM));

    // Check if longitude length has changed
    if (strlen(coordinateStringDDMM) != (longitudeStop - longitudeStart))
    {
        if (settings.enableImuCompensationDebug == true && !inMainMenu)
            systemPrintf("Compensated longitude length has changed! Orig: %d New: %d\r\n",
                         (longitudeStop - longitudeStart), strlen(coordinateStringDDMM));
    }

    // Add tilt-compensated Longitude
    strncat(newSentence, coordinateStringDDMM, sizeof(newSentence) - 1);

    // We can't allow the message length to change. Truncate if needed
    while (strlen(newSentence) > longitudeStop)
        *(newSentence + strlen(newSentence) - 1) = 0; // Move the NULL terminator

    // We can't allow the message length to change. Pad with zeros if needed
    while (strlen(newSentence) < longitudeStop)
        strncat(newSentence, "0", sizeof(newSentence) - 1);

    // Add remainder of the sentence up to checksum
    strncat(newSentence, nmeaSentence + longitudeStop, checksumStart - longitudeStop);

    // From: http://engineeringnotes.blogspot.com/2015/02/generate-crc-for-nmea-strings-arduino.html
    byte CRC = 0; // XOR chars between '$' and '*'
    for (byte x = 1; x < strlen(newSentence); x++)
        CRC = CRC ^ newSentence[x];

    // Convert CRC to string, add * and CR LF
    snprintf(coordinateStringDDMM, sizeof(coordinateStringDDMM), "*%02X\r\n", CRC);

    // Add CRC
    strncat(newSentence, coordinateStringDDMM, sizeof(newSentence) - 1);

    // Overwrite the original NMEA
    strncpy(nmeaSentence, newSentence, sentenceLength);

    if (settings.enableImuCompensationDebug == true && !inMainMenu)
        systemPrintf("Compensated GNGLL:\r\n%s\r\n", nmeaSentence);
}

// Modify a RMC sentence with tilt compensation
//$GNRMC,214210.00,A,4005.4176871,N,10511.1034563,W,0.000,,070923,,,A,V*04 - Original
//$GNRMC,214210.00,A,4005.41769994,N,10507.40740734,W,0.000,,070923,,,A,V*01 - Modified
void applyCompensationRMC(char *nmeaSentence, int sentenceLength)
{
    // RMC only needs to be changed in tilt mode
    if (tiltIsCorrecting() == false)
        return;

    if (settings.enableImuCompensationDebug == true && !inMainMenu)
        systemPrintf("Original GNRMC:\r\n%s\r\n", nmeaSentence);

    char coordinateStringDDMM[strlen("10511.12345678") + 1] = {0}; // UM980 outputs 8 decimals in GGA sentence

    const int latitudeComma = 3;
    const int longitudeComma = 5;

    uint8_t latitudeStart = 0;
    uint8_t latitudeStop = 0;
    uint8_t longitudeStart = 0;
    uint8_t longitudeStop = 0;
    uint8_t checksumStart = 0;

    int commaCount = 0;
    for (int x = 0; x < strnlen(nmeaSentence, sentenceLength); x++) // Assumes sentence is null terminated
    {
        if (nmeaSentence[x] == ',')
        {
            commaCount++;
            if (commaCount == latitudeComma)
                latitudeStart = x + 1;
            else if (commaCount == latitudeComma + 1)
                latitudeStop = x;
            else if (commaCount == longitudeComma)
                longitudeStart = x + 1;
            else if (commaCount == longitudeComma + 1)
                longitudeStop = x;
        }
        if (nmeaSentence[x] == '*')
        {
            checksumStart = x;
        }
    }

    if (latitudeStart == 0 || latitudeStop == 0 || longitudeStart == 0 || longitudeStop == 0 || checksumStart == 0)
    {
        systemPrintln("Delineator not found");
        return;
    }

    char newSentence[150] = {0};

    if (sizeof(newSentence) < sentenceLength)
    {
        systemPrintln("newSentence not big enough!");
        return;
    }

    // strncat terminates
    // Add start of message up to latitude
    strncat(newSentence, nmeaSentence, latitudeStart);

    // Convert tilt-compensated latitude to DDMM
    coordinateConvertInput(abs(tiltSensor->getNaviLatitude()), COORDINATE_INPUT_TYPE_DDMM, coordinateStringDDMM,
                           sizeof(coordinateStringDDMM));

    // Check if latitude length has changed
    if (strlen(coordinateStringDDMM) != (latitudeStop - latitudeStart))
    {
        if (settings.enableImuCompensationDebug == true && !inMainMenu)
            systemPrintf("Compensated latitude length has changed! Orig: %d New: %d\r\n",
                         (latitudeStop - latitudeStart), strlen(coordinateStringDDMM));
    }

    // Add tilt-compensated Latitude
    strncat(newSentence, coordinateStringDDMM, sizeof(newSentence) - 1);

    // We can't allow the message length to change. Truncate if needed
    while (strlen(newSentence) > latitudeStop)
        *(newSentence + strlen(newSentence) - 1) = 0; // Move the NULL terminator

    // We can't allow the message length to change. Pad with zeros if needed
    while (strlen(newSentence) < latitudeStop)
        strncat(newSentence, "0", sizeof(newSentence) - 1);

    // Add interstitial between end of lat and beginning of lon
    strncat(newSentence, nmeaSentence + latitudeStop, longitudeStart - latitudeStop);

    // Convert tilt-compensated longitude to DDMM
    coordinateConvertInput(abs(tiltSensor->getNaviLongitude()), COORDINATE_INPUT_TYPE_DDMM, coordinateStringDDMM,
                           sizeof(coordinateStringDDMM));

    // Check if longitude length has changed
    if (strlen(coordinateStringDDMM) != (longitudeStop - longitudeStart))
    {
        if (settings.enableImuCompensationDebug == true && !inMainMenu)
            systemPrintf("Compensated longitude length has changed! Orig: %d New: %d\r\n",
                         (longitudeStop - longitudeStart), strlen(coordinateStringDDMM));
    }

    // Add tilt-compensated Longitude
    strncat(newSentence, coordinateStringDDMM, sizeof(newSentence) - 1);

    // We can't allow the message length to change. Truncate if needed
    while (strlen(newSentence) > longitudeStop)
        *(newSentence + strlen(newSentence) - 1) = 0; // Move the NULL terminator

    // We can't allow the message length to change. Pad with zeros if needed
    while (strlen(newSentence) < longitudeStop)
        strncat(newSentence, "0", sizeof(newSentence) - 1);

    // Add remainder of the sentence up to checksum
    strncat(newSentence, nmeaSentence + longitudeStop, checksumStart - longitudeStop);

    // From: http://engineeringnotes.blogspot.com/2015/02/generate-crc-for-nmea-strings-arduino.html
    byte CRC = 0; // XOR chars between '$' and '*'
    for (byte x = 1; x < strlen(newSentence); x++)
        CRC = CRC ^ newSentence[x];

    // Convert CRC to string, add * and CR LF
    snprintf(coordinateStringDDMM, sizeof(coordinateStringDDMM), "*%02X\r\n", CRC);

    // Add CRC
    strncat(newSentence, coordinateStringDDMM, sizeof(newSentence) - 1);

    // Overwrite the original NMEA
    strncpy(nmeaSentence, newSentence, sentenceLength);

    if (settings.enableImuCompensationDebug == true && !inMainMenu)
        systemPrintf("Compensated GNRMC:\r\n%s\r\n", nmeaSentence);
}

// Modify a GGA sentence with tilt compensation
//$GNGGA,213441.00,4005.4176871,N,10511.1034563,W,1,12,99.99,1581.450,M,-21.3612,M,,*7D - Original
//$GNGGA,213441.00,4005.41769994,N,10507.40740734,W,1,12,99.99,1602.348,M,-21.3612,M,,*4C - Modified
// 1580.987 is what is provided by the IMU and is the ellisoidal height
//'Ellipsoidal height' includes the MSL + undulation
// To get mean sea level: 1580.987 - -21.3612 = 1602.3482
// 1602.3482 is the orthometric height in meters (MSL reference) that we need to insert into the NMEA sentence
// See issue: https://github.com/sparkfun/SparkFun_RTK_Everywhere_Firmware/issues/334
// https://support.virtual-surveyor.com/support/solutions/articles/1000261349-the-difference-between-ellipsoidal-geoid-and-orthometric-elevations-
void applyCompensationGGA(char *nmeaSentence, int sentenceLength)
{
    const int latitudeComma = 2;
    const int longitudeComma = 4;
    const int altitudeComma = 9;
    const int undulationComma = 11;

    uint8_t latitudeStart = 0;
    uint8_t latitudeStop = 0;
    uint8_t longitudeStart = 0;
    uint8_t longitudeStop = 0;
    uint8_t altitudeStart = 0;
    uint8_t altitudeStop = 0;
    uint8_t undulationStart = 0;
    uint8_t undulationStop = 0;
    uint8_t checksumStart = 0;

    if (settings.enableImuCompensationDebug == true && !inMainMenu)
        systemPrintf("Original GNGGA:\r\n%s\r\n", nmeaSentence);

    int commaCount = 0;
    for (int x = 0; x < strnlen(nmeaSentence, sentenceLength); x++) // Assumes sentence is null terminated
    {
        if (nmeaSentence[x] == ',')
        {
            commaCount++;
            if (commaCount == latitudeComma)
                latitudeStart = x + 1;
            if (commaCount == latitudeComma + 1)
                latitudeStop = x;
            if (commaCount == longitudeComma)
                longitudeStart = x + 1;
            if (commaCount == longitudeComma + 1)
                longitudeStop = x;
            if (commaCount == altitudeComma)
                altitudeStart = x + 1;
            if (commaCount == altitudeComma + 1)
                altitudeStop = x;
            if (commaCount == undulationComma)
                undulationStart = x + 1;
            if (commaCount == undulationComma + 1)
                undulationStop = x;
        }
        if (nmeaSentence[x] == '*')
        {
            checksumStart = x;
            break;
        }
    }

    if (latitudeStart == 0 || latitudeStop == 0 || longitudeStart == 0 || longitudeStop == 0 || altitudeStart == 0 ||
        altitudeStop == 0 || undulationStart == 0 || undulationStop == 0 || checksumStart == 0)
    {
        systemPrintln("Delineator not found");
        return;
    }

    // Extract the altitude
    char altitudeStr[strlen("-1602.3481") + 1]; // 4 decimals
    strncpy(altitudeStr, &nmeaSentence[altitudeStart], altitudeStop - altitudeStart);
    float altitude = (float)atof(altitudeStr);

    // Extract the undulation
    char undulationStr[strlen("-1602.3481") + 1]; // 4 decimals
    strncpy(undulationStr, &nmeaSentence[undulationStart], undulationStop - undulationStart);
    float undulation = (float)atof(undulationStr);

    char newSentence[150] = {0};

    if (sizeof(newSentence) < sentenceLength)
    {
        systemPrintln("newSentence not big enough!");
        return;
    }

    char coordinateStringDDMM[strlen("10511.12345678") + 1] = {0}; // UM980 outputs 8 decimals in GGA sentence

    // strncat terminates

    if (tiltIsCorrecting() == true)
    {
        // Add start of message up to latitude
        strncat(newSentence, nmeaSentence, latitudeStart);

        // Convert tilt-compensated latitude to DDMM
        coordinateConvertInput(abs(tiltSensor->getNaviLatitude()), COORDINATE_INPUT_TYPE_DDMM, coordinateStringDDMM,
                               sizeof(coordinateStringDDMM));

        // Check if latitude length has changed
        if (strlen(coordinateStringDDMM) != (latitudeStop - latitudeStart))
        {
            if (settings.enableImuCompensationDebug == true && !inMainMenu)
                systemPrintf("Compensated latitude length has changed! Orig: %d New: %d\r\n",
                             (latitudeStop - latitudeStart), strlen(coordinateStringDDMM));
        }

        // Add tilt-compensated Latitude
        strncat(newSentence, coordinateStringDDMM, sizeof(newSentence) - 1);

        // We can't allow the message length to change. Truncate if needed
        while (strlen(newSentence) > latitudeStop)
            *(newSentence + strlen(newSentence) - 1) = 0; // Move the NULL terminator

        // We can't allow the message length to change. Pad with zeros if needed
        while (strlen(newSentence) < latitudeStop)
            strncat(newSentence, "0", sizeof(newSentence) - 1);

        // Add interstitial between end of lat and beginning of lon
        strncat(newSentence, nmeaSentence + latitudeStop, longitudeStart - latitudeStop);

        // Convert tilt-compensated longitude to DDMM
        coordinateConvertInput(abs(tiltSensor->getNaviLongitude()), COORDINATE_INPUT_TYPE_DDMM, coordinateStringDDMM,
                               sizeof(coordinateStringDDMM));

        // Check if longitude length has changed
        if (strlen(coordinateStringDDMM) != (longitudeStop - longitudeStart))
        {
            if (settings.enableImuCompensationDebug == true && !inMainMenu)
                systemPrintf("Compensated longitude length has changed! Orig: %d New: %d\r\n",
                             (longitudeStop - longitudeStart), strlen(coordinateStringDDMM));
        }

        // Add tilt-compensated Longitude
        strncat(newSentence, coordinateStringDDMM, sizeof(newSentence) - 1);

        // We can't allow the message length to change. Truncate if needed
        while (strlen(newSentence) > longitudeStop)
            *(newSentence + strlen(newSentence) - 1) = 0; // Move the NULL terminator

        // We can't allow the message length to change. Pad with zeros if needed
        while (strlen(newSentence) < longitudeStop)
            strncat(newSentence, "0", sizeof(newSentence) - 1);

        // Add interstitial between end of lon and beginning of alt
        strncat(newSentence, nmeaSentence + longitudeStop, altitudeStart - longitudeStop);
    }
    else // No tilt compensation, no changes to the lat/lon
    {
        // Add start of message up to altitude
        strncat(newSentence, nmeaSentence, altitudeStart);
    }

    // Calculate newAltitude based on tilt mode and outputTipAltitude setting
    float newAltitude = 0;
    if (tiltIsCorrecting() == true)
    {
        // If tilt is active and outputTipAltitude is disabled, then subtract undulation from IMU altitude, and add
        // pole+ARP
        if (settings.outputTipAltitude == false)
            newAltitude = tiltSensor->getNaviAltitude() - undulation +
                          ((settings.antennaHeight_mm + settings.antennaPhaseCenter_mm) / 1000.0);

        // If tilt is active and outputTipAltitude is enabled, then subtract undulation from IMU altitude
        else if (settings.outputTipAltitude == true)
            newAltitude = tiltSensor->getNaviAltitude() - undulation;
    }
    else
    {
        // If tilt is off and outputTipAltitude is enabled, then subtract pole+ARP from altitude
        if (settings.outputTipAltitude == true)
            newAltitude = altitude - ((settings.antennaHeight_mm + settings.antennaPhaseCenter_mm) / 1000.0);

        // If tilt is off and outputTipAltitude is disabled, then we should not be here
    }

    // Convert altitude double to string
    snprintf(coordinateStringDDMM, sizeof(coordinateStringDDMM), "%0.4f", newAltitude);

    // Check if altitude length has changed
    if (strlen(coordinateStringDDMM) != (altitudeStop - altitudeStart))
    {
        if (settings.enableImuCompensationDebug == true && !inMainMenu)
            systemPrintf("Compensated altitude length has changed! Orig: %d New: %d\r\n",
                         (altitudeStop - altitudeStart), strlen(coordinateStringDDMM));
    }

    // Add tilt-compensated Altitude
    strncat(newSentence, coordinateStringDDMM, sizeof(newSentence) - 1);

    // We can't allow the message length to change. Truncate if needed
    // altitudeStop is the position of the comma.
    while (strlen(newSentence) > altitudeStop)
        *(newSentence + strlen(newSentence) - 1) = 0; // Move the NULL terminator

    // We can't allow the message length to change. Pad with zeros if needed
    while (strlen(newSentence) < altitudeStop)
        strncat(newSentence, "0", sizeof(newSentence) - 1);

    // Add remainder of the sentence up to checksum
    strncat(newSentence, nmeaSentence + altitudeStop, checksumStart - altitudeStop);

    // From: http://engineeringnotes.blogspot.com/2015/02/generate-crc-for-nmea-strings-arduino.html
    byte CRC = 0; // XOR chars between '$' and '*'
    for (byte x = 1; x < strlen(newSentence); x++)
        CRC = CRC ^ newSentence[x];

    // Convert CRC to string, add * and CR LF
    snprintf(coordinateStringDDMM, sizeof(coordinateStringDDMM), "*%02X\r\n", CRC);

    // Add CRC
    strncat(newSentence, coordinateStringDDMM, sizeof(newSentence) - 1);

    // Overwrite the original NMEA
    strncpy(nmeaSentence, newSentence, sentenceLength);

    if (settings.enableImuCompensationDebug == true && !inMainMenu)
        systemPrintf("Compensated GNGGA:\r\n%s\r\n", nmeaSentence);
}

// Determine if a tilt sensor is available or not
// Records outcome to NVM
void tiltDetect()
{
    // Only test housings that may have a tilt sensor on board
    if (variantHousingProperties->tiltPossible == false)
        return;

    // Skip test if previously detected as present
    if (settings.detectedTilt == true)
    {
        present.imu_im19 = true; // Allow tiltUpdate() to run
        return;
    }

    // Test for tilt only once
    if (settings.testedTilt == true)
        return;

    // Skip test if the FacetFP GNSS is unknown
    if (productVariant == RTK_FACET_FP)
    {
        if (settings.detectedGnssReceiver == GNSS_RECEIVER_UNKNOWN)
        {
            systemPrintln("FacetFP GNSS is unknown. Skipping tilt autodetection");
            settings.testedTilt = true;
            recordSystemSettings();
            return;
        }
    }

    systemPrintln("Beginning tilt autodetection");
    displayTiltAutodetect(0);

    // Locally instantiate the library and hardware so it will release on exit
    IM19 *tiltSensor;

    tiltSensor = new IM19();

    // On Facet FP, ESP UART2 is connected to SW3, then UART3 of the GNSS (where a tilt module resides, if populated)
    HardwareSerial SerialTiltTest(2); // Use UART2 on the ESP32 to communicate with IMU

    // Confirm SW3 is in the correct position
    gpioExpanderSelectImu();

    // We must start the serial port before handing it over to the library
    SerialTiltTest.begin(115200, SERIAL_8N1, pin_IMU_RX, pin_IMU_TX);

    if (settings.enableImuDebug == true)
        tiltSensor->enableDebugging(); // Print all debug to Serial

    // The IM19 requires ~2.5s from power up before it responds
    // The library will try twice with a 250ms
    // If communication fails, retry after a 3s timeout
    uint8_t maxTries = 2;
    for (int x = 0; x < maxTries; x++)
    {
        if (tiltSensor->begin(SerialTiltTest) == true)
        {
            present.imu_im19 = true; // Allow tiltUpdate() to run
            settings.detectedTilt = true;
            gnssConfigure(GNSS_CONFIG_TILT); // Request receiver to use new settings
            break;
        }

        if (x < (maxTries - 1))
            delay(3000);
    }

    SerialTiltTest.end(); // Release UART2 for reuse

    if (settings.detectedTilt == true)
        systemPrintln("Tilt sensor detected");
    else
    {
        systemPrintln("Tilt sensor not detected");
        displayTiltNotDetected(2000);
    }

    settings.testedTilt = true; // Record this test so we don't do it again
    recordSystemSettings();
    return;
}

#endif // COMPILE_IM19_IMU
