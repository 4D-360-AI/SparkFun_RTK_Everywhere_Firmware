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

// Complementary-filter roll/pitch (radians), fed by tiltComplementaryFilterUpdate()
// in the COMPILE_IM19_IMU block below. Declared here (file scope, unconditional) so
// memsBleUpdate() — compiled under COMPILE_BT, and appearing earlier in this file —
// can read them regardless of ifdef ordering. Default 0 matches the existing
// "IM19 not running yet" fallback used elsewhere in this file.
static float cfRollRad = 0.0f, cfPitchRad = 0.0f;

#ifdef COMPILE_BT
#include "BleSerialServer.h"
#include <BLE2902.h>

// UUIDs — all on the same 4d360001 service so the phone connects once.
#define MEMS_SERVICE_UUID "4d360001-0000-1000-8000-004d36000000"
#define MEMS_CHAR_UUID    "4d360002-0000-1000-8000-004d36000000"
#define RTCM_WRITE_UUID   "4d360003-0000-1000-8000-004d36000000"  // phone → Torch RTCM3
#define STATUS_CHAR_UUID  "4d360004-0000-1000-8000-004d36000000"  // Torch → phone fix status
#define GNSS_CTRL_UUID    "4d360005-0000-1000-8000-004d36000000"  // phone → Torch: 0x01=send file, 0x00=abort
#define GNSS_DATA_UUID    "4d360006-0000-1000-8000-004d36000000"  // Torch → phone: raw obs stream (post-capture)
#define GNSS_STREAM_UUID  "4d360007-0000-1000-8000-004d36000000"  // Torch → phone: live RTCM3 obs (during capture)

static BLECharacteristic *memsChar       = nullptr;
static BLECharacteristic *rtcmChar       = nullptr;
static BLECharacteristic *statusChar     = nullptr;
static BLECharacteristic *gnssCtrlChar   = nullptr;
static BLECharacteristic *gnssDataChar   = nullptr;
static BLECharacteristic *gnssStreamChar = nullptr;

// Raw GNSS byte capture (RTCM MSM7 + NMEA) streamed to LittleFS for PPK post-processing.
// Written by gnssReadTask; served to the phone via 4d360006 on demand.
static File           gnssRawFile;
static char           gnssRawFileName[64] = {0};
static volatile bool  gnssRawLogging = false;
static volatile bool  gnssXferActive = false;
static SemaphoreHandle_t gnssAckSem = nullptr; // released by [0x02] write from phone

// Called from gnssReadTask (Tasks.ino) — shields it from the static internals.
void gnssRawWriteBytes(const uint8_t *buf, size_t len)
{
    if (len == 0) return;
    if (gnssRawLogging && gnssRawFile)
        gnssRawFile.write(buf, len);
    // Live-stream to the phone during capture. Fire-and-forget notify is fine here:
    // RTCM3 arrives at ~1 Hz so the BLE queue never overflows. Muted during file
    // transfer to avoid mixing the post-capture download stream with live obs bytes.
    if (gnssStreamChar && !gnssXferActive)
    {
        gnssStreamChar->setValue((uint8_t *)buf, len);
        gnssStreamChar->notify();
    }
}

// Enable RTCM3 MSM7 rover output on the UM980 so the logged stream contains
// carrier-phase observations (needed for PPK post-processing).
// Messages: 1019 GPS ephemeris, 1077 GPS MSM7, 1087 GLO MSM7, 1097 GAL MSM7.
// gnssConfigure() only sets a bit in settings.gnssConfigureRequest — thread-safe.
// The main GNSS task polls that flag and does the actual UART work.
static void gnssEnableRtcmMsm7()
{
#ifdef COMPILE_UM980
    if (!present.gnss_um980) return;
    for (int x = 0; x < MAX_UM980_RTCM_MSG; x++)
    {
        const char *n = umMessagesRTCM[x].msgTextName;
        if (strcmp(n, "RTCM1019") == 0 || strcmp(n, "RTCM1077") == 0 ||
            strcmp(n, "RTCM1087") == 0 || strcmp(n, "RTCM1097") == 0)
            settings.um980MessageRatesRTCMRover[x] = 1;
    }
    gnssConfigure(GNSS_CONFIG_MESSAGE_RATE_RTCM_ROVER);
    systemPrintln("gnssRawFile: RTCM MSM7 rover output requested");
#endif
}

static void gnssDisableRtcmMsm7()
{
#ifdef COMPILE_UM980
    if (!present.gnss_um980) return;
    for (int x = 0; x < MAX_UM980_RTCM_MSG; x++)
        settings.um980MessageRatesRTCMRover[x] = 0;
    gnssConfigure(GNSS_CONFIG_MESSAGE_RATE_RTCM_ROVER);
    systemPrintln("gnssRawFile: RTCM MSM7 rover output disable requested");
#endif
}

static void gnssOpenRawFile()
{
    if (gnssRawFile) return;
    // Fixed name so FILE_WRITE always truncates the previous session's data.
    // RTC time is not needed: every RTCM3 message carries its own GPS time.
    strlcpy(gnssRawFileName, "/gnss_obs.rtcm3", sizeof(gnssRawFileName));
    gnssRawFile = LittleFS.open(gnssRawFileName, FILE_WRITE);
    if (!gnssRawFile)
    {
        gnssRawFileName[0] = 0;
        systemPrintln("gnssRawFile: open failed");
        return;
    }
    gnssRawLogging = true;
    systemPrintln("gnssRawFile: logging to /gnss_obs.rtcm3");
    gnssEnableRtcmMsm7();
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
            systemPrintf("gnssXfer: requested. fileName='%s' fileOpen=%d\r\n",
                         gnssRawFileName, gnssRawFile ? 1 : 0);
            if (gnssRawFile)
            {
                gnssRawFile.flush();
                gnssRawFile.close();
            }
            if (gnssDataChar == nullptr) return;
            if (gnssRawFileName[0] == 0)
            {
                systemPrintln("gnssXfer: no file — sending zero header");
                uint8_t zero[4] = {0};
                gnssDataChar->setValue(zero, 4);
                gnssDataChar->notify();
                return;
            }
            if (!gnssXferActive)
            {
                // Set flag here, before xTaskCreate, to close the race with
                // memsBleUpdate() which would otherwise reopen (and truncate)
                // the file before the task gets a chance to read it.
                gnssXferActive = true;
                xTaskCreate(gnssXferTask, "gnssXfer", 8192, nullptr, 1, nullptr);
            }
        }
        else if (cmd == 0x02) // ACK: phone ready for next chunk
        {
            if (gnssAckSem) xSemaphoreGive(gnssAckSem);
        }
        else if (cmd == 0x00) // abort
        {
            gnssXferActive = false;
            if (gnssAckSem) xSemaphoreGive(gnssAckSem); // unblock task so it can exit
        }
    }
};
static GnssCtrlCallback gnssCtrlCb;

// FreeRTOS task: streams the most recent raw obs file over BLE in 500-byte chunks.
// Protocol (stop-and-wait):
//   Torch → phone: first notification = [fileSize: u32 LE][data...]; subsequent = [data...]
//   Phone → Torch: [0x02] write on ctrl char after each chunk received (ACK / next-chunk request)
// This prevents BLE notify queue overflow that caused silent packet drops at ~145 KB.
static void gnssXferTask(void *e)
{
    gnssXferActive = true;
    gnssAckSem = xSemaphoreCreateBinary();
    gnssDisableRtcmMsm7(); // Stop UM980 raw obs output before reading the file

    File f = LittleFS.open(gnssRawFileName, FILE_READ);
    systemPrintf("gnssXfer: open '%s' ok=%d\r\n", gnssRawFileName, f ? 1 : 0);
    if (!f)
    {
        systemPrintln("gnssXfer: file open failed — sending zero header");
        uint8_t zero[4] = {0};
        gnssDataChar->setValue(zero, 4);
        gnssDataChar->notify();
        vSemaphoreDelete(gnssAckSem);
        gnssAckSem = nullptr;
        gnssXferActive = false;
        vTaskDelete(nullptr);
        return;
    }

    uint32_t fileSize = (uint32_t)f.size();
    systemPrintf("gnssXfer: fileSize=%u bytes\r\n", (unsigned)fileSize);
    // 500 bytes fits within negotiated MTU of 517 (514 max GATT payload).
    const uint16_t CHUNK = 500;
    uint8_t buf[500 + 4];

    bool ok = true;

    // First packet: 4-byte size header + up to CHUNK bytes of data.
    buf[0] = (uint8_t)(fileSize & 0xFF);
    buf[1] = (uint8_t)((fileSize >>  8) & 0xFF);
    buf[2] = (uint8_t)((fileSize >> 16) & 0xFF);
    buf[3] = (uint8_t)((fileSize >> 24) & 0xFF);
    int n = f.read(buf + 4, CHUNK);
    if (n > 0)
    {
        gnssDataChar->setValue(buf, (size_t)(4 + n));
        gnssDataChar->notify();
        // Wait up to 5 s for phone to ACK before sending next chunk.
        if (xSemaphoreTake(gnssAckSem, pdMS_TO_TICKS(5000)) != pdTRUE)
        {
            systemPrintln("gnssXfer: ACK timeout on first chunk — aborting");
            ok = false;
        }
    }

    while (ok && gnssXferActive)
    {
        n = f.read(buf, CHUNK);
        if (n <= 0) break;
        gnssDataChar->setValue(buf, (size_t)n);
        gnssDataChar->notify();
        if (xSemaphoreTake(gnssAckSem, pdMS_TO_TICKS(5000)) != pdTRUE)
        {
            systemPrintf("gnssXfer: ACK timeout after %u bytes — aborting\r\n",
                         (unsigned)f.position());
            break;
        }
    }

    f.close();
    systemPrintln("gnssXfer: transfer complete");

    vSemaphoreDelete(gnssAckSem);
    gnssAckSem = nullptr;

    // Free the space and immediately start a fresh obs file for the next capture.
    // Clear gnssXferActive AFTER gnssOpenRawFile() so that memsBleUpdate() cannot
    // win the race between this task's remove() and open() calls.
    LittleFS.remove(gnssRawFileName);
    gnssRawFileName[0] = 0;
    if (online.fs)
        gnssOpenRawFile();
    gnssXferActive = false;

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

    // Handle space: 1 service + 3+2+3+2+3+3 chars/descriptors = 17 → use 30 for headroom.
    BLEService *svc = srv->createService(BLEUUID(MEMS_SERVICE_UUID), 30);

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

    // 4d360006 — raw obs data stream (notify, Torch → phone, post-capture file download)
    gnssDataChar = svc->createCharacteristic(BLEUUID(GNSS_DATA_UUID),
                                              BLECharacteristic::PROPERTY_NOTIFY);
    gnssDataChar->addDescriptor(new BLE2902());

    // 4d360007 — live RTCM3 obs stream (notify, Torch → phone, during capture)
    gnssStreamChar = svc->createCharacteristic(BLEUUID(GNSS_STREAM_UUID),
                                                BLECharacteristic::PROPERTY_NOTIFY);
    gnssStreamChar->addDescriptor(new BLE2902());

    svc->start();

    BLEAdvertising *adv = srv->getAdvertising();
    adv->addServiceUUID(BLEUUID(MEMS_SERVICE_UUID));
    adv->start();

    memsBleInited = true;
    systemPrintln("MEMS BLE ready: IMU(4d360002) RTCM(4d360003) Status(4d360004) GNSS(4d360005/06)");

    // Open the obs log immediately — beginFS() runs in setup long before BLE,
    // so online.fs is guaranteed true here. This decouples obs logging from the
    // tilt state machine (which may not run if tilt compensation is disabled).
    if (online.fs)
        gnssOpenRawFile();
}

void memsBleUpdate()
{
    if (!memsBleInited || memsChar == nullptr)
        return;

    // ── Raw obs log — open once LittleFS is ready, but never while a transfer
    // is in progress (would truncate the file the xfer task is about to read).
    if (!gnssRawFile && !gnssXferActive && online.fs)
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

    // ── Fix status (10 Hz) ──────────────────────────────────────────────────
    if (statusChar == nullptr || gnss == nullptr) return;

    static unsigned long lastStatus = 0;
    if (millis() - lastStatus < 100) return;
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
    // RECTIMEB fires at 1 Hz on the whole-second boundary (ms≈0). Anchor it to
    // ESP32 millis() when the second changes, then dead-reckon at the 100 ms BLE
    // rate. This gives 1 ms sub-second precision without any library changes.
    static uint32_t rectimeAnchorMillis = 0; // millis() when RECTIMEB last ticked
    static uint32_t rectimeSodMs        = 0; // UTC ms-of-day at that tick
    static uint8_t  rectimeLastSec      = 255;

    float tUtc = 0.0f;
    if (gnss->isConfirmedTime())
    {
        uint8_t nowSec = gnss->getSecond();
        if (nowSec != rectimeLastSec)
        {
            // New RECTIMEB epoch — refresh the dead-reckoning anchor.
            rectimeLastSec      = nowSec;
            rectimeAnchorMillis = millis();
            rectimeSodMs        = (uint32_t)gnss->getHour()   * 3600000UL
                                + (uint32_t)gnss->getMinute() *   60000UL
                                + (uint32_t)nowSec            *    1000UL
                                + gnss->getMillisecond();
        }
        if (rectimeAnchorMillis != 0)
        {
            uint32_t sodMs = rectimeSodMs + (millis() - rectimeAnchorMillis);
            if (sodMs >= 86400000UL) sodMs -= 86400000UL; // midnight wrap
            tUtc = sodMs / 1000.0f;
        }
    }

    // Roll/pitch from the raw-accel/gyro complementary filter (tiltOnMemsFrame()) —
    // not the IM19 NAVI Kalman output, which needs a hand-shake init a vehicle
    // mount can't perform. Heading still comes from NAVI (0.0 when uninitialized,
    // which is expected here); the app falls back to GNSS course_deg for heading.
    float rollDeg    = cfRollRad * RAD_TO_DEG;
    float pitchDeg   = cfPitchRad * RAD_TO_DEG;
    float headingDeg = tiltSensor != nullptr ? tiltSensor->getNaviHeading()    : 0.0f;

    // Ellipsoidal altitude (m) from GNSS; IM19 NAVI status word for convergence gating.
    float    altM      = gnss->getAltitude();
    uint32_t naviStat  = tiltSensor != nullptr ? tiltSensor->getNaviStatus() : 0;

    // Doppler velocity from BESTNAV (GNSS source).
    float speedMps  = (float)gnss->getHorizontalSpeed(); // horizontal speed, m/s
    float courseDeg = (float)gnss->getTrackGround();     // course over ground, deg CW from N
    float velDMps   = -(float)gnss->getVerticalSpeed();  // vertical, m/s positive-DOWN
    float sVelMps   = gnss->getSpeedDeviation();         // horizontal speed accuracy, m/s

    uint8_t status[52];
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
    memcpy(&status[12], &tUtc,       4); // float32 LE
    memcpy(&status[16], &rollDeg,    4); // float32 LE, +right-side-down
    memcpy(&status[20], &pitchDeg,   4); // float32 LE, +nose-up
    memcpy(&status[24], &headingDeg, 4); // float32 LE, true heading 0=N 90=E
    memcpy(&status[28], &altM,       4); // float32 LE, ellipsoidal altitude (m)
    memcpy(&status[32], &naviStat,   4); // uint32 LE, IM19 NAVI status word
    memcpy(&status[36], &speedMps,   4); // float32 LE, horizontal speed (m/s)
    memcpy(&status[40], &courseDeg,  4); // float32 LE, course over ground (deg CW from N)
    memcpy(&status[44], &velDMps,    4); // float32 LE, vertical speed positive-down (m/s)
    memcpy(&status[48], &sVelMps,    4); // float32 LE, horizontal speed accuracy (m/s)
    statusChar->setValue(status, 52);
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
        tiltSensor->update();   // frames reach the ring via tiltOnMemsFrame(), not by polling
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
        tiltSensor->update(); // drains UART; every MEMS frame lands via tiltOnMemsFrame()
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
        tiltSensor->update(); // drains UART; every MEMS frame lands via tiltOnMemsFrame()
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

// Complementary-filter roll/pitch, derived from the raw 100 Hz MEMS accel/gyro
// stream instead of the IM19's Kalman NAVI output. The NAVI filter requires a
// hand-shake init sequence (see datasheet steps in tiltUpdate() above) that a
// vehicle-fixed mount can never perform, so it never leaves TILT_INITIALIZED.
// This filter needs no init gesture: gyro integration gives continuous fast
// response, and the accelerometer's gravity vector pulls out long-term drift
// whenever the vehicle isn't under significant non-gravity acceleration
// (braking/accelerating/cornering), which is when the accel reading stops
// looking like 1 g and the correction is skipped for that sample.
// Sign convention: roll +right-side-down, pitch +nose-down (memsBleUpdate()
// below) — verified on bench 2026-08-05. (Torch
// flat: expect az≈+1g, ax≈ay≈0) before trusting the sign in the field, since
// it assumes the IMU's Z axis is vertical when the unit sits normally in its
// mount. cfRollRad/cfPitchRad themselves are declared at file scope near the
// top of this file (read by memsBleUpdate(), which appears earlier).
static bool cfInited = false;

static void tiltComplementaryFilterUpdate(float ax, float ay, float az, float gx, float gy, float dt)
{
    // The Torch is mounted at 45° yaw in the vehicle: one corner faces forward,
    // one back, one left, one right. Rotate IMU body-frame accel and gyro into
    // vehicle frame (FLU: x=forward, y=left, z=up) before computing roll/pitch.
    // Bench-verified 2026-08-05: nose-down gives ax>0,ay>0 → vehicle forward
    // aligns with IMU (x+y)/√2, so the rotation angle is +45°.
    // az is the vertical axis and is unaffected by a yaw rotation.
    // az≈-1g when level (IMU Z inverted from textbook); -az is used below.
    const float c45 = 0.70711f; // cos(45°) = sin(45°) = 1/√2
    float ax_v = (ax + ay) * c45;  // vehicle forward
    float ay_v = (-ax + ay) * c45; // vehicle left
    float gx_v = (gx + gy) * c45; // roll rate  (around vehicle forward axis)
    float gy_v = (-gx + gy) * c45;// pitch rate (around vehicle lateral axis)

    float accMag = sqrtf(ax_v * ax_v + ay_v * ay_v + az * az);
    float accelRoll  = atan2f(ay_v, -az);
    float accelPitch = atan2f(ax_v, sqrtf(ay_v * ay_v + az * az));

    if (!cfInited)
    {
        cfRollRad = accelRoll;
        cfPitchRad = accelPitch;
        cfInited = true;
        return;
    }

    // Gyro integration every sample (rad/s * s = rad) — carries us through
    // the periods where the accel correction below is gated out.
    cfRollRad  += gx_v * dt;
    cfPitchRad += gy_v * dt;

    // Only trust the accelerometer as "down" when it's close to 1 g — otherwise
    // the vehicle is accelerating/braking/turning and the reading isn't gravity.
    if (fabsf(accMag - 1.0f) < 0.1f)
    {
        const float alpha = 0.98f; // gyro:accel trust ratio per correction step
        cfRollRad  = alpha * cfRollRad  + (1.0f - alpha) * accelRoll;
        cfPitchRad = alpha * cfPitchRad + (1.0f - alpha) * accelPitch;
    }
}

// Called by the IM19 parser for EVERY MEMS frame, not once per main loop.
//
// It used to be called from the loop and read whatever was latest in packetMems. That caps
// capture at the LOOP rate: update() parses every frame that arrived, but each overwrites
// the single packetMems struct, so one poll yields one frame however many turned up. The
// RTK Everywhere loop runs near 70 Hz, and 019ffbe7 duly recorded 2814 of the IM19's 4043
// frames — 69.9 Hz of a true 100 Hz, with 985 single-frame gaps to 56 doubles, which is a
// poll one frame late rather than anything dropping on the radio. The BLE link was at 55%
// of capacity throughout, so it was never the transport.
//
// Runs in parser context: push to the ring and return. No serial, no BLE, no logging.
void tiltOnMemsFrame(const IM19_MEMS_data_t *src)
{
    double t = src->timestamp;

    // Detect new frames by timestamp change
    static double lastSeen = -1.0;
    if (t == lastSeen)
        return;
    double dt = (lastSeen >= 0.0) ? (t - lastSeen) : 0.0;
    lastSeen = t;

    // Named `src`, not `f`: the ring push below builds a local MemsFrame f, and the
    // shadowing is a hard error under the project's -Werror settings.
    float ax = src->accelX, ay = src->accelY, az = src->accelZ;
    float gx = src->gyroX,  gy = src->gyroY,  gz = src->gyroZ;

    // Clamp dt so a startup sample or a stall doesn't inject a huge gyro step.
    if (dt > 0.0 && dt < 0.05)
        tiltComplementaryFilterUpdate(ax, ay, az, gx, gy, (float)dt);

    MemsFrame f = { t, ax, ay, az, gx, gy, gz };
    // Push to ring buffer (inline to avoid Arduino prototype-injector issue with MemsFrame in sig)
    {
        uint8_t next = (memsHead + 1) & (MEMS_RING_SIZE - 1);
        if (next != memsTail) { memsRing[memsHead] = f; memsHead = next; }
    }

    // Bench-test aid: 1 Hz print of raw accel + the complementary-filter roll/pitch,
    // so the sign convention can be checked over USB serial without app changes.
    // Enable via the menu's IMU debug toggle (same flag as printTiltDebug()).
    if (settings.enableImuDebug == true)
    {
        static unsigned long lastPrint = 0;
        if (millis() - lastPrint > 1000)
        {
            lastPrint = millis();
            systemPrintf("MEMS accel g(x,y,z)=(%0.3f,%0.3f,%0.3f) roll=%0.1f pitch=%0.1f\r\n",
                         ax, ay, az, cfRollRad * RAD_TO_DEG, cfPitchRad * RAD_TO_DEG);
        }
    }
}

// Start communication with the IM19 IMU
void beginTilt()
{
    // Use UART2 on the ESP32 to receive IMU corrections
    // Shown as UART2 on these schematics: Torch, Facet FP
    tiltSensor = new IM19();
    // EVERY frame, not one per loop. See tiltOnMemsFrame() for why polling costs ~30%.
    tiltSensor->setMemsCallback(tiltOnMemsFrame);
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
