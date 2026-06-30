#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <esp_timer.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <Adafruit_SHT31.h>
#include <Adafruit_Sensor.h>
#include <math.h>
#include <RTClib.h>
#include <sys/time.h>
#include <time.h>
#include <TinyGPSPlus.h>

/* -----------------------------------------------------------------
####################################################################
------------------------ PIN DEFINITIONS ---------------------------
####################################################################
----------------------------------------------------------------- */

/* ------------------------- UART pins -------------------------- */
static const int UART_RX = 18;  // RX, connected to payload TX
static const int UART_TX = 17;  // TX, connected to payload RX
static const uint32_t UART_BAUD = 115200; // 1 Mbps for stress test sender, 115200 for Li-560 anemometer

/* ------------------------ SD SPI pins ------------------------- */
static const int PIN_SCK  = 12;
static const int PIN_MISO = 13;
static const int PIN_MOSI = 11;
static const int PIN_CS   = 10;

/* ---------------------- BNO08x I2C pins ----------------------- */
static const int PIN_SDA  = 8;
static const int PIN_SCL  = 9;
#define BNO08X_INT    7      // optional interrupt, not used for I2C polling
#define BNO08X_RESET -1      // no RESET pin for I2C

/* ------------------------- GY-NEO-8M -------------------------- */
static const int GPS_RX_PIN = 4;  // ESP32 receives on this pin (connect to GPS TX)
static const int GPS_TX_PIN = 5;  // ESP32 transmits on this pin (connect to GPS RX)
static const uint32_t GPS_BAUD = 9600;

/* -----------------------------------------------------------------
####################################################################
----------------- GLOBAL VARIABLES DEFINITIONS ---------------------
####################################################################
----------------------------------------------------------------- */


/* -------------------------- BNO08x ---------------------------- */
Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

// Simple container for Euler angles
struct euler_t { float yaw, pitch, roll; };
static euler_t ypr;

// For all data to be stored on the SD card
typedef struct {
  uint32_t t_us;                 // The last assembly timestamp (updated everytime the info is updated)
  int32_t qw, qx, qy, qz;        // Quaternion
  int32_t yaw, pitch, roll;      // deg*1e2
  int32_t ax, ay, az;            // m/s^2*1e6
  int32_t gx, gy, gz;            // rad/s*1e4 (?)

  uint8_t rv_status;              // rotation vector accuracy class
  uint8_t acc_status;             // linear accel status
  uint8_t gyr_status;             // gyro status

  uint32_t bno_ts_us;             // lower 32 bits of sensorValue.timestamp (us-ish)
} LatestBNO_t;

// -------- gather state --------
static LatestBNO_t LatestBNO;    // being-filled sample

// ESP32 critical section for consistent snapshots
portMUX_TYPE latestMux = portMUX_INITIALIZER_UNLOCKED;

/* -------------------------- SHT3x ----------------------------- */
Adafruit_SHT31 sht31 = Adafruit_SHT31();

// For the temp & humidity data
typedef struct {
  uint32_t  t_us;                 // assembly timestamp
  float     temp;                 // temperature in C
  float     humidity;             // relative humidity
} LatestXHT_t;

// -------- gather state --------
static LatestXHT_t LatestXHT;     // being sampled at intervals

/* ----------------------- DS3231 Timer ------------------------- */
RTC_DS3231 rtc;

/* -------------------------- GPS ------------------------------- */
TinyGPSPlus gps;
HardwareSerial GPSSerial(2);  // UART2

// Validity bits (field-by-field)
enum : uint16_t {
  GPS_LOC_BIT  = 1u << 0,
  GPS_ALT_BIT  = 1u << 1,
  GPS_SPD_BIT  = 1u << 2,
  GPS_SATS_BIT = 1u << 3,
  GPS_HDOP_BIT = 1u << 4,
  GPS_UTC_BIT  = 1u << 5,
};

typedef struct {
  uint32_t t_us;         // when we assembled this snapshot (micros)
  uint16_t validMask;    // which fields are valid
  uint16_t age_ms;       // TinyGPSPlus "age" of location in ms (0..~65535)

  int32_t  lat_e7;       // deg * 1e7
  int32_t  lon_e7;       // deg * 1e7
  int32_t  alt_cm;       // meters -> cm (or 0 if invalid)
  uint32_t spd_cms;      // m/s -> cm/s
  uint16_t hdop_centi;   // hdop * 100
  uint8_t  sats;

  // UTC (from GPS) — optional but handy for alignment
  uint16_t year;
  uint8_t  month, day, hour, minute, second;
} LatestGPS_t;

static LatestGPS_t LatestGPS = {};
LatestGPS_t GPSsnap;
portMUX_TYPE gpsMux = portMUX_INITIALIZER_UNLOCKED;

/* ------------------------- Logging ---------------------------- */
File logFile;
const char* FNAME = "/CUES_log.csv";
char filename[64];

#define LINE_MAX  256   // Max length received with each UART RX
#define Q_CAP     256   // Max num of messages stored in RAM

// Structure for the logging of messages
typedef struct {
  uint64_t    t_rtc_s;            // constructed RTC time
  uint64_t    t_rtc_us;           // epoch microseconds (for alignment with GPS)
  uint32_t    t_log_us;           // micros() at log moment (for local timing)
  LatestBNO_t bno;                // snapshot of BNO fields
  LatestXHT_t xht;                // snapshot of XHT fields
  LatestGPS_t gps;
} Line;

// Ring buffer
volatile uint16_t qHead=0, qTail=0;
Line q[Q_CAP];

// Default logging rate: 10 Hz (change as needed)
#define LOG_HZ 10

// Derived interval (microseconds). LOG_HZ must be > 0.
#define LOG_INTERVAL_US (1000000UL / (LOG_HZ))

/* --------------- Wireless CC2530 DL-20 Family ----------------- */
HardwareSerial Radio(1);
char txBuf[256];
size_t wireless_n;

/* -----------------------------------------------------------------
####################################################################
----------------------- FUNCTION DEFINITIONS -----------------------
####################################################################
----------------------------------------------------------------- */

/* ---------------------- integrated LED ------------------------ */
#ifndef STATUS_LED_H
#define STATUS_LED_H

// Some boards have GRB color order. If colors look wrong, set this to 1.
// The ESP32-S3-WROOM1 N8R2 has RGB order
#ifndef RGB_GRB_ORDER
#define RGB_GRB_ORDER 0
#endif

// Thin wrapper so colour can be swapped if needed.
static inline void rgbWrite(uint8_t r, uint8_t g, uint8_t b) {
  #if RGB_GRB_ORDER
    neopixelWrite(RGB_BUILTIN, g, r, b);   // GRB -> pass G,R,B
  #else
    neopixelWrite(RGB_BUILTIN, r, g, b);   // RGB
  #endif
}

static inline void rgbOff() { rgbWrite(0,0,0); }

// Handy color macros
#define MAX_BRIGHTNESS  31
#define RGB_RED()       rgbWrite(MAX_BRIGHTNESS,0,0)
#define RGB_GREEN()     rgbWrite(0,MAX_BRIGHTNESS,0)
#define RGB_BLUE()      rgbWrite(0,0,MAX_BRIGHTNESS)
#define RGB_WHITE()     rgbWrite(MAX_BRIGHTNESS,MAX_BRIGHTNESS,MAX_BRIGHTNESS)

//       ------------ Non-blocking blinker -----------
struct RgbBlinker {
  bool   enabled = false;
  bool   onPhase = false;
  uint32_t on_ms = 0, off_ms = 0;
  uint8_t r=0,g=0,b=0;
  uint32_t t0 = 0;

  void start(uint8_t r_, uint8_t g_, uint8_t b_, uint32_t on_ms_, uint32_t off_ms_) {
    enabled = true; onPhase = true;
    r=r_; g=g_; b=b_; on_ms=on_ms_; off_ms=off_ms_;
    t0 = millis(); rgbWrite(r,g,b);
  }

  void stop() { enabled=false; rgbOff(); }

  void poll() {
    if(!enabled) return;
    uint32_t dt = millis() - t0;
    if(onPhase && dt >= on_ms) {
      onPhase=false; t0=millis(); rgbOff();
    } else if(!onPhase && dt >= off_ms) {
      onPhase=true;  t0=millis(); rgbWrite(r,g,b);
    }
  }

  void resume() { 
    enabled = true; onPhase = true;
    t0 = millis(); rgbWrite(r,g,b);
  }

  void changeColour(uint8_t r_, uint8_t g_, uint8_t b_) {
    r=r_; g=g_; b=b_;
    // Update immediately if we're currently lit
    if (enabled && onPhase) rgbWrite(r, g, b);
  }
};

RgbBlinker StatusBlinker; // Global blinker instance

// Convenience helpers
static inline void statusBlinkGreen(uint32_t on_ms=120, uint32_t off_ms=380) {
  StatusBlinker.start(0,MAX_BRIGHTNESS,0,on_ms,off_ms);
}
static inline void statusBlinkRed(uint32_t on_ms=120, uint32_t off_ms=380) {
  StatusBlinker.start(MAX_BRIGHTNESS,0,0,on_ms,off_ms);
}
static inline void statusSolidGreen() { StatusBlinker.stop(); RGB_GREEN(); }
static inline void statusSolidRed()   { StatusBlinker.stop(); RGB_RED(); }

#endif // STATUS_LED_H

/* --------------------------- GPS ------------------------------ */
static inline int32_t deg_e7(double deg) { return (int32_t)llround(deg * 1e7); }

// Read/decode GPS bytes for up to budget_us, then update LatestGPS if new info arrived.
static inline void gpsPoll(uint32_t budget_us = 300) {
  uint32_t t0 = micros();
  bool anyNew = false;

  while (GPSSerial.available()) {
    anyNew |= gps.encode(GPSSerial.read());
    if (budget_us && (micros() - t0) >= budget_us) break;
  }

  // Only touch LatestGPS when new sentences were parsed (keeps it cheap)
  if (!anyNew) return;

  LatestGPS_t g = {};
  g.t_us = micros();

  // Location
  if (gps.location.isValid()) {
    g.validMask |= GPS_LOC_BIT;
    g.lat_e7 = deg_e7(gps.location.lat());
    g.lon_e7 = deg_e7(gps.location.lng());

    uint32_t age = gps.location.age();              // ms since last valid location
    g.age_ms = (age > 65535u) ? 65535u : (uint16_t)age;
  } else {
    g.age_ms = 65535;
  }

  // Altitude
  if (gps.altitude.isValid()) {
    g.validMask |= GPS_ALT_BIT;
    g.alt_cm = (int32_t)llround(gps.altitude.meters() * 100.0);
  }

  // Speed
  if (gps.speed.isValid()) {
    g.validMask |= GPS_SPD_BIT;
    g.spd_cms = (uint32_t)llround(gps.speed.mps() * 100.0);
  }

  // Satellites
  if (gps.satellites.isValid()) {
    g.validMask |= GPS_SATS_BIT;
    g.sats = (uint8_t)gps.satellites.value();
  }

  // HDOP
  if (gps.hdop.isValid()) {
    g.validMask |= GPS_HDOP_BIT;
    g.hdop_centi = (uint16_t)gps.hdop.value(); // TinyGPSPlus hdop.value() is already *100
  }

  // UTC time/date
  if (gps.date.isValid() && gps.time.isValid()) {
    g.validMask |= GPS_UTC_BIT;
    g.year   = (uint16_t)gps.date.year();
    g.month  = (uint8_t)gps.date.month();
    g.day    = (uint8_t)gps.date.day();
    g.hour   = (uint8_t)gps.time.hour();
    g.minute = (uint8_t)gps.time.minute();
    g.second = (uint8_t)gps.time.second();
  }

  // Commit atomically
  taskENTER_CRITICAL(&gpsMux);
  LatestGPS = g;
  taskEXIT_CRITICAL(&gpsMux);
}

static inline void gpsSnapshot(LatestGPS_t &out) {
  taskENTER_CRITICAL(&gpsMux);
  out = LatestGPS;
  taskEXIT_CRITICAL(&gpsMux);
}

// “Available for use” helper: requires valid lat/lon AND freshness
static inline bool gpsHasFreshFix(const LatestGPS_t &g, uint16_t maxAgeMs = 1500) {
  return (g.validMask & GPS_LOC_BIT) && (g.age_ms <= maxAgeMs);
}


/* ------------------------- Logging ---------------------------- */
void enqueueLine(uint64_t rtc_s, uint64_t rtc_us, uint32_t t_log_us, 
                LatestBNO_t BNOsnap, LatestXHT_t XHTsnap, LatestGPS_t GPSsnap){
  uint16_t next = (qHead + 1) % Q_CAP;
  if (next == qTail) return; // drop if full
  
  q[qHead].t_rtc_s  = rtc_s;
  q[qHead].t_rtc_us = rtc_us;
  q[qHead].t_log_us = t_log_us;
  q[qHead].bno = BNOsnap;
  q[qHead].xht = XHTsnap;
  q[qHead].gps = GPSsnap;
  qHead = next;
}

bool dequeueLine(Line &out){
  if (qTail == qHead) return false;
  out = q[qTail];
  qTail = (qTail + 1) % Q_CAP;
  return true;
}

size_t lineToCSV(const Line& L, char* buf, size_t bufSize) {
  // Minimal fields for receiver-side altitude estimate:
  // t_log_us, temp, humidity, qw,qx,qy,qz, ax,ay,az
  size_t n = snprintf(
    buf, bufSize,
    "%lu,%.2f,%.2f,%ld,%ld,%ld,%ld,%ld,%ld,%ld",
    (unsigned long)L.t_log_us,
    (double)L.xht.temp,
    (double)L.xht.humidity,
    (long)L.bno.qw, (long)L.bno.qx, (long)L.bno.qy, (long)L.bno.qz,
    (long)L.bno.ax, (long)L.bno.ay, (long)L.bno.az
  );

  // Append GPS only if fresh fix is available
  if (n < bufSize) {
    if (gpsHasFreshFix(L.gps, 1500)) {
      // Append: lat_e7,lon_e7,alt_cm,sats,hdop_centi
      n += snprintf(buf + n, bufSize - n,
                    ",%ld,%ld,%ld,%u,%u",
                    (long)L.gps.lat_e7,
                    (long)L.gps.lon_e7,
                    (long)((L.gps.validMask & GPS_ALT_BIT) ? L.gps.alt_cm : 0),
                    (unsigned)L.gps.sats,
                    (unsigned)L.gps.hdop_centi);
    }
  }

  return n;
}


/* ----------------------- DS3231 Timer ------------------------- */
static void setSystemTimeFromRTC(const DateTime& dt) {
  // Convert RTClib DateTime -> Unix epoch seconds
  time_t t = (time_t)dt.unixtime();

  struct timeval now = { .tv_sec = t, .tv_usec = 0 };
  settimeofday(&now, nullptr);
}

static void printSystemTime() {
  time_t now = time(nullptr);
  struct tm lt;
  localtime_r(&now, &lt);

  char buf[32];
  strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &lt);
  Serial.println(buf);
}

void makeTimestampedFilename() {
  time_t now = time(nullptr);
  struct tm t;
  localtime_r(&now, &t);

  // Split base name and extension
  const char* dot = strrchr(FNAME, '.');

  if (dot) {
    // With extension
    snprintf(filename, sizeof(filename),
             "%.*s_%04d_%02d_%02d_%02d_%02d_%02d%s",
             int(dot - FNAME), FNAME,
             t.tm_year + 1900,
             t.tm_mon + 1,
             t.tm_mday,
             t.tm_hour,
             t.tm_min,
             t.tm_sec,
             dot);
  } else {
    // No extension
    snprintf(filename, sizeof(filename),
             "%s_%04d_%02d_%02d_%02d_%02d_%02d",
             FNAME,
             t.tm_year + 1900,
             t.tm_mon + 1,
             t.tm_mday,
             t.tm_hour,
             t.tm_min,
             t.tm_sec);
  }
}

/* -------------------------- XHT3X  ---------------------------- */
static inline void xhtPoll(){
  LatestXHT.t_us = micros();
  LatestXHT.temp = sht31.readTemperature();
  LatestXHT.humidity = sht31.readHumidity();
}

/* -------------------------- BNO08x ---------------------------- */

// Convert float numbers to int for smaller storage sizes
static inline int32_t fx7(float v) { return (int32_t)lrintf(v * 1e7f); }  // quaternion [-1,1] -> int32
static inline int32_t fx6(float v) { return (int32_t)lrintf(v * 1e6f); }
static inline int32_t fx4(float v) { return (int32_t)lrintf(v * 1e4f); }
static inline int32_t fx2(float v) { return (int32_t)lrintf(v * 1e2f); }

void setReports() {
  // Enable the 9-axis, mag-aligned rotation vector @100 Hz
  // Rotation vector (quaternion) 100 Hz → interval_us = 1e6/100 = 10000 µs
  if (!bno08x.enableReport(SH2_ARVR_STABILIZED_RV, 10000)) {
    Serial.println("! Failed to enable ARVR Stabilized Rotation Vector");
    while (1) delay(10);
  }

  // Linear acceleration (m/s^2, gravity removed)
  if (!bno08x.enableReport(SH2_LINEAR_ACCELERATION, 10000)) {
    Serial.println("Could not enable linear acceleration");
  }

  // Gyroscope (angular velocity, rad/s)
  if (!bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, 10000)) {
    Serial.println("Could not enable gyroscope");
  }

  // No magnetometer
  // if (!bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, 10000)) {
  //   Serial.println("Could not enable GAME rotation vector");
  // }
}

static inline void upsertRV(float qr,float qi,float qj,float qk,
                            uint8_t status, uint32_t bno_ts_us)
{
  ypr.yaw   = atan2f(2*(qr*qk + qi*qj), 1 - 2*(qj*qj + qk*qk));
  ypr.pitch = asinf (2*(qr*qj - qk*qi));
  ypr.roll  = atan2f(2*(qr*qi + qj*qk), 1 - 2*(qi*qi + qj*qj));
  ypr.yaw   *= 180.0f/M_PI; if (ypr.yaw < 0) ypr.yaw += 360.0f;
  ypr.pitch *= 180.0f/M_PI;
  ypr.roll  *= 180.0f/M_PI;

  taskENTER_CRITICAL(&latestMux);
  LatestBNO.yaw   = fx2(ypr.yaw);
  LatestBNO.pitch = fx2(ypr.pitch);
  LatestBNO.roll  = fx2(ypr.roll);

  LatestBNO.qw = fx7(qr);
  LatestBNO.qx = fx7(qi);
  LatestBNO.qy = fx7(qj);
  LatestBNO.qz = fx7(qk);

  LatestBNO.rv_status = status;
  LatestBNO.bno_ts_us = bno_ts_us;
  
  LatestBNO.t_us  = micros();
  taskEXIT_CRITICAL(&latestMux);
  // validMask |= RV_BIT;
  // lastUpd[0] = millis();
}

static inline void upsertAcc(float ax,float ay,float az,uint8_t status){
  taskENTER_CRITICAL(&latestMux);
  LatestBNO.ax = fx6(ax);
  LatestBNO.ay = fx6(ay);
  LatestBNO.az = fx6(az);
  LatestBNO.acc_status = status;
  LatestBNO.t_us = micros();
  // validMask |= ACC_BIT;
  // lastUpd[1] = millis();
  taskEXIT_CRITICAL(&latestMux);
}

static inline void upsertGyr(float gx,float gy,float gz,uint8_t status){
  taskENTER_CRITICAL(&latestMux);
  LatestBNO.gx = fx4(gx);
  LatestBNO.gy = fx4(gy);
  LatestBNO.gz = fx4(gz);
  LatestBNO.gyr_status = status;
  LatestBNO.t_us = micros();
  // validMask |= GYR_BIT;
  // lastUpd[2] = millis();
  taskEXIT_CRITICAL(&latestMux);
}

// Drain pending BNO08x events for up to `budget_us` microseconds.
// Call this wherever you need the freshest sensor state.
static inline void bnoPoll(uint32_t budget_us = 150) {
  uint32_t t0 = micros();
  // Loop ends if: no more events OR time budget exceeded
  while (bno08x.getSensorEvent(&sensorValue)) {
    switch (sensorValue.sensorId) {
      case SH2_ARVR_STABILIZED_RV: {
        auto &r = sensorValue.un.arvrStabilizedRV;
        uint8_t st = (uint8_t)sensorValue.status;          // typical 0..3
        uint32_t ts = (uint32_t)sensorValue.timestamp;     // lower 32 bits
        upsertRV(r.real, r.i, r.j, r.k, st, ts);
        break;
      }
      case SH2_LINEAR_ACCELERATION: {
        auto &a = sensorValue.un.linearAcceleration;
        upsertAcc(a.x, a.y, a.z, (uint8_t)sensorValue.status);
        break;
      }
      case SH2_GYROSCOPE_CALIBRATED: {
        auto &g = sensorValue.un.gyroscope;
        upsertGyr(g.x, g.y, g.z, (uint8_t)sensorValue.status);
        break;
      }
    }
    if (budget_us && (micros() - t0 >= budget_us)) break;
  }
}

// Atomically copy the latest sensor state
static inline void bnoSnapshot(LatestBNO_t &out) {
  taskENTER_CRITICAL(&latestMux);
  out = LatestBNO;
  taskEXIT_CRITICAL(&latestMux);
}


/* -----------------------------------------------------------------
####################################################################
--------------------------- PROGRAM SETUP --------------------------
####################################################################
----------------------------------------------------------------- */

void setup() {
  // For communication with computers, and change anemometer setting
  Serial.begin(115200);

  while (!Serial) delay(10);     // will pause Zero, Leonardo, etc until serial console opens

  // UART (Serial1) init
  Radio.begin(UART_BAUD, SERIAL_8N1, UART_RX, UART_TX);
  Serial.println("Wireless UART test initiated.");

  // Increase internal UART RX buffer (Arduino core uses HardwareSerial)
  // Radio.setRxBufferSize(8192); // helps at higher baud

  /* ------------------------ I2C INIT -------------------------- */
  // I2C on GPIO8 (SDA) / GPIO9 (SCL)
  Wire.begin(PIN_SDA, PIN_SCL);
  // Wire.setClock(100000); // DONT Set it! BNO library will take care of it!!

  /* ------------------------ GET TIME -------------------------- */
  if (!rtc.begin()) {
    Serial.println("DS3231 not found on I2C!");
    while (1) delay(10);
  }

  // If RTC lost power, you must set it at least once (compile-time as a fallback)
  if (rtc.lostPower()) {
    Serial.println("RTC lost power, setting RTC to compile time...");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  DateTime dt = rtc.now();
  setSystemTimeFromRTC(dt);

  // Optional: set timezone for correct "localtime()"
  // UK example: GMT/BST rules
  setenv("TZ", "GMT0BST,M3.5.0/1,M10.5.0/2", 1);
  tzset();

  Serial.print("System time set from DS3231: ");
  printSystemTime();

  /* ---------------------- BNO08x INIT ------------------------- */
  if (!bno08x.begin_I2C()) {
    Serial.println("! BNO08x not found");
    while (1) delay(10);
  }
  // Turn on the BNO08x report
  setReports();

  // Wait for first packet to arrive
  delay(100);

  /* ---------------- SHT3X INIT TEMP & HUMIDITY ---------------- */
  if (!sht31.begin(0x44)) {
    Serial.println("! SHT3X not found at 0x44");
    while (1) delay(10);
  }

  /* ----------------------- GPS INIT --------------------------- */
  // Start GPS UART on the chosen pins
  GPSSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  Serial.println("GPS reader started. Waiting for satellites...");

  /* ------------------------ SD INIT --------------------------- */

  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);
  if (!SD.begin(PIN_CS)) { 
    // FLASH RED forever to indicate SD mount error. 120 ms on, 380 ms off
    statusBlinkRed(120, 380);
    for(;;) { StatusBlinker.poll(); delay(1); } // stay here; no SD = no logging 
  }

  makeTimestampedFilename();
  // Create CSV with header if SD init successful
  logFile = SD.open(filename, FILE_WRITE);
  if (!logFile) {
    // FLASH RED forever to indicate file creation error. 380 ms on, 120 ms off
    statusBlinkRed(380, 120);
    while(true) { delay(1); }
  }

  // Successfully created csv file
  logFile.println(F("t_RTC_s,t_RTC_us,t_log_us,t_XHT,temp,humidity,t_us_BNO,"
                  "yaw,pitch,roll,"
                  "qw,qx,qy,qz,rv_status,bno_ts_us,"
                  "ax,ay,az,acc_status,"
                  "gx,gy,gz,gyr_status,"
                  "gps_t_us,gps_age_ms,gps_validMask,"
                  "lat_e7,lon_e7,alt_cm,spd_cms,sats,hdop_centi,"
                  "gps_year,gps_month,gps_day,gps_hour,gps_min,gps_sec"));
  logFile.flush();


  /* -------------------------- LED ----------------------------- */
  // show "running"
  statusBlinkGreen();
}

/* -----------------------------------------------------------------
####################################################################
----------------------------- MAIN LOOP ----------------------------
####################################################################
----------------------------------------------------------------- */

void loop() {
  /* -------------------------- LED ----------------------------- */
  // keep blink timing non-blocking
  StatusBlinker.poll();

  /* -------------------------- GPS ----------------------------- */
  // Keep GPS decoding fresh
  gpsPoll(300);

  /* ------------------------- BNO08x --------------------------- */
  if (bno08x.wasReset()) { setReports(); }      // keep reports enabled

  // 1) Keep BNO reasonably fresh even if UART is idle
  bnoPoll(150);
  LatestBNO_t BNOsnap;
  
  /* ------------------- SHT3X TEMP & HUMIDITY ------------------ */
  xhtPoll();

  /* ------------------------ Logging --------------------------- */
  static uint32_t nextLog_us = 0;
  uint32_t now_us = micros();
  
  // Handle micros() wraparound safely by using signed subtraction
  if ((int32_t)(now_us - nextLog_us) >= 0) {
    nextLog_us = now_us + LOG_INTERVAL_US;
    bnoPoll(150);
    bnoSnapshot(BNOsnap);
    gpsSnapshot(GPSsnap);

    struct timeval tv;
    gettimeofday(&tv, nullptr);
    uint64_t rtc_s  = (uint64_t)tv.tv_sec;
    uint64_t rtc_us = (uint64_t)tv.tv_sec * 1000000ULL + (uint64_t)tv.tv_usec;

    enqueueLine(rtc_s, rtc_us, micros(), BNOsnap, LatestXHT, GPSsnap);
  }

  // --------- Drain queue to SD in batches ----------
  static uint32_t batch = 0;
  Line L;
  bool wrote = false;
  while (dequeueLine(L)) {
    // Line timestamp
    logFile.print(L.t_rtc_s);  logFile.print(',');
    logFile.print(L.t_rtc_us); logFile.print(',');
    logFile.print(L.t_log_us); logFile.print(',');

    // XHT timestamp
    logFile.print(L.xht.t_us);  logFile.print(',');

    // Temperature and humidity
    logFile.print(L.xht.temp);  logFile.print(',');
    logFile.print(L.xht.humidity);
    logFile.print(',');

    // BNO timestamp
    logFile.print(L.bno.t_us);  logFile.print(',');

    // Euler angles
    logFile.print(L.bno.yaw);   logFile.print(',');
    logFile.print(L.bno.pitch); logFile.print(',');
    logFile.print(L.bno.roll);  logFile.print(',');

    // Quaternion (unitless *1e7)
    logFile.print(L.bno.qw); logFile.print(',');
    logFile.print(L.bno.qx); logFile.print(',');
    logFile.print(L.bno.qy); logFile.print(',');
    logFile.print(L.bno.qz); logFile.print(',');
    logFile.print(L.bno.rv_status); logFile.print(',');
    logFile.print(L.bno.bno_ts_us); logFile.print(',');

    // Linear acceleration
    logFile.print(L.bno.ax);    logFile.print(',');
    logFile.print(L.bno.ay);    logFile.print(',');
    logFile.print(L.bno.az);    logFile.print(',');
    logFile.print(L.bno.acc_status); logFile.print(',');

    // Gyroscope
    logFile.print(L.bno.gx);    logFile.print(',');
    logFile.print(L.bno.gy);    logFile.print(',');
    logFile.print(L.bno.gz); logFile.print(',');
    logFile.print(L.bno.gyr_status);
    logFile.print(',');

    // GPS
    logFile.print(L.gps.t_us); logFile.print(',');
    logFile.print(L.gps.age_ms); logFile.print(',');
    logFile.print(L.gps.validMask); logFile.print(',');

    if (L.gps.validMask & GPS_LOC_BIT)  logFile.print(L.gps.lat_e7);
    logFile.print(',');

    if (L.gps.validMask & GPS_LOC_BIT)  logFile.print(L.gps.lon_e7);
    logFile.print(',');

    if (L.gps.validMask & GPS_ALT_BIT)  logFile.print(L.gps.alt_cm);
    logFile.print(',');

    if (L.gps.validMask & GPS_SPD_BIT)  logFile.print(L.gps.spd_cms);
    logFile.print(',');

    if (L.gps.validMask & GPS_SATS_BIT) logFile.print(L.gps.sats);
    logFile.print(',');

    if (L.gps.validMask & GPS_HDOP_BIT) logFile.print(L.gps.hdop_centi);
    logFile.print(',');

    if (L.gps.validMask & GPS_UTC_BIT)  logFile.print(L.gps.year);
    logFile.print(',');

    if (L.gps.validMask & GPS_UTC_BIT)  logFile.print(L.gps.month);
    logFile.print(',');

    if (L.gps.validMask & GPS_UTC_BIT)  logFile.print(L.gps.day);
    logFile.print(',');

    if (L.gps.validMask & GPS_UTC_BIT)  logFile.print(L.gps.hour);
    logFile.print(',');

    if (L.gps.validMask & GPS_UTC_BIT)  logFile.print(L.gps.minute);
    logFile.print(',');

    if (L.gps.validMask & GPS_UTC_BIT)  logFile.print(L.gps.second);
    logFile.println();

    if (++batch % 64 == 0) { logFile.flush(); wrote = true; }
    if (batch % 10 == 0) {
      // Prep a message every 10 lines.
      // Convert L into a single string that could be sent to Radio later
      wireless_n = lineToCSV(L, txBuf, sizeof(txBuf));
    }
  }


  // Light flush every ~100ms even if small volume (reduces loss on power cut)
  static uint32_t lastFlush = 0;
  uint32_t now = millis();
  if (!wrote && (now - lastFlush) > 100) { logFile.flush(); lastFlush = now; }
  

  /* ------------------------ UART1 TX -------------------------- */
  if (wireless_n > 0) {
    Radio.write((uint8_t*)txBuf, wireless_n);
    Radio.write('\n');
    wireless_n = 0;
  }
}
