#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>

// XIAO ESP32-S3 default I2C: SDA=GPIO5 (D4), SCL=GPIO6 (D5)
// Change here if wiring is different.
#ifndef I2C_SDA
#define I2C_SDA SDA
#endif
#ifndef I2C_SCL
#define I2C_SCL SCL
#endif

#define BNO08X_RESET -1
// INT pin for watchdog read-only diagnostic. Default matches RobotConfig
// gyro_int on XIAO ESP32-S3 (D1). Override if wired elsewhere.
#ifndef BNO08X_INT
#define BNO08X_INT D1
#endif

struct euler_t { float yaw, pitch, roll; } ypr;

Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

sh2_SensorId_t reportType = SH2_GAME_ROTATION_VECTOR;
long reportIntervalUs = 50000;  // 20 Hz — keep easy on the bus while debugging

static void scanI2C() {
  Serial.println("=== I2C scan ===");
  uint8_t found = 0;
  for (uint8_t addr = 0x08; addr < 0x78; addr++) {
    Wire.beginTransmission(addr);
    uint8_t err = Wire.endTransmission();
    if (err == 0) {
      Serial.printf("  device at 0x%02X\n", addr);
      found++;
    }
  }
  if (!found) Serial.println("  no devices responded");
  Serial.printf("=== scan done (%u found) ===\n", found);
}

void setReports(sh2_SensorId_t t, long interval) {
  Serial.println("Setting desired reports");
  if (!bno08x.enableReport(t, interval)) {
    Serial.println("Could not enable stabilized remote vector");
  }
}

void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t* ypr, bool degrees = false) {
  float sqr = sq(qr), sqi = sq(qi), sqj = sq(qj), sqk = sq(qk);
  ypr->yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
  ypr->pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
  ypr->roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));
  if (degrees) { ypr->yaw *= RAD_TO_DEG; ypr->pitch *= RAD_TO_DEG; ypr->roll *= RAD_TO_DEG; }
}

void setup(void) {
  Serial.begin(115200);
  delay(2000);  // USB-CDC enumerate
  Serial.println();
  Serial.printf("Using SDA=%d SCL=%d\n", (int)I2C_SDA, (int)I2C_SCL);

  // Let the BNO finish its own power-up before ANY bus activity.
  // Datasheet: ~300 ms to reach operational state. Give it plenty.
  delay(800);

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);  // slow for reliability
  delay(100);

  // Retry begin_I2C — one cold-boot failure shouldn't end the sketch.
  Serial.println("Attempting BNO08x @ 0x4A...");
  bool ok = false;
  for (int attempt = 1; attempt <= 5 && !ok; attempt++) {
    Serial.printf("  attempt %d...\n", attempt);
    ok = bno08x.begin_I2C(0x4A, &Wire);
    if (!ok) {
      Serial.println("  failed, waiting 1s");
      delay(1000);
    }
  }
  if (!ok) {
    Serial.println("All attempts failed. Halting.");
    while (1) delay(1000);
  }
  Serial.println("BNO08x Found!");
  setReports(reportType, reportIntervalUs);
  Serial.println("Reading events");
}

void loop() {
  if (bno08x.wasReset()) {
    Serial.println("sensor was reset");
    setReports(reportType, reportIntervalUs);
  }

  static uint32_t event_count = 0;
  static uint32_t last_status_ms = 0;
  static uint32_t last_event_ms = 0;
  static bool scanned_for_hang = false;

  if (bno08x.getSensorEvent(&sensorValue)) {
    event_count++;
    last_event_ms = millis();
    scanned_for_hang = false;  // re-arm watchdog once data resumes
    if (sensorValue.sensorId == SH2_GAME_ROTATION_VECTOR) {
      auto& q = sensorValue.un.gameRotationVector;
      quaternionToEuler(q.real, q.i, q.j, q.k, &ypr, true);
      Serial.printf("\nGRV  st=%u  y=%.2f  p=%.2f  r=%.2f\n",
                    sensorValue.status, ypr.yaw, ypr.pitch, ypr.roll);
    } else {
      Serial.printf("other event id=0x%02X\n", sensorValue.sensorId);
    }
  }

  uint32_t now = millis();

  // Hang watchdog: if events have stopped for >1 s after we've seen at least
  // one, do a single I2C scan to check whether 0x4A still answers. This
  // splits "chip is gone" from "ESP32 I2C peripheral is wedged".
  if (!scanned_for_hang && event_count > 0 && (now - last_event_ms) > 1000) {
    Serial.printf("[WDG] no events for %lu ms — scanning bus\n",
                  now - last_event_ms);
    scanI2C();
    Serial.printf("[WDG] INT pin (GPIO %d) level = %d\n", BNO08X_INT,
                  digitalRead(BNO08X_INT));
    scanned_for_hang = true;
  }

  if (now - last_status_ms >= 2000) {
    last_status_ms = now;
    Serial.printf("[%lus] events=%lu\n", now / 1000, event_count);
  }
}
