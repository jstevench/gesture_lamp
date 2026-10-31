// ESP32 Dev Module + MPU6050 (SDA=21, SCL=22). See README.md for wiring and setup.
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>

constexpr int SDA_PIN = 21;
constexpr int SCL_PIN = 22;
constexpr uint32_t SAMPLE_MS = 10;  // About 100 readings per second.
constexpr float TRIGGER_MS2 = 3.0f; // Lower = more sensitive.
constexpr float QUIET_MS2 = 1.0f;
constexpr float STILL_CHANGE_MS2 = 0.8f;
constexpr uint32_t COOLDOWN_MS = 450;
constexpr uint32_t QUIET_TIME_MS = 200;
constexpr bool PRINT_READINGS = false; // true: also print X/Y/Z for debugging.

Adafruit_MPU6050 mpu;
float restX, restY, restZ;
float quietX, quietY, quietZ;
float quietSumX, quietSumY, quietSumZ;
uint32_t quietSamples = 0;
float gravityMagnitude;
bool ready = false;
bool armed = false;
bool quietTiming = false;
uint8_t positiveSamples = 0;
uint8_t negativeSamples = 0;
uint32_t lastSample = 0;
uint32_t lastGesture = 0;
uint32_t quietSince = 0;
uint32_t lastDebug = 0;

// Calibration needs one continuous second of nearly unchanged readings.
bool calibrate() {
  Serial.println("Keep the sensor still in your normal holding position...");
  float sumX = 0, sumY = 0, sumZ = 0;
  float previousX = 0, previousY = 0, previousZ = 0;
  int count = 0;
  const uint32_t start = millis();
  while (millis() - start < 15000) {
    sensors_event_t a, g, temperature;
    if (!mpu.getEvent(&a, &g, &temperature)) return false;
    const float x = a.acceleration.x;
    const float y = a.acceleration.y;
    const float z = a.acceleration.z;
    const float magnitude = sqrtf(x*x + y*y + z*z);
    const float rotation = sqrtf(g.gyro.x*g.gyro.x + g.gyro.y*g.gyro.y +
                                 g.gyro.z*g.gyro.z);
    const float change = sqrtf((x-previousX)*(x-previousX) +
                               (y-previousY)*(y-previousY) +
                               (z-previousZ)*(z-previousZ));
    if (fabsf(magnitude - 9.81f) > 1.2f || rotation > 0.35f ||
        (count > 0 && change > 0.5f)) {
      count = 0;
      sumX = sumY = sumZ = 0;
    } else {
      sumX += x;
      sumY += y;
      sumZ += z;
      ++count;
    }
    previousX = x;
    previousY = y;
    previousZ = z;
    if (count >= 100) {
      restX = sumX / count;
      restY = sumY / count;
      restZ = sumZ / count;
      return true;
    }
    delay(SAMPLE_MS);
  }
  return false;
}

void setup() {
  Serial.begin(115200);
  // Give USB Serial time to connect, but do not wait forever.
  const uint32_t start = millis();
  while (!Serial && millis() - start < 3000) delay(10);
  Serial.println("\nMPU6050 flick test");
  if (!Wire.begin(SDA_PIN, SCL_PIN, 100000)) {
    Serial.println("ERROR: Could not start I2C. Check pin settings, then reset.");
    return;
  }
  if (!mpu.begin(0x68, &Wire) && !mpu.begin(0x69, &Wire)) {
    Serial.println("ERROR: MPU6050 not found. Check power, GND, SDA and SCL, then reset.");
    Serial.println("I2C connection check: SDA=21, SCL=22");
    int found = 0;
    for (uint8_t address = 1; address < 127; ++address) {
      Wire.beginTransmission(address);
      if (Wire.endTransmission() == 0) {
        Serial.printf("I2C response at 0x%02X\n", address);
        ++found;
      }
      delay(1);
    }
    if (found == 0) Serial.println("No I2C devices responded. Check wiring and contact.");
    return;
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  delay(200);
  if (!calibrate()) {
    Serial.println("ERROR: Calibration failed. Hold still and press RESET to retry.");
    return;
  }
  ready = true;
  gravityMagnitude = sqrtf(restX*restX + restY*restY + restZ*restZ);
  Serial.println("Ready! Flick UP or DOWN without twisting the sensor.");
}

void loop() {
  if (!ready) {
    delay(100);
    return;
  }
  const uint32_t now = millis();
  if (now - lastSample < SAMPLE_MS) return;
  lastSample = now;
  sensors_event_t a, g, temperature;
  if (!mpu.getEvent(&a, &g, &temperature)) {
    Serial.println("ERROR: Sensor read failed. Check wiring and reset.");
    ready = false;
    return;
  }

  const float dx = a.acceleration.x - restX;
  const float dy = a.acceleration.y - restY;
  const float dz = a.acceleration.z - restZ;
  const float restMagnitude = sqrtf(restX*restX + restY*restY + restZ*restZ);
  // At rest the accelerometer points upward. Project the change onto that
  // direction, so the sensor need not be mounted with a particular axis up.
  const float vertical = (dx*restX + dy*restY + dz*restZ) / restMagnitude;
  const float magnitude = sqrtf(a.acceleration.x*a.acceleration.x +
                                a.acceleration.y*a.acceleration.y +
                                a.acceleration.z*a.acceleration.z);
  const float rotation = sqrtf(g.gyro.x*g.gyro.x + g.gyro.y*g.gyro.y +
                               g.gyro.z*g.gyro.z);
  // Stillness is independent of the original holding angle. Comparing the
  // entire vector with startup gravity prevented rearming after a small tilt.
  const bool quiet = fabsf(magnitude - gravityMagnitude) < QUIET_MS2 && rotation < 0.5f;

  if (PRINT_READINGS && now - lastDebug >= 100) {
    lastDebug = now;
    Serial.printf("X:%.2f Y:%.2f Z:%.2f Vertical:%.2f m/s^2\n",
                  a.acceleration.x, a.acceleration.y, a.acceleration.z, vertical);
  }

  if (!armed) {
    // Both a minimum cooldown and a still period are required to rearm.
    // This suppresses the opposite acceleration as the hand stops.
    const float qdx = a.acceleration.x - quietX;
    const float qdy = a.acceleration.y - quietY;
    const float qdz = a.acceleration.z - quietZ;
    const bool stableWindow = !quietTiming ||
        qdx*qdx + qdy*qdy + qdz*qdz < STILL_CHANGE_MS2*STILL_CHANGE_MS2;
    if (quiet && stableWindow) {
      if (!quietTiming) {
        quietTiming = true;
        quietSince = now;
        quietX = a.acceleration.x;
        quietY = a.acceleration.y;
        quietZ = a.acceleration.z;
        quietSumX = quietSumY = quietSumZ = 0;
        quietSamples = 0;
      }
      quietSumX += a.acceleration.x;
      quietSumY += a.acceleration.y;
      quietSumZ += a.acceleration.z;
      ++quietSamples;
      if (now - lastGesture >= COOLDOWN_MS && now - quietSince >= QUIET_TIME_MS) {
        // Refresh the upward direction only after a stable pause, never during
        // the flick. Keep the original gravity magnitude as a sanity check.
        restX = quietSumX / quietSamples;
        restY = quietSumY / quietSamples;
        restZ = quietSumZ / quietSamples;
        armed = true;
        positiveSamples = negativeSamples = 0;
        Serial.println("Armed. Ready for next flick.");
      }
    } else {
      quietTiming = false;
    }
    return;
  }

  // Two consecutive samples reject isolated spikes.
  positiveSamples = vertical > TRIGGER_MS2 ? positiveSamples + 1 : 0;
  negativeSamples = vertical < -TRIGGER_MS2 ? negativeSamples + 1 : 0;
  if (positiveSamples >= 2 || negativeSamples >= 2) {
    Serial.println(positiveSamples >= 2 ? "FLICK UP" : "FLICK DOWN");
    lastGesture = now;
    armed = false;
    quietTiming = false;
  }
}
