// ESP32 Dev Module + MPU6050 gesture sender. Arduino ESP32 core 3.3.12.
#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <atomic>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>

constexpr int SDA_PIN = 21;
constexpr int SCL_PIN = 22;
constexpr uint32_t SAMPLE_MS = 10;  // About 100 readings per second.
constexpr float TRIGGER_MS2 = 6.0f; // Lower = more sensitive.
constexpr float TRIGGER_SIDE_MS2 = 6.0f; // Left/right sensitivity. Lower = more sensitive.
constexpr float QUIET_MS2 = 1.0f;
constexpr float STILL_CHANGE_MS2 = 0.8f;
constexpr uint32_t COOLDOWN_MS = 450;
constexpr uint32_t QUIET_TIME_MS = 200;
constexpr bool PRINT_READINGS = false; // true: also print X/Y/Z for debugging.

// Original receiver address: verify against the RECEIVER's Serial Monitor.
uint8_t receiverMAC[] = {0x30, 0x76, 0xF5, 0x91, 0xF3, 0x50};
constexpr uint8_t RADIO_CHANNEL = 1; // Must match receiver_espnow.
constexpr int32_t FLICK_UP_COMMAND = 1;
constexpr int32_t FLICK_DOWN_COMMAND = -1;
constexpr int32_t FLICK_RIGHT_COMMAND = 2;   // Receiver must handle these two.
constexpr int32_t FLICK_LEFT_COMMAND = -2;
// Sensor axis that points to your RIGHT when held normally. If left and
// right come out swapped, change 1.0f to -1.0f. If the sensor is mounted so
// its Y axis points right instead, use {0, 1, 0}.
constexpr float RIGHT_AXIS[3] = {1.0f, 0.0f, 0.0f};
static_assert(sizeof(int32_t) == 4, "Gesture packets must be four bytes");
std::atomic<int> sendResult{-1};
bool sendInFlight = false;
int32_t outgoingCommand = 0;

// Signature for Arduino ESP32 3.3.12. Keep the Wi-Fi callback short;
// loop() handles Serial output and the next send.
void onDataSent(const wifi_tx_info_t *, esp_now_send_status_t status) {
  sendResult.store(static_cast<int>(status));
}

bool startRadio() {
  if (!WiFi.mode(WIFI_STA)) return false;
  WiFi.setSleep(false);
  if (esp_wifi_set_channel(RADIO_CHANNEL, WIFI_SECOND_CHAN_NONE) != ESP_OK) return false;
  if (esp_now_init() != ESP_OK) return false;
  if (esp_now_register_send_cb(onDataSent) != ESP_OK) return false;
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, receiverMAC, sizeof(receiverMAC));
  peer.channel = RADIO_CHANNEL;
  peer.ifidx = WIFI_IF_STA;
  peer.encrypt = false;
  if (esp_now_add_peer(&peer) != ESP_OK) return false;
  Serial.print("Sender MAC: ");
  Serial.println(WiFi.macAddress());
  Serial.printf("Receiver: %02X:%02X:%02X:%02X:%02X:%02X, channel %u\n",
                receiverMAC[0], receiverMAC[1], receiverMAC[2],
                receiverMAC[3], receiverMAC[4], receiverMAC[5], RADIO_CHANNEL);
  return true;
}

void reportSendResult() {
  const int result = sendResult.exchange(-1);
  if (result < 0) return;
  sendInFlight = false;
  Serial.println(result == ESP_NOW_SEND_SUCCESS
      ? "Send OK (radio delivery; check receiver Serial for the command)"
      : "Send FAILED: check receiver power, MAC and channel.");
}

void sendGesture(int32_t command) {
  if (sendInFlight) {
    Serial.println("Send skipped: previous packet still pending.");
    return;
  }
  outgoingCommand = command;
  sendInFlight = true;
  const esp_err_t result = esp_now_send(receiverMAC,
      reinterpret_cast<const uint8_t *>(&outgoingCommand), sizeof(outgoingCommand));
  if (result != ESP_OK) {
    sendInFlight = false;
    Serial.printf("Send rejected: %s\n", esp_err_to_name(result));
  } else {
    Serial.printf("Queued gesture command: %ld\n", static_cast<long>(outgoingCommand));
  }
}

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
uint8_t rightSamples = 0;
uint8_t leftSamples = 0;
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
  Serial.println("\nMPU6050 ESP-NOW gesture sender");
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
  if (!startRadio()) {
    Serial.println("ERROR: ESP-NOW setup failed. Press EN to retry.");
    return;
  }
  ready = true;
  gravityMagnitude = sqrtf(restX*restX + restY*restY + restZ*restZ);
  Serial.println("Ready! Flick UP, DOWN, LEFT or RIGHT without twisting the sensor.");
}

void loop() {
  reportSendResult();
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
  // Sideways direction: the RIGHT_AXIS with its gravity component removed, so
  // it lies flat (horizontal) whatever the tilt. Left/right = change along it.
  const float ux = restX / restMagnitude;
  const float uy = restY / restMagnitude;
  const float uz = restZ / restMagnitude;
  const float axisDotUp = RIGHT_AXIS[0]*ux + RIGHT_AXIS[1]*uy + RIGHT_AXIS[2]*uz;
  const float rx = RIGHT_AXIS[0] - axisDotUp*ux;
  const float ry = RIGHT_AXIS[1] - axisDotUp*uy;
  const float rz = RIGHT_AXIS[2] - axisDotUp*uz;
  const float rNorm = sqrtf(rx*rx + ry*ry + rz*rz);
  // If the right axis points almost straight up/down, sideways is undefined.
  const float side = rNorm > 0.3f ? (dx*rx + dy*ry + dz*rz) / rNorm : 0.0f;
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
    Serial.printf("X:%.2f Y:%.2f Z:%.2f Vertical:%.2f Side:%.2f m/s^2\n",
                  a.acceleration.x, a.acceleration.y, a.acceleration.z, vertical, side);
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
        rightSamples = leftSamples = 0;
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
  rightSamples = side > TRIGGER_SIDE_MS2 ? rightSamples + 1 : 0;
  leftSamples = side < -TRIGGER_SIDE_MS2 ? leftSamples + 1 : 0;

  // If both an up/down and a left/right flick qualify, the stronger one
  // (relative to its own threshold) wins.
  const float verticalStrength = fabsf(vertical) / TRIGGER_MS2;
  const float sideStrength = fabsf(side) / TRIGGER_SIDE_MS2;
  const bool verticalHit = positiveSamples >= 2 || negativeSamples >= 2;
  const bool sideHit = rightSamples >= 2 || leftSamples >= 2;
  if (verticalHit || sideHit) {
    int32_t command;
    if (verticalHit && (!sideHit || verticalStrength >= sideStrength)) {
      command = positiveSamples >= 2 ? FLICK_UP_COMMAND : FLICK_DOWN_COMMAND;
      Serial.println(command == FLICK_UP_COMMAND ? "FLICK UP" : "FLICK DOWN");
    } else {
      command = rightSamples >= 2 ? FLICK_RIGHT_COMMAND : FLICK_LEFT_COMMAND;
      Serial.println(command == FLICK_RIGHT_COMMAND ? "FLICK RIGHT" : "FLICK LEFT");
    }
    sendGesture(command);
    lastGesture = now;
    armed = false;
    quietTiming = false;
  }
}
