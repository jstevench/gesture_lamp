// ESP32 Dev Module + hand-mounted MPU6050 brightness and animation controls.
#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <atomic>
#include <Adafruit_MPU6050.h>
#include <math.h>

constexpr int SDA_PIN = 21;
constexpr int SCL_PIN = 22;
constexpr uint32_t SAMPLE_MS = 10;
constexpr uint8_t RADIO_CHANNEL = 1;
uint8_t receiverMAC[] = {0x30, 0x76, 0xF5, 0x91, 0xF3, 0x50};

// Keep these wire values in sync with gesture_slider_receiver.
constexpr int32_t BRIGHTNESS_SET_BASE = 12000;
constexpr int PWM_MAX = 1023;
constexpr int32_t FLICK_RIGHT_COMMAND = 2;
constexpr int32_t FLICK_LEFT_COMMAND = -2;
constexpr uint32_t BRIGHTNESS_UPDATE_MS = 50;
constexpr uint32_t BRIGHTNESS_REFRESH_MS = 750;
constexpr float PALM_ENDPOINT_DEG = 1.0f;

// Mount sensor X along the hand (wrist to fingertips). Set to 1 if Y is
// along the hand instead, so the sideways projection follows the mounting.
constexpr uint8_t HAND_FORWARD_AXIS = 0;
constexpr uint8_t HAND_SIDE_AXIS = HAND_FORWARD_AXIS == 0 ? 1 : 0;
constexpr int SIDE_SIGN = 1; // Change to -1 if left and right are swapped.
constexpr float LOCK_LIFT_MS2 = 2.4f;
constexpr float LOCK_PITCH_RAD_S = 2.6f;
constexpr float LOCK_LIFT_MAX_ROLL_RAD_S = 1.7f;
constexpr uint8_t LOCK_FLICK_SAMPLES = 3;
constexpr uint32_t LOCK_COOLDOWN_MS = 450;
constexpr uint8_t LOCK_QUIET_SAMPLES = 6;
constexpr uint32_t SIDE_SETTLE_AFTER_LOCK_MS = 150;

struct FlickDetector {
  float startMs2;
  float brakeMs2;
  float quietMs2;
  int direction = 0;
  uint8_t startSamples = 0;
  uint8_t brakeSamples = 0;
  uint32_t startedMs = 0;
  uint32_t lastFlickMs = 0;
  bool latched = false;

  void clearCandidate() {
    direction = 0;
    startSamples = brakeSamples = 0;
  }

  int update(float acceleration, uint32_t now, bool allowed) {
    const float strength = fabsf(acceleration);
    if (latched) {
      if (now - lastFlickMs >= 450 && strength <= quietMs2) latched = false;
      return 0;
    }
    if (!allowed) {
      clearCandidate();
      return 0;
    }
    if (startSamples < 3) {
      const int next = strength < startMs2 ? 0 : (acceleration > 0.0f ? 1 : -1);
      if (next != direction) startSamples = 0;
      direction = next;
      if (direction != 0 && ++startSamples == 3) startedMs = now;
      return 0;
    }
    if (now - startedMs > 220) {
      clearCandidate();
      return 0;
    }
    // The first burst determines direction. Its braking pulse completes the flick.
    if (acceleration * direction > -brakeMs2) {
      brakeSamples = 0;
      return 0;
    }
    if (++brakeSamples < 2) return 0;

    const int result = direction;
    lastFlickMs = now;
    latched = true;
    clearCandidate();
    return result;
  }
};

FlickDetector sideFlick{8.0f, 3.2f, 1.1f};

Adafruit_MPU6050 mpu;
uint8_t sensorAddress = 0x68;
bool ready = false;
float gravity[3] = {};
float gyroBias[3] = {};
float gravityMagnitude = 0.0f;
float palmDownSign = 1.0f;
uint32_t lastSampleUs = 0;

bool controlsEnabled = false;
uint32_t lastLockToggleMs = 0;
int32_t pendingAnimation = 0;
int brightnessTarget = 0;
int lastDeliveredBrightness = -1;
uint32_t lastBrightnessSendMs = 0;
uint32_t lastDeliveryLogMs = 0;
int32_t lastSentCommand = 0;

std::atomic<int> sendResult{-1};
bool sendInFlight = false;

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
  if (esp_now_add_peer(&peer) != ESP_OK) return false;
  uint8_t mac[6];
  if (esp_wifi_get_mac(WIFI_IF_STA, mac) == ESP_OK) {
    Serial.printf("Sender MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                  mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  } else {
    Serial.println("Could not read sender MAC from Wi-Fi driver.");
  }
  return true;
}

void sendCommand(int32_t command) {
  lastSentCommand = command;
  // Do not replay an incremental command; the receiver may have applied it.
  sendInFlight = true;
  const esp_err_t result = esp_now_send(receiverMAC,
      reinterpret_cast<const uint8_t *>(&command), sizeof(command));
  if (result != ESP_OK) {
    sendInFlight = false;
    Serial.printf("Gesture send rejected: %s\n", esp_err_to_name(result));
  }
}

void sendPendingCommand(uint32_t now) {
  if (!controlsEnabled || sendInFlight) return;
  if (pendingAnimation != 0) {
    const int32_t command = pendingAnimation;
    pendingAnimation = 0;
    sendCommand(command);
    return;
  }
  const uint32_t sinceBrightness = now - lastBrightnessSendMs;
  if (sinceBrightness < BRIGHTNESS_UPDATE_MS) return;
  if (brightnessTarget == lastDeliveredBrightness && sinceBrightness < BRIGHTNESS_REFRESH_MS) return;
  // An absolute refresh is safe to repeat if a packet was missed.
  lastBrightnessSendMs = now;
  sendCommand(BRIGHTNESS_SET_BASE + brightnessTarget);
}

bool readMotion(float accel[3], float gyro[3]) {
  // The Adafruit library configures the sensor. Read the measurement block
  // through Wire so a short or failed I2C read cannot become a gesture.
  constexpr uint8_t FIRST_REGISTER = 0x3B; // ACCEL_XOUT_H
  constexpr size_t BYTE_COUNT = 14;      // XYZ accel, temperature, XYZ gyro
  uint8_t data[BYTE_COUNT];
  Wire.beginTransmission(sensorAddress);
  Wire.write(FIRST_REGISTER);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(sensorAddress, BYTE_COUNT, true) != BYTE_COUNT) return false;
  if (Wire.readBytes(data, BYTE_COUNT) != BYTE_COUNT) return false;
  for (size_t axis = 0; axis < 3; ++axis) {
    const size_t a = 2 * axis;
    const size_t g = 8 + 2 * axis;
    const int16_t rawAccel = static_cast<int16_t>((uint16_t(data[a]) << 8) | data[a + 1]);
    const int16_t rawGyro = static_cast<int16_t>((uint16_t(data[g]) << 8) | data[g + 1]);
    accel[axis] = rawAccel * (9.80665f / 4096.0f);       // ±8 g
    gyro[axis] = rawGyro * (DEG_TO_RAD / 65.5f);          // ±500 deg/s
  }
  return true;
}

float length(const float v[3]) {
  return sqrtf(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
}

bool calibrate() {
  Serial.println("Hold your palm down and still for calibration.");
  float accelSum[3] = {}, gyroSum[3] = {};
  int count = 0;
  const uint32_t start = millis();
  while (millis() - start < 15000) {
    float accel[3], gyro[3];
    if (!readMotion(accel, gyro)) return false;
    const float magnitude = length(accel);
    if (fabsf(magnitude - 9.80665f) > 1.0f || length(gyro) > 0.35f ||
        fabsf(accel[2]) < 0.85f * magnitude) {
      count = 0;
      for (size_t i = 0; i < 3; ++i) accelSum[i] = gyroSum[i] = 0.0f;
    } else {
      for (size_t i = 0; i < 3; ++i) {
        accelSum[i] += accel[i];
        gyroSum[i] += gyro[i];
      }
      if (++count >= 100) {
        for (size_t i = 0; i < 3; ++i) {
          gravity[i] = accelSum[i] / count;
          gyroBias[i] = gyroSum[i] / count;
        }
        gravityMagnitude = length(gravity);
        palmDownSign = gravity[2] >= 0.0f ? 1.0f : -1.0f;
        return gravityMagnitude > 8.0f;
      }
    }
    delay(SAMPLE_MS);
  }
  return false;
}

void measureHandAcceleration(const float accel[3], const float gyro[3], float rotation,
                             float dt, float &vertical, float &side) {
  const float gx = gravity[0], gy = gravity[1], gz = gravity[2];
  // Rotate the upward reference opposite to the sensor's rotation.
  gravity[0] -= (gyro[1]*gz - gyro[2]*gy) * dt;
  gravity[1] -= (gyro[2]*gx - gyro[0]*gz) * dt;
  gravity[2] -= (gyro[0]*gy - gyro[1]*gx) * dt;

  // Correct gyro drift gently when the accelerometer is close to 1 g.
  // The gyro carries the orientation estimate during turns and sharp flicks.
  if (rotation < 0.55f && fabsf(length(accel) - gravityMagnitude) < 0.30f) {
    for (size_t i = 0; i < 3; ++i) gravity[i] += (accel[i] - gravity[i]) * 0.025f;
  }
  const float scale = gravityMagnitude / length(gravity);
  for (size_t i = 0; i < 3; ++i) gravity[i] *= scale;
  const float up = (accel[0]*gravity[0] + accel[1]*gravity[1] + accel[2]*gravity[2]) /
      gravityMagnitude;
  vertical = up - gravityMagnitude;

  // Use the horizontal direction across the hand. This remains usable after
  // a wrist roll, even if the sensor's sideways axis now points straight up.
  const float horizontalLength = hypotf(gravity[HAND_SIDE_AXIS], gravity[2]);
  side = horizontalLength > 0.3f * gravityMagnitude
      ? SIDE_SIGN * palmDownSign * (accel[HAND_SIDE_AXIS]*gravity[2] - accel[2]*gravity[HAND_SIDE_AXIS]) /
          horizontalLength : 0.0f;
}

float palmAngle() {
  // Gravity projected onto the palm normal gives 0..180 degrees for either
  // roll direction, without accumulating hand travel or roll angle over time.
  const float cosine = constrain(palmDownSign * gravity[2] / gravityMagnitude, -1.0f, 1.0f);
  return acosf(cosine) * RAD_TO_DEG;
}

int palmBrightness(float angle) {
  if (angle <= PALM_ENDPOINT_DEG) return 0;
  if (angle >= 180.0f - PALM_ENDPOINT_DEG) return PWM_MAX;
  return lroundf(angle * (PWM_MAX / 180.0f));
}

void updateHandControls(float vertical, float side, const float gyro[3], uint32_t now) {
  static bool flickArmed = true;
  static uint8_t flickSamples = 0;
  static uint8_t quietSamples = 0;
  const float lift = fabsf(vertical);
  const float pitch = fabsf(gyro[HAND_SIDE_AXIS]);
  const float roll = fabsf(gyro[HAND_FORWARD_AXIS]);
  // A quick lift/drop or a bend of the wrist can toggle the lock. A palm
  // roll is around the wrist-to-fingertip axis and only adjusts brightness.
  const bool liftBurst = lift >= LOCK_LIFT_MS2 && lift >= 1.10f * fabsf(side) &&
      roll <= LOCK_LIFT_MAX_ROLL_RAD_S;
  const bool pitchBurst = pitch >= LOCK_PITCH_RAD_S && pitch >= 1.35f * max(roll, fabsf(gyro[2]));
  if (flickArmed) {
    flickSamples = (liftBurst || pitchBurst) ? flickSamples + 1 : 0;
  } else {
    const bool quiet = now - lastLockToggleMs >= LOCK_COOLDOWN_MS && lift < 0.9f && pitch < 0.7f;
    quietSamples = quiet ? quietSamples + 1 : 0;
    flickArmed = quietSamples >= LOCK_QUIET_SAMPLES;
  }
  if (flickSamples == LOCK_FLICK_SAMPLES) {
    flickArmed = false;
    flickSamples = quietSamples = 0;
    controlsEnabled = !controlsEnabled;
    lastLockToggleMs = now;
    pendingAnimation = 0;
    sideFlick.clearCandidate();
    sideFlick.latched = false;
    lastDeliveredBrightness = -1;
    lastBrightnessSendMs = now - BRIGHTNESS_UPDATE_MS;
    Serial.print("FLICK: ");
    Serial.println(controlsEnabled ? "CONTROLS ON: palm angle sets brightness."
                                   : "CONTROLS LOCKED: brightness and animation selection held.");
  }
  if (!controlsEnabled) return;

  brightnessTarget = palmBrightness(palmAngle());
  const bool sideAllowed = flickSamples == 0 && !pitchBurst &&
      now - lastLockToggleMs >= SIDE_SETTLE_AFTER_LOCK_MS;
  const int sideGesture = sideFlick.update(side, now, sideAllowed);
  if (sideGesture != 0) {
    pendingAnimation = sideGesture > 0 ? FLICK_RIGHT_COMMAND : FLICK_LEFT_COMMAND;
    Serial.println(sideGesture > 0 ? "FLICK RIGHT: next animation." : "FLICK LEFT: previous animation.");
  }
}

void printMotionDebug(float vertical, float side, const float gyro[3], uint32_t now) {
  static bool enabled = false;
  static uint32_t lastReportMs = 0;
  static float peakLift = 0.0f, peakSide = 0.0f, peakPitch = 0.0f, peakRoll = 0.0f;
  if (Serial.available() && Serial.read() == 'd') {
    enabled = !enabled;
    peakLift = peakSide = peakPitch = peakRoll = 0.0f;
    lastReportMs = now;
    Serial.println(enabled ? "Motion debug on (peaks every 250 ms)." : "Motion debug off.");
  }
  if (!enabled) return;
  peakLift = max(peakLift, fabsf(vertical));
  peakSide = max(peakSide, fabsf(side));
  peakPitch = max(peakPitch, fabsf(gyro[HAND_SIDE_AXIS]));
  peakRoll = max(peakRoll, fabsf(gyro[HAND_FORWARD_AXIS]));
  if (now - lastReportMs < 250) return;
  lastReportMs = now;
  Serial.printf("%s angle=%.0f deg | lift=%.1f side=%.1f m/s^2 | pitch=%.1f roll=%.1f rad/s\n",
                controlsEnabled ? "ON" : "LOCKED", palmAngle(), peakLift, peakSide, peakPitch, peakRoll);
  peakLift = peakSide = peakPitch = peakRoll = 0.0f;
}

void setup() {
  Serial.begin(115200);
  if (!Wire.begin(SDA_PIN, SCL_PIN, 100000)) {
    Serial.println("ERROR: Could not start I2C.");
    return;
  }
  if (!mpu.begin(sensorAddress, &Wire)) {
    sensorAddress = 0x69;
    if (!mpu.begin(sensorAddress, &Wire)) {
      Serial.println("ERROR: MPU6050 not found. Check SDA=21 and SCL=22.");
      return;
    }
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  delay(200);
  if (!calibrate()) {
    Serial.println("ERROR: Calibration failed. Check the sensor, then reset with your palm down and still.");
    return;
  }
  if (!startRadio()) {
    Serial.println("ERROR: ESP-NOW setup failed.");
    return;
  }
  lastSampleUs = micros();
  ready = true;
  Serial.println("CONTROLS LOCKED: flick your wrist up or down. Send d for motion debug.");
}

void loop() {
  const int result = sendResult.exchange(-1);
  if (result >= 0) {
    sendInFlight = false;
    if (result != ESP_NOW_SEND_SUCCESS) {
      Serial.println("Gesture send failed. Check receiver power and MAC.");
    } else if (lastSentCommand == FLICK_LEFT_COMMAND || lastSentCommand == FLICK_RIGHT_COMMAND) {
      Serial.println("Animation command delivered.");
    } else {
      lastDeliveredBrightness = lastSentCommand - BRIGHTNESS_SET_BASE;
      if (millis() - lastDeliveryLogMs >= 300) {
        lastDeliveryLogMs = millis();
        Serial.printf("Palm brightness delivered: %d/%d\n", lastDeliveredBrightness, PWM_MAX);
      }
    }
  }
  if (!ready) {
    delay(10);
    return;
  }
  const uint32_t nowUs = micros();
  const uint32_t elapsedUs = nowUs - lastSampleUs;
  if (elapsedUs < SAMPLE_MS * 1000) {
    sendPendingCommand(millis());
    delay(1);
    return;
  }
  lastSampleUs = nowUs;
  const float dt = min(elapsedUs * 1e-6f, 0.05f);
  const uint32_t now = millis();
  float accel[3], gyro[3];
  if (!readMotion(accel, gyro)) {
    Serial.println("ERROR: MPU6050 read failed. Controls locked. Check wiring and reset.");
    ready = false;
    controlsEnabled = false;
    pendingAnimation = 0;
    return;
  }
  for (size_t i = 0; i < 3; ++i) gyro[i] -= gyroBias[i];
  const float rotation = length(gyro);
  float vertical, side;
  measureHandAcceleration(accel, gyro, rotation, dt, vertical, side);
  updateHandControls(vertical, side, gyro, now);
  printMotionDebug(vertical, side, gyro, now);
  sendPendingCommand(now);
}
