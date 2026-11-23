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
constexpr int32_t SLIDER_DELTA_BASE = 10000;
constexpr int32_t FLICK_RIGHT_COMMAND = 2;
constexpr int32_t FLICK_LEFT_COMMAND = -2;
constexpr int BRIGHTNESS_STEP = 15;
constexpr uint32_t BRIGHTNESS_STEP_INTERVAL_MS = 50;

// Mount sensor X along the hand (wrist to fingertips). Set to 1 if Y is
// along the hand instead. Either direction of the twist toggles the slider.
constexpr uint8_t WRIST_ROLL_AXIS = 0;
constexpr int SIDE_SIGN = 1; // Change to -1 if left and right are swapped.
constexpr float SIDE_FLICK_START_MS2 = 8.0f;
constexpr float SIDE_FLICK_BRAKE_MS2 = 3.2f;
constexpr float SIDE_QUIET_MS2 = 1.1f;
constexpr uint8_t SIDE_START_SAMPLES = 3;
constexpr uint8_t SIDE_BRAKE_SAMPLES = 2;
constexpr uint32_t SIDE_FLICK_WINDOW_MS = 220;
constexpr uint32_t SIDE_COOLDOWN_MS = 450;
constexpr float TWIST_START_RAD_S = 1.5f;
constexpr float TWIST_CONTINUE_RAD_S = 0.75f;
constexpr float TWIST_ANGLE_RAD = 0.58f;
constexpr uint32_t TWIST_WINDOW_MS = 650;
constexpr uint32_t TWIST_COOLDOWN_MS = 650;
constexpr uint32_t ROTATION_SETTLE_MS = 180;

constexpr float VERTICAL_FILTER_S = 0.040f;
constexpr float VERTICAL_ACCEL_DEADBAND = 0.20f; // m/s²
constexpr float MAX_VERTICAL_ACCEL = 6.0f;      // m/s²
constexpr float VERTICAL_START_SPEED = 0.050f;  // m/s
constexpr float VERTICAL_STOP_SPEED = 0.028f;   // m/s
constexpr float VERTICAL_REVERSE_SPEED = 0.065f;
constexpr float DIRECTION_ACCEL_MS2 = 0.06f;
constexpr float MAX_VERTICAL_SPEED = 0.80f;     // m/s
constexpr float VELOCITY_DECAY_S = 2.60f;
constexpr float QUIET_DECAY_S = 0.11f;
constexpr float BRAKE_START_MS2 = 0.12f;
constexpr float GENTLE_BRAKE_MAX_MS2 = 0.85f;
constexpr float REVERSAL_DECAY_S = 0.23f;
constexpr float BRAKE_CONFIRM_S = 0.040f;
constexpr uint32_t VERTICAL_QUIET_MS = 260;
constexpr uint32_t VERTICAL_REARM_MS = 180;
constexpr uint32_t VERTICAL_CONFIRM_MS = 30;
constexpr uint32_t VERTICAL_REVERSE_CONFIRM_MS = 45;

Adafruit_MPU6050 mpu;
uint8_t sensorAddress = 0x68;
bool ready = false;
float gravity[3] = {};
float gyroBias[3] = {};
float gravityMagnitude = 0.0f;
float sideMountSign = 1.0f;
uint32_t lastSampleUs = 0;

bool sliderEnabled = false;
bool twistLatched = false;
float twistAngle = 0.0f;
uint32_t twistStartedMs = 0;
uint32_t lastTwistMs = 0;
uint32_t lastRotationMs = 0;

float filteredVertical = 0.0f;
float verticalSpeed = 0.0f;
int brightnessDirection = 0;
int lastBrightnessDirection = 0;
int brightnessCandidate = 0;
float brakingSeconds = 0.0f;
uint32_t brightnessCandidateMs = 0;
uint32_t lastVerticalMotionMs = 0;
int sideCandidate = 0;
uint8_t sideStartSamples = 0;
uint8_t sideBrakeSamples = 0;
uint32_t sideStartedMs = 0;
bool sideLatched = false;
uint32_t lastSideFlickMs = 0;
int32_t pendingAnimation = 0;
uint32_t lastBrightnessStepMs = 0;
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
  if (!sliderEnabled || now - lastTwistMs < ROTATION_SETTLE_MS || sendInFlight) return;
  if (pendingAnimation != 0) {
    const int32_t command = pendingAnimation;
    pendingAnimation = 0;
    sendCommand(command);
    return;
  }
  if (brightnessDirection == 0 || now - lastBrightnessStepMs < BRIGHTNESS_STEP_INTERVAL_MS) return;
  // Send one fixed step at the current cadence. Never queue missed steps.
  lastBrightnessStepMs = now;
  sendCommand(SLIDER_DELTA_BASE + brightnessDirection * BRIGHTNESS_STEP);
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
  Serial.println("Hold your hand still for calibration.");
  float accelSum[3] = {}, gyroSum[3] = {};
  int count = 0;
  const uint32_t start = millis();
  while (millis() - start < 15000) {
    float accel[3], gyro[3];
    if (!readMotion(accel, gyro)) return false;
    if (fabsf(length(accel) - 9.80665f) > 1.0f || length(gyro) > 0.35f) {
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
        sideMountSign = gravity[2] >= 0.0f ? SIDE_SIGN : -SIDE_SIGN;
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

  // Only correct gyro drift when acceleration is close to gravity.
  if (fabsf(verticalSpeed) < VERTICAL_STOP_SPEED && rotation < 0.55f &&
      fabsf(length(accel) - gravityMagnitude) < 0.30f) {
    for (size_t i = 0; i < 3; ++i) gravity[i] += (accel[i] - gravity[i]) * 0.045f;
  }
  const float scale = gravityMagnitude / length(gravity);
  for (size_t i = 0; i < 3; ++i) gravity[i] *= scale;
  const float up = (accel[0]*gravity[0] + accel[1]*gravity[1] + accel[2]*gravity[2]) /
      gravityMagnitude;
  vertical = up - gravityMagnitude;

  // Use the horizontal direction across the hand. This remains usable after
  // a wrist roll, even if the sensor's sideways axis now points straight up.
  const uint8_t crossAxis = WRIST_ROLL_AXIS == 0 ? 1 : 0;
  const float horizontalLength = hypotf(gravity[crossAxis], gravity[2]);
  side = horizontalLength > 0.3f * gravityMagnitude
      ? sideMountSign * (accel[crossAxis]*gravity[2] - accel[2]*gravity[crossAxis]) /
          horizontalLength : 0.0f;
}

void resetHandControls(uint32_t now) {
  filteredVertical = 0.0f;
  verticalSpeed = 0.0f;
  brightnessDirection = 0;
  lastBrightnessDirection = 0;
  brightnessCandidate = 0;
  brakingSeconds = 0.0f;
  lastVerticalMotionMs = now;
  sideCandidate = 0;
  sideStartSamples = sideBrakeSamples = 0;
  sideLatched = false;
}

void updateTwist(float roll, float rotation, float dt, uint32_t now) {
  if (rotation > TWIST_CONTINUE_RAD_S) lastRotationMs = now;
  if (twistLatched) {
    if (now - lastTwistMs >= TWIST_COOLDOWN_MS && now - lastRotationMs >= ROTATION_SETTLE_MS) {
      twistLatched = false;
    }
    return;
  }

  const float speed = fabsf(roll);
  if (speed < TWIST_CONTINUE_RAD_S || speed < rotation * 0.75f ||
      (twistAngle != 0.0f && (now - twistStartedMs > TWIST_WINDOW_MS || twistAngle * roll < 0))) {
    twistAngle = 0.0f;
    return;
  }
  if (twistAngle == 0.0f) {
    if (speed < TWIST_START_RAD_S) return;
    twistStartedMs = now;
  }
  twistAngle += roll * dt;
  if (fabsf(twistAngle) < TWIST_ANGLE_RAD) return;

  sliderEnabled = !sliderEnabled;
  twistLatched = true;
  lastTwistMs = now;
  twistAngle = 0.0f;
  pendingAnimation = 0;
  resetHandControls(now);
  Serial.println(sliderEnabled ? "SLIDER ON: +/-15 every 50 ms while moving. Flick left/right for animations."
                               : "SLIDER LOCKED: hand gestures paused.");
}

void updateBrightness(float vertical, float dt, uint32_t now) {
  filteredVertical += (vertical - filteredVertical) * dt / (VERTICAL_FILTER_S + dt);
  const float accel = copysignf(constrain(fabsf(filteredVertical) - VERTICAL_ACCEL_DEADBAND,
      0.0f, MAX_VERTICAL_ACCEL), filteredVertical);
  if (fabsf(accel) >= DIRECTION_ACCEL_MS2) lastVerticalMotionMs = now;
  const uint32_t quietMs = now - lastVerticalMotionMs;

  const float opposingAccel = -lastBrightnessDirection * accel;
  const bool braking = opposingAccel >= BRAKE_START_MS2;
  brakingSeconds = braking ? brakingSeconds + dt : 0.0f;
  float decay = VELOCITY_DECAY_S;
  if (quietMs >= VERTICAL_QUIET_MS) {
    decay = QUIET_DECAY_S;
  } else if (braking && opposingAccel <= GENTLE_BRAKE_MAX_MS2 &&
             lastBrightnessDirection * verticalSpeed > 0.0f) {
    // A gentle reversal must not spend a long time cancelling stale speed.
    // With hard braking, keep the measured impulse to avoid a false reversal.
    decay = REVERSAL_DECAY_S;
  }
  verticalSpeed = constrain((verticalSpeed + accel * dt) /
      (1.0f + dt / decay), -MAX_VERTICAL_SPEED, MAX_VERTICAL_SPEED);

  // Stop the old steps before waiting for the new direction to qualify.
  const float speed = fabsf(verticalSpeed);
  if (brightnessDirection != 0 &&
      (brightnessDirection * verticalSpeed <= VERTICAL_STOP_SPEED || brakingSeconds >= BRAKE_CONFIRM_S)) {
    brightnessDirection = 0;
  }
  if (quietMs >= VERTICAL_REARM_MS && speed <= VERTICAL_STOP_SPEED) lastBrightnessDirection = 0;

  const int direction = verticalSpeed > 0.0f ? 1 : -1;
  const bool reversing = lastBrightnessDirection != 0 && direction != lastBrightnessDirection;
  const float threshold = reversing ? VERTICAL_REVERSE_SPEED : VERTICAL_START_SPEED;
  if (direction == brightnessDirection || speed < threshold || direction * accel < DIRECTION_ACCEL_MS2) {
    brightnessCandidate = 0;
    return;
  }
  if (direction != brightnessCandidate) {
    brightnessCandidate = direction;
    brightnessCandidateMs = now;
    return;
  }
  const uint32_t confirmMs = reversing ? VERTICAL_REVERSE_CONFIRM_MS : VERTICAL_CONFIRM_MS;
  if (now - brightnessCandidateMs < confirmMs) return;

  brightnessDirection = direction;
  lastBrightnessDirection = direction;
  brightnessCandidate = 0;
  brakingSeconds = 0.0f;
  Serial.println(direction > 0 ? "Brightness sliding UP" : "Brightness sliding DOWN");
}

void updateSideFlick(float side, uint32_t now) {
  // Sideways flicks use only the horizontal projection so a lift or lower
  // can continue at the same time.
  const float strength = fabsf(side);
  if (sideLatched) {
    // Only the sideways detector has a cooldown. Brightness keeps updating.
    if (now - lastSideFlickMs >= SIDE_COOLDOWN_MS && strength <= SIDE_QUIET_MS2) {
      sideLatched = false;
    }
    return;
  }
  if (sideStartSamples < SIDE_START_SAMPLES) {
    const int direction = strength < SIDE_FLICK_START_MS2
        ? 0 : (side > 0.0f ? 1 : -1);
    if (direction != sideCandidate) sideStartSamples = 0;
    sideCandidate = direction;
    if (direction != 0 && ++sideStartSamples == SIDE_START_SAMPLES) sideStartedMs = now;
    return;
  }

  if (now - sideStartedMs > SIDE_FLICK_WINDOW_MS) {
    sideCandidate = 0;
    sideStartSamples = sideBrakeSamples = 0;
    return;
  }
  // A short sideways burst followed by braking is a more reliable flick
  // than a single peak caused by tilt or spillover from a vertical lift.
  if (side * sideCandidate > -SIDE_FLICK_BRAKE_MS2) {
    sideBrakeSamples = 0;
    return;
  }
  if (++sideBrakeSamples < SIDE_BRAKE_SAMPLES) return;

  pendingAnimation = sideCandidate > 0 ? FLICK_RIGHT_COMMAND : FLICK_LEFT_COMMAND;
  Serial.println(sideCandidate > 0 ? "FLICK RIGHT: next animation." : "FLICK LEFT: previous animation.");
  lastSideFlickMs = now;
  sideLatched = true;
  sideCandidate = 0;
  sideStartSamples = sideBrakeSamples = 0;
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
    Serial.println("ERROR: Calibration failed. Check the sensor and reset while holding still.");
    return;
  }
  if (!startRadio()) {
    Serial.println("ERROR: ESP-NOW setup failed.");
    return;
  }
  lastSampleUs = micros();
  ready = true;
  Serial.println("SLIDER LOCKED: twist your wrist to enable it.");
  Serial.println("Serial radio test: send t for one brightness increase.");
}

void loop() {
  const int result = sendResult.exchange(-1);
  if (result >= 0) {
    sendInFlight = false;
    if (result != ESP_NOW_SEND_SUCCESS) {
      Serial.println("Gesture send failed. Check receiver power and MAC.");
    } else if (lastSentCommand == FLICK_LEFT_COMMAND || lastSentCommand == FLICK_RIGHT_COMMAND) {
      Serial.println("Animation command delivered.");
    } else if (millis() - lastDeliveryLogMs >= 300) {
      lastDeliveryLogMs = millis();
      Serial.printf("Brightness command delivered: %+ld\n",
                    static_cast<long>(lastSentCommand - SLIDER_DELTA_BASE));
    }
  }
  if (!ready) {
    delay(10);
    return;
  }
  if (Serial.available() && Serial.read() == 't') {
    if (sendInFlight) {
      Serial.println("Radio is busy. Send t again while holding still.");
    } else {
      Serial.println("Sending one brightness test to the lamp.");
      sendCommand(SLIDER_DELTA_BASE + BRIGHTNESS_STEP);
    }
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
    Serial.println("ERROR: MPU6050 read failed. Slider locked. Check wiring and reset.");
    ready = false;
    sliderEnabled = false;
    pendingAnimation = 0;
    return;
  }
  for (size_t i = 0; i < 3; ++i) gyro[i] -= gyroBias[i];
  const float rotation = length(gyro);
  float vertical, side;
  measureHandAcceleration(accel, gyro, rotation, dt, vertical, side);
  updateTwist(gyro[WRIST_ROLL_AXIS], rotation, dt, now);

  if (!sliderEnabled || now - lastTwistMs < ROTATION_SETTLE_MS) return;
  updateBrightness(vertical, dt, now);
  updateSideFlick(side, now);
  sendPendingCommand(now);
}
