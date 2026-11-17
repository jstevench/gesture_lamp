// ESP32 ESP-NOW gesture receiver driving an LED through a MOSFET.
//   Flick UP / DOWN    -> power doubles / halves (8, 16, 32 ... 1023); DOWN from 8 -> off
//   Flick RIGHT / LEFT -> next / previous animation (wraps around)
// Serial Monitor also works for testing: 0-1023 sets power, a0-a6 picks an
// animation (a0 = steady), s = stop animation.
#include <WiFi.h>
#include <esp_now.h>
#include <esp_mac.h>
#include <esp_wifi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

constexpr uint8_t RADIO_CHANNEL = 1;

// Must match the sender's command values.
constexpr int32_t FLICK_UP_COMMAND = 1;
constexpr int32_t FLICK_DOWN_COMMAND = -1;
constexpr int32_t FLICK_RIGHT_COMMAND = 2;
constexpr int32_t FLICK_LEFT_COMMAND = -2;

constexpr int PWM_PIN = 23;          // to MOSFET gate (through 220 ohm)
constexpr int PWM_FREQ = 20000;
constexpr int PWM_RES = 10;          // duty 0-1023
constexpr int PWM_MAX = (1 << PWM_RES) - 1;
constexpr int PWM_MIN_ON = 8;        // lowest "on" power; one more DOWN flick turns off
constexpr uint32_t ANIM_MS = 2000;   // time for one full animation cycle

constexpr int ANIM_COUNT = 7;        // 0 = steady, 1-6 = animations
const char *const ANIM_NAMES[ANIM_COUNT] = {
  "steady", "pulse power/2 to power*2", "pulse 0 to power*2",
  "breathe", "breathe 0 to power*2", "heartbeat", "candle"
};

QueueHandle_t commands = nullptr;
bool ready = false;
int currentPower = PWM_MIN_ON;       // base brightness, 0-1023
int animation = 0;

void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len != sizeof(int32_t)) return;
  int32_t command;
  memcpy(&command, data, sizeof(command));
  xQueueSend(commands, &command, 0);
}

void printMac() {
  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
  Serial.printf("This receiver's MAC address: %02X:%02X:%02X:%02X:%02X:%02X\n",
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

// ---------------- LED helpers and animations ----------------

void setPower(int p) {
  currentPower = constrain(p, 0, PWM_MAX);
  if (animation == 0) ledcWrite(PWM_PIN, currentPower);  // animations drive the pin themselves
  Serial.printf("Power: %d/%d\n", currentPower, PWM_MAX);
}

// UP: off -> 8, then double each flick (capped at the maximum).
void powerUp() {
  setPower(currentPower < PWM_MIN_ON ? PWM_MIN_ON : currentPower * 2);
}

// DOWN: halve each flick; at 8 (or lower) the next flick goes to 0.
void powerDown() {
  if (currentPower <= PWM_MIN_ON) setPower(0);
  else if (currentPower == PWM_MAX) setPower((PWM_MAX + 1) / 2);  // 1023 -> 512
  else setPower(currentPower / 2);
}

void setAnimation(int a) {
  animation = ((a % ANIM_COUNT) + ANIM_COUNT) % ANIM_COUNT;
  if (animation == 0) ledcWrite(PWM_PIN, currentPower);
  Serial.printf("Animation %d: %s\n", animation, ANIM_NAMES[animation]);
}

// Smooth triangle wave between lo and hi, repeating every animMs.
void triangleWave(int lo, int hi, uint32_t animMs) {
  uint32_t phase = millis() % animMs;
  uint32_t half = animMs / 2;
  uint32_t x = (phase < half) ? phase : (animMs - phase);   // 0..half..0
  int duty = lo + (long)(hi - lo) * x / half;
  ledcWrite(PWM_PIN, duty);
}

// 1: pulses from power/2 to power*2
void pulseHalfToDouble(int power, uint32_t animMs) {
  triangleWave(power / 2, constrain(power * 2, 0, PWM_MAX), animMs);
}

// 2: pulses from 0 to power*2
void pulseZeroToDouble(int power, uint32_t animMs) {
  triangleWave(0, constrain(power * 2, 0, PWM_MAX), animMs);
}

// 3: sine-wave breathing between power/2 and power*2
void breathe(int power, uint32_t animMs) {
  int lo = power / 2;
  int hi = constrain(power * 2, 0, PWM_MAX);
  float t = (millis() % animMs) / (float)animMs;
  float s = (1 - cos(t * 2 * PI)) / 2;     // 0 to 1 and back
  s = s * s;                               // gamma-ish, looks smoother to the eye
  ledcWrite(PWM_PIN, lo + (int)((hi - lo) * s));
}

// 4: sine-wave breathing between 0 and power*2
void breatheZeroToDouble(int power, uint32_t animMs) {
  int hi = constrain(power * 2, 0, PWM_MAX);
  float t = (millis() % animMs) / (float)animMs;
  float s = (1 - cos(t * 2 * PI)) / 2;
  s = s * s;
  ledcWrite(PWM_PIN, (int)(hi * s));
}

// 5: two quick bumps, then a pause (rests at power/2, peaks at power*2)
void heartbeat(int power) {
  int lo = power / 2;
  int hi = constrain(power * 2, 0, PWM_MAX);
  uint32_t t = millis() % 1200;            // one beat every 1.2 s
  float s = 0;
  if      (t < 100) s = t / 100.0;                       // first bump up
  else if (t < 200) s = 1 - (t - 100) / 100.0;           // first bump down
  else if (t < 300) s = 0;
  else if (t < 400) s = 0.7 * (t - 300) / 100.0;         // second bump up
  else if (t < 500) s = 0.7 * (1 - (t - 400) / 100.0);   // second bump down
  ledcWrite(PWM_PIN, lo + (int)((hi - lo) * s));
}

// 6: gentle candle flicker around the current power
void candle(int power) {
  static uint32_t next = 0;
  static int target = 0;
  static float cur = 0;
  if (millis() >= next) {
    int lo = power * 8 / 10;                           // 80% of power
    int hi = constrain(power * 12 / 10, 0, PWM_MAX);   // 120% of power
    target = random(lo, hi + 1);
    next = millis() + random(80, 250);                 // new target every 80-250 ms
  }
  cur += (target - cur) * 0.05;                        // slow easing
  ledcWrite(PWM_PIN, (int)cur);
}

void runAnimation() {
  switch (animation) {
    case 1: pulseHalfToDouble(currentPower, ANIM_MS);   break;
    case 2: pulseZeroToDouble(currentPower, ANIM_MS);   break;
    case 3: breathe(currentPower, ANIM_MS);             break;
    case 4: breatheZeroToDouble(currentPower, ANIM_MS); break;
    case 5: heartbeat(currentPower);                    break;
    case 6: candle(currentPower);                       break;
    default: break;
  }
}

// ---------------- Input handling ----------------

void handleGesture(int32_t command) {
  switch (command) {
    case FLICK_UP_COMMAND:
      Serial.print("Received: FLICK UP | ");
      powerUp();
      break;
    case FLICK_DOWN_COMMAND:
      Serial.print("Received: FLICK DOWN | ");
      powerDown();
      break;
    case FLICK_RIGHT_COMMAND:
      Serial.print("Received: FLICK RIGHT | Next. ");
      setAnimation(animation + 1);
      break;
    case FLICK_LEFT_COMMAND:
      Serial.print("Received: FLICK LEFT | Previous. ");
      setAnimation(animation - 1);
      break;
    default:
      Serial.printf("Received: %ld (unknown command)\n", static_cast<long>(command));
      break;
  }
}

void handleSerial() {
  if (!Serial.available()) return;
  String input = Serial.readStringUntil('\n');
  input.trim();
  if (input.length() == 0) return;

  if (input == "s") {
    setAnimation(0);
  } else if (input.length() == 2 && input[0] == 'a' && isDigit(input[1]) &&
             input[1] - '0' < ANIM_COUNT) {
    setAnimation(input[1] - '0');
  } else if (input.toInt() != 0 || input == "0") {
    setPower(input.toInt());
  } else {
    Serial.printf("Not understood. Enter 0-%d, a0-a%d, or s.\n", PWM_MAX, ANIM_COUNT - 1);
  }
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(20);   // don't stall the animation waiting on input
  delay(1000);

  ledcAttach(PWM_PIN, PWM_FREQ, PWM_RES);
  ledcWrite(PWM_PIN, currentPower);

  if (!WiFi.mode(WIFI_STA)) {
    Serial.println("ERROR: Could not start Wi-Fi.");
    return;
  }
  WiFi.setSleep(false);
  if (esp_wifi_set_channel(RADIO_CHANNEL, WIFI_SECOND_CHAN_NONE) != ESP_OK) {
    Serial.println("ERROR: Could not select radio channel.");
    return;
  }
  commands = xQueueCreate(8, sizeof(int32_t));
  if (commands == nullptr) {
    Serial.println("ERROR: Could not allocate receive queue.");
    return;
  }

  printMac();

  if (esp_now_init() != ESP_OK) {
    Serial.println("[FAILED] esp_now_init error");
    return;
  }
  if (esp_now_register_recv_cb(onDataRecv) != ESP_OK) {
    Serial.println("ERROR: Could not register receive callback.");
    return;
  }
  ready = true;
  Serial.println("Ready. Waiting for gestures on channel 1...");
  Serial.println("UP/DOWN = brightness, LEFT/RIGHT = previous/next animation.");
  Serial.printf("Serial: 0-%d sets power, a0-a%d picks an animation, s = stop.\n",
                PWM_MAX, ANIM_COUNT - 1);
}

void loop() {
  if (!ready) {
    delay(10);
    return;
  }

  int32_t command;
  if (xQueueReceive(commands, &command, 0) == pdTRUE) handleGesture(command);

  handleSerial();
  runAnimation();
  delay(1);
}
