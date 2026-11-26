// ESP32 ESP-NOW gesture receiver driving an LED through a MOSFET.
// The hand controller sends absolute brightness targets. Older senders can still
// send up/down steps and right/left animation commands.
// Serial Monitor also works for testing: 0-1023 sets peak power, a0-a6 picks an
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
constexpr int32_t SLIDER_DELTA_BASE = 10000;
constexpr int32_t BRIGHTNESS_SET_BASE = 12000;

constexpr int PWM_PIN = 23;          // to MOSFET gate (through 220 ohm)
constexpr int PWM_FREQ = 20000;
constexpr int PWM_RES = 10;          // duty 0-1023
constexpr int PWM_MAX = (1 << PWM_RES) - 1;
constexpr int PWM_MIN_ON = 8;        // lowest "on" power; one more DOWN flick turns off
constexpr uint32_t ANIM_MS = 2000;   // time for one full animation cycle
constexpr float ANIM_LOW = 0.25f;    // floor for effects that stay lit

constexpr int ANIM_COUNT = 7;        // 0 = steady, 1-6 = animations
const char *const ANIM_NAMES[ANIM_COUNT] = {
  "steady", "pulse", "pulse from off",
  "breathe", "breathe from off", "heartbeat", "candle"
};

QueueHandle_t commands = nullptr;
bool ready = false;
int currentPower = PWM_MIN_ON;       // peak PWM for every mode, 0-1023
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
  Serial.printf("Animation %d: %s\n", animation, ANIM_NAMES[animation]);
}

// The effects return a fraction of the selected brightness, from 0 to 1.
float pulseLevel(uint32_t now) {
  const uint32_t phase = now % ANIM_MS;
  const uint32_t ramp = min(phase, ANIM_MS - phase);
  return ramp / (ANIM_MS / 2.0f);
}

float breatheLevel(uint32_t now) {
  const float phase = (now % ANIM_MS) / float(ANIM_MS);
  const float level = (1.0f - cosf(2.0f * PI * phase)) * 0.5f;
  return level * level;
}

float heartbeatLevel(uint32_t now) {
  const uint32_t t = now % 1200;
  if (t < 100) return t / 100.0f;
  if (t < 200) return (200 - t) / 100.0f;
  if (t >= 300 && t < 400) return 0.7f * (t - 300) / 100.0f;
  if (t >= 400 && t < 500) return 0.7f * (500 - t) / 100.0f;
  return 0.0f;
}

float candleLevel(uint32_t now) {
  static uint32_t lastChange = 0;
  static uint32_t holdMs = 0;
  static float target = 0.83f;
  static float level = target;
  if (now - lastChange >= holdMs) {
    target = random(80, 121) / 120.0f;
    lastChange = now;
    holdMs = random(80, 250);
  }
  level += (target - level) * 0.05f;
  return level;
}

void renderLamp() {
  const uint32_t now = millis();
  float level = 1.0f;
  switch (animation) {
    case 1: level = ANIM_LOW + (1.0f - ANIM_LOW) * pulseLevel(now);     break;
    case 2: level = pulseLevel(now);                                  break;
    case 3: level = ANIM_LOW + (1.0f - ANIM_LOW) * breatheLevel(now);   break;
    case 4: level = breatheLevel(now);                                break;
    case 5: level = ANIM_LOW + (1.0f - ANIM_LOW) * heartbeatLevel(now); break;
    case 6: level = candleLevel(now);                                break;
    default: break;
  }
  // Only this function writes PWM. The current slider value scales every
  // frame, including while the candle effect is easing to its next target.
  const int duty = int(currentPower * level + 0.5f);
  static int lastDuty = -1;
  if (duty != lastDuty && ledcWrite(PWM_PIN, duty)) lastDuty = duty;
}

// ---------------- Input handling ----------------

void handleGesture(int32_t command) {
  if (command >= BRIGHTNESS_SET_BASE && command <= BRIGHTNESS_SET_BASE + PWM_MAX) {
    const int power = command - BRIGHTNESS_SET_BASE;
    if (power != currentPower) setPower(power);
    return;
  }
  if (command >= SLIDER_DELTA_BASE - PWM_MAX && command <= SLIDER_DELTA_BASE + PWM_MAX) {
    const int delta = command - SLIDER_DELTA_BASE;
    const int power = constrain(currentPower + delta, 0, PWM_MAX);
    if (power != currentPower) setPower(power);
    return;
  }

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
  renderLamp();

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
  Serial.println("Ready for palm-angle brightness and legacy gesture commands.");
  Serial.println("Brightness adjusts in every animation mode.");
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
  renderLamp();
  delay(1);
}
