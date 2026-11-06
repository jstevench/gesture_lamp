#include <WiFi.h>
#include <esp_now.h>
#include <esp_mac.h>
#include <esp_wifi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

constexpr uint8_t RADIO_CHANNEL = 1;

constexpr int PWM_PIN = 23;         // to MOSFET gate (through 220 ohm)
constexpr int PWM_FREQ = 5000;
constexpr int PWM_RES = 8;          // duty 0-255
constexpr int PWM_STEP = 10;

QueueHandle_t commands = nullptr;
bool ready = false;
int pwmPower = 50;

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

void setup() {
  Serial.begin(115200);
  delay(1000);

  ledcAttach(PWM_PIN, PWM_FREQ, PWM_RES);
  ledcWrite(PWM_PIN, pwmPower);

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
}

void loop() {
  if (!ready) {
    delay(10);
    return;
  }

  int32_t command;
  if (xQueueReceive(commands, &command, 0) == pdTRUE) {
    if (command == 1) {
      pwmPower = constrain(pwmPower + PWM_STEP, 0, 255);
      ledcWrite(PWM_PIN, pwmPower);
      Serial.printf("Received: FLICK UP (+1) | POWER: %d\n", pwmPower);
    } else if (command == -1) {
      pwmPower = constrain(pwmPower - PWM_STEP, 0, 255);
      ledcWrite(PWM_PIN, pwmPower);
      Serial.printf("Received: FLICK DOWN (-1) | POWER: %d\n", pwmPower);
    } else {
      Serial.printf("Received: %ld (unknown command)\n", static_cast<long>(command));
    }
  }
  delay(1);
}