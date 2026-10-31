#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

constexpr uint8_t RADIO_CHANNEL = 1;
constexpr int LED_PIN = 8; // Active-low onboard LED on the C3 receiver.
QueueHandle_t commands = nullptr;
bool ready = false;
bool ledOn = false;
uint32_t ledStarted = 0;

// Callback: runs whenever a packet arrives
void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len != sizeof(int32_t)) return;
  int32_t command;
  memcpy(&command, data, sizeof(command));
  // Do not print or delay in the Wi-Fi callback task.
  xQueueSend(commands, &command, 0);
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
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

  // Print this board's MAC address 鈥?you need to paste this into the SENDER sketch
  Serial.print("This receiver's MAC address: ");
  Serial.println(WiFi.macAddress());

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
    if (command == 1) Serial.println("Received: FLICK UP (1)");
    else if (command == -1) Serial.println("Received: FLICK DOWN (-1)");
    else Serial.printf("Received: %ld (unknown command)\n", static_cast<long>(command));
    digitalWrite(LED_PIN, LOW);
    ledOn = true;
    ledStarted = millis();
  }
  if (ledOn && millis() - ledStarted >= 500) {
    digitalWrite(LED_PIN, HIGH);
    ledOn = false;
  }
  delay(1);
}
