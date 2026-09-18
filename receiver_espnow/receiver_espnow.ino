#include <WiFi.h>
#include <esp_now.h>

// Callback: runs whenever a packet arrives
void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  int counter;
  memcpy(&counter, data, sizeof(counter));
  Serial.println("Received: " + String(counter));
  digitalWrite(8, LOW); // Turn LED on
  delay(500);
  digitalWrite(8, HIGH); // Turn LED on
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  WiFi.mode(WIFI_STA); // ESP-NOW needs STA mode, but it does NOT connect to any network

  // Print this board's MAC address — you need to paste this into the SENDER sketch
  Serial.print("This receiver's MAC address: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("[FAILED] esp_now_init error");
    return;
  }
  
  pinMode(8, OUTPUT);
  esp_now_register_recv_cb(onDataRecv);
  Serial.println("Ready. Waiting for counter...");
}

void loop() {
  // Nothing needed here — onDataRecv fires automatically
}
