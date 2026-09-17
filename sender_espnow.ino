#include <WiFi.h>
#include <esp_now.h>

// ⚠️ REPLACE with the MAC address printed by the RECEIVER sketch's serial monitor
uint8_t receiverMAC[] = {0x70, 0xAF, 0x09, 0x21, 0x3D, 0x4C};

int counter = 0;

// Callback: confirms whether the last send succeeded
void onDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Send OK" : "Send FAILED");
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  WiFi.mode(WIFI_STA); // ESP-NOW needs STA mode, but it does NOT connect to any network

  if (esp_now_init() != ESP_OK) {
    Serial.println("[FAILED] esp_now_init error");
    return;
  }

  esp_now_register_send_cb(onDataSent);

  // Register the receiver as a peer
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverMAC, 6);
  peerInfo.channel = 0; // use current Wi-Fi channel
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("[FAILED] Could not add peer");
    return;
  }

  Serial.println("Ready. Sending counter...");
}

void loop() {
  esp_now_send(receiverMAC, (uint8_t*)&counter, sizeof(counter));
  Serial.println("Sending: " + String(counter));

  counter++;
  delay(1000);
}
