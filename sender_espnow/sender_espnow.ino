#include <WiFi.h>
#include <esp_now.h>
#define BUTTON_PIN 9 // Onboard BOOT button for ESP32-C3

// 'volatile' tells the compiler to always read the variable fresh from RAM
volatile bool buttonPressed = false; 

// The Interrupt Service Routine (ISR) - Must be fast and light
void IRAM_ATTR handleButtonPress() {
  buttonPressed = true;
}
// REPLACE with the MAC address printed by the RECEIVER sketch's serial monitor
uint8_t receiverMAC[] = {0x70, 0xAF, 0x09, 0x0D, 0xD0, 0xA8};

int counter = 0;

// Callback: confirms whether the last send succeeded
void onDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Send OK" : "Send FAILED");
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
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
  
  // Attach the interrupt to the pin, point it to the ISR, and watch for a FALLING edge
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), handleButtonPress, FALLING);

  Serial.println("Ready. Sending counter...");
}

void loop() {

  if (buttonPressed) {
    Serial.println("Interrupt triggered: Onboard BOOT button pressed!");
    
    esp_now_send(receiverMAC, (uint8_t*)&counter, sizeof(counter));
    Serial.println("Sending: " + String(counter));
    delay(1000);

    // Reset the flag
    buttonPressed = false;
  }
  delay(10);
}
