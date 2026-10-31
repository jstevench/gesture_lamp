#define PIN_A 4
#define PIN_B 5

void setup() {
  Serial.begin(115200);
  
  // Enable internal pull-up resistors 
  // (Pin reads 1 normally, reads 0 when connected to GND)
  pinMode(PIN_A, INPUT_PULLUP);
  pinMode(PIN_B, INPUT_PULLUP);
  
  Serial.println("Starting GPIO 4 and GPIO 5 input test...");
}

void loop() {
  int valA = digitalRead(PIN_A);
  int valB = digitalRead(PIN_B);
  
  Serial.print("GPIO 4: ");
  Serial.print(valA);
  Serial.print(" | GPIO 5: ");
  Serial.println(valB);
  
  delay(200); // Check 5 times per second
}
