#include <DHT.h>

#define DHTPIN 2
#define DHTTYPE DHT11
#define LED_PIN 13   // onboard "L" LED

const int pirPin = 3;
volatile bool motionDetected = false;

DHT dht(DHTPIN, DHTTYPE);

unsigned long lastRead = 0;
const unsigned long interval = 2000;

// Non-blocking LED flash state -- no delay() here, so it can't interfere
// with PIR interrupt timing or slow down the main loop.
unsigned long ledOffAt = 0;
const unsigned long LED_FLASH_MS = 300;
volatile unsigned long lastTriggerTime = 0;
const unsigned long DEBOUNCE_MS = 1000;

void flashLed() {
  digitalWrite(LED_PIN, HIGH);
  ledOffAt = millis() + LED_FLASH_MS;
}

void setup() {
  digitalWrite(LED_PIN, HIGH);
  delay(1000);
  digitalWrite(LED_PIN, LOW);

  Serial.begin(9600);   // wired directly to ESP32 RX/TX (through a voltage divider)
                         // — disconnect this wire before uploading new sketches
  dht.begin();

  pinMode(pirPin, INPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  attachInterrupt(digitalPinToInterrupt(pirPin), pirISR, RISING);
}

void loop() {
  // Turn the LED back off once its flash window has elapsed. Checked every
  // loop iteration, but cheap -- just a comparison, no blocking.
  if (ledOffAt != 0 && millis() >= ledOffAt) {
    digitalWrite(LED_PIN, LOW);
    ledOffAt = 0;
  }

  unsigned long now = millis();
  if (now - lastRead >= interval) {
    lastRead = now;
    float h = dht.readHumidity();
    float t = dht.readTemperature();
    if (!isnan(h) && !isnan(t)) {
      Serial.print("T:");
      Serial.print(t);
      Serial.print(",H:");
      Serial.println(h);
      flashLed();
    }
  }

  if (motionDetected) {
    Serial.println("M:1");  // matches parseLine handling on the ESP32 side
    flashLed();
    motionDetected = false;
  }
}

void pirISR() {
  unsigned long now = millis();
  if (now - lastTriggerTime > DEBOUNCE_MS) {
    motionDetected = true;
    lastTriggerTime = now;
  }
}