// Ensure that PSRAM is enabled and the partition is set properly
// Also make sure to enable USB CDC on boot in the Tools menu in Arduino IDE

#include <Arduino.h>
#include "esp_camera.h"
#include <Adafruit_NeoPixel.h>
#include "board_config.h"

#define NEOPIXEL_PIN 48
#define NUM_PIXELS 1

float latestTemp = 0;
float latestHumidity = 0;

// Set when a Serial ALERT_MAGIC command arrives from the Python script.
// Cleared automatically after ALERT_DISPLAY_MS -- no timer/interrupt needed,
// just a millis() check in loop().
volatile bool personAlertActive = false;
volatile unsigned long personAlertSetAt = 0;
const unsigned long ALERT_DISPLAY_MS = 5000;

// Set when the Arduino's "M:1" motion line arrives. Bookkeeping only now --
// the old /sensor JSON endpoint that used to expose this is gone, so its
// only remaining job is clearing itself after 3s. (Happy to wire this into
// its own NeoPixel color -- e.g. blue -- if a "motion seen, not yet
// confirmed" indicator would be useful; just say the word.)
volatile bool hardwareMotionDetected = false;
volatile unsigned long motionDetectedAt = 0;

// Set when the Arduino's "T:"/"H:" line arrives. Flashes the NeoPixel green
// to show the ESP is genuinely receiving fresh sensor data, not stalled.
bool messageReceived = false;
unsigned long messageReceivedAt = 0;
const unsigned long MESSAGE_RECEIVED_FLASH_MS = 100;

Adafruit_NeoPixel pixel(NUM_PIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);
HardwareSerial ArduinoLink(1);  // Peripheral 1 -- independent of Serial's peripheral 0.

// Markers exchanged with the PC script over the single Serial byte stream.
static const uint8_t FRAME_MAGIC[4]  = {0xAA, 0x55, 0xAA, 0x55};  // ESP -> PC: a JPEG frame follows
static const uint8_t MOTION_MAGIC[4] = {0xBB, 0x66, 0xBB, 0x66};  // ESP -> PC: PIR motion event
static const uint8_t ALERT_MAGIC[4]  = {0xCC, 0x33, 0xCC, 0x33};  // PC -> ESP: person confirmed
static const uint8_t ENV_MAGIC[4]    = {0xDD, 0x44, 0xDD, 0x44};  // ESP -> PC: DHT11 Data

void parseLine(String line);
void checkForArduinoData();
void checkForAlertCommand();

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

  Serial.begin(115200);  // Native USB_OTG port: frames + motion events out,
                             // alert commands in. Baud is a formality for native
                             // USB CDC, not an actual throughput limit.

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.frame_size = FRAMESIZE_UXGA;
  config.pixel_format = PIXFORMAT_JPEG;
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 15;
  config.fb_count = 1;

  if (config.pixel_format == PIXFORMAT_JPEG) {
    if (psramFound()) {
      config.frame_size = FRAMESIZE_VGA;  // 320x240 -- realistic target for sustained fps
      config.jpeg_quality = 15;
      config.fb_count = 2;
      config.grab_mode = CAMERA_GRAB_LATEST;
      config.xclk_freq_hz = 20000000;
    } else {
      config.frame_size = FRAMESIZE_VGA;
      config.fb_location = CAMERA_FB_IN_DRAM;
    }
  } else {
    config.frame_size = FRAMESIZE_240X240;
#if CONFIG_IDF_TARGET_ESP32S3
    config.fb_count = 2;
#endif
  }

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x\n", err);
    return;
  }

  sensor_t *s = esp_camera_sensor_get();
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1);
    s->set_brightness(s, 1);
    s->set_saturation(s, -2);  // corrects the sensor's default oversaturation
  }

  // Explicit, though less critical now than it was over WiFi -- there's no
  // HTTP control panel left in this build that could silently override it.
  s->set_framesize(s, FRAMESIZE_VGA);

#if defined(CAMERA_MODEL_M5STACK_WIDE) || defined(CAMERA_MODEL_M5STACK_ESP32CAM)
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
#endif

#if defined(CAMERA_MODEL_ESP32S3_EYE)
  s->set_vflip(s, 1);
#endif

#if defined(LED_GPIO_NUM)
  setupLedFlash();
#endif

  ArduinoLink.begin(9600, SERIAL_8N1, 1, 21);  // RX=GPIO1 (used), TX=GPIO21 (unused direction)
  Serial.println("Waiting for Arduino data...");

  pixel.begin();
  pixel.setBrightness(50);
  pixel.show();

  Serial.println("Camera ready -- streaming frames over the native USB_OTG port.");
}

void loop() {
  checkForArduinoData();
  checkForAlertCommand();

  // ====================================
  // CORE LOGIC: Motion state bookkeeping
  // ====================================
  if (hardwareMotionDetected && (millis() - motionDetectedAt > 3000)) {
    hardwareMotionDetected = false;
  }

  // ==========================================
  // UI LOGIC: Unified non-blocking NeoPixel state machine
  // ==========================================
  static bool pixelShowingAlert = false;

  if (personAlertActive) {
    if (!pixelShowingAlert || pixel.getPixelColor(0) != pixel.Color(255, 0, 0)) {
      pixel.setPixelColor(0, pixel.Color(255, 0, 0));
      pixel.show();
      pixelShowingAlert = true;
    }
    if (millis() - personAlertSetAt > ALERT_DISPLAY_MS) {
      personAlertActive = false;
    }
  } else if (messageReceived) {
    if (!pixelShowingAlert || pixel.getPixelColor(0) != pixel.Color(0, 255, 0)) {
      pixel.setPixelColor(0, pixel.Color(0, 255, 0));
      pixel.show();
      pixelShowingAlert = true;
    }
    if (millis() - messageReceivedAt > MESSAGE_RECEIVED_FLASH_MS) {
      messageReceived = false;
    }
  } else if (pixelShowingAlert) {
    pixel.setPixelColor(0, pixel.Color(0, 0, 0));
    pixel.show();
    pixelShowingAlert = false;
  }

  // ====================================
  // VIDEO: capture and send one frame over USB
  // ====================================
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Frame capture failed");
    return;
  }
  if (fb->format != PIXFORMAT_JPEG) {
    Serial.println("Unexpected non-JPEG frame, skipping");
    esp_camera_fb_return(fb);
    return;
  }

  uint32_t len = fb->len;
  Serial.write(FRAME_MAGIC, sizeof(FRAME_MAGIC));
  Serial.write((uint8_t *)&len, sizeof(len));
  Serial.write(fb->buf, fb->len);

  esp_camera_fb_return(fb);
}

void checkForArduinoData() {
  static String buffer = "";
  while (ArduinoLink.available()) {
    char c = ArduinoLink.read();
    if (c == '\n') {
      parseLine(buffer);
      buffer = "";
    } else if (c != '\r') {
      buffer += c;
    }
  }
}

void checkForAlertCommand() {
  static uint8_t window[4] = {0, 0, 0, 0};
  while (Serial.available()) {
    window[0] = window[1];
    window[1] = window[2];
    window[2] = window[3];
    window[3] = Serial.read();
    if (window[0] == ALERT_MAGIC[0] && window[1] == ALERT_MAGIC[1] &&
        window[2] == ALERT_MAGIC[2] && window[3] == ALERT_MAGIC[3]) {
      personAlertActive = true;
      personAlertSetAt = millis();
    }
  }
}

void parseLine(String line) {
  int hIndex = line.indexOf(",H:");
  if (line.startsWith("T:") && hIndex > 0) {
    latestTemp = line.substring(2, hIndex).toFloat();
    latestHumidity = line.substring(hIndex + 3).toFloat();
    messageReceived = true;
    messageReceivedAt = millis();

    Serial.write(ENV_MAGIC, sizeof(ENV_MAGIC));
    Serial.write((uint8_t*)&latestTemp, sizeof(latestTemp));
    Serial.write((uint8_t*)&latestHumidity, sizeof(latestHumidity));

  } else if (line.startsWith("M:1")) {
    hardwareMotionDetected = true;
    motionDetectedAt = millis();
    Serial.write(MOTION_MAGIC, sizeof(MOTION_MAGIC));  // tells the PC script to run a detection burst
  }
}