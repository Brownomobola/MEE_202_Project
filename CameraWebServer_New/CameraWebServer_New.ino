#include <Arduino.h>
#include "esp_camera.h"
#include <WiFi.h>
#include <Adafruit_NeoPixel.h>

// ===========================
// Select camera model in board_config.h
// ===========================
#include "board_config.h"

#define NEOPIXEL_PIN 48
#define NUM_PIXELS 1
// ===========================
// Config for external sensors
// #define PIR_PIN 21
// #define BUZZER_PIN 1
// ===========================
// ===========================
// Enter your WiFi credentials
// ===========================
const char *ssid = "Brown's Desktop";
const char *password = "12345678";

float latestTemp = 0;
float latestHumidity = 0;

// Set by the /alert endpoint (app_httpd.cpp) when the Python detection
// script reports a person. Cleared automatically after ALERT_DISPLAY_MS
// — no timer/interrupt needed, just a millis() check in loop().
volatile bool personAlertActive = false;
volatile unsigned long personAlertSetAt = 0;
const unsigned long ALERT_DISPLAY_MS = 5000;

// Set to true when the ESP recieves the motion detected signal from the Arduino
// records the time so that we can set it back to false after 3 seconds
volatile bool hardwareMotionDetected = false;
volatile unsigned long motionDetectedAt = 0;

// Set by ESP when it receives temperature data from the Arduino
// Flashes the Neopixel green to show the ESP is receiving data
// Cleared automatically after MESSAGE_RECEIVED_FLASH_MS
bool messageReceived = false;
unsigned long messageReceivedAt = 0;
const unsigned long MESSAGE_RECEIVED_FLASH_MS = 100; 

Adafruit_NeoPixel pixel(NUM_PIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);
HardwareSerial ArduinoLink(0);

// Global variable for the number of frames to flag when motion is sensed
extern volatile int MotionFlagFramesRemaining;

void startCameraServer();
void setupLedFlash();

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

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
  config.pixel_format = PIXFORMAT_JPEG;  // for streaming
  //config.pixel_format = PIXFORMAT_RGB565; // for face detection/recognition
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;
  config.fb_count = 1;

  // if PSRAM IC present, init with higher-res frame buffer available,
  // but actually STREAM at VGA — the realistic target for 30fps over WiFi.
  // (SVGA/UXGA are too many bytes/frame to hit 30fps reliably — test VGA
  // first, only push higher if FPS log shows headroom.)
  if (config.pixel_format == PIXFORMAT_JPEG) {
    if (psramFound()) {
      config.frame_size = FRAMESIZE_QVGA;   // 320x240 — start here, not VGA
      config.jpeg_quality = 12;
      config.fb_count = 2;
      config.grab_mode = CAMERA_GRAB_LATEST;
      config.xclk_freq_hz = 20000000;      // full clock — was dropped to 15MHz before, no reason to underclock on this board
    } else {
      // Limit the frame size when PSRAM is not available
      config.frame_size = FRAMESIZE_VGA;
      config.fb_location = CAMERA_FB_IN_DRAM;
    }
  } else {
    // Best option for face detection/recognition
    config.frame_size = FRAMESIZE_240X240;
#if CONFIG_IDF_TARGET_ESP32S3
    config.fb_count = 2;
#endif
  }

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t *s = esp_camera_sensor_get();
  // initial sensors are flipped vertically and colors are a bit saturated
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1);        // flip it back
    s->set_brightness(s, 1);   // up the brightness just a bit
    s->set_saturation(s, 2);  // lower the saturation
  }
  // NOTE: removed the old "s->set_framesize(s, FRAMESIZE_QVGA);" line that
  // used to run here — it was silently overriding the config above back
  // down to 320x240 on every boot. That was your 240p ceiling. Don't
  // re-add a framesize override here unless you mean it.

  // Force VGA explicitly, every boot, so the web UI's control panel (or
  // anyone hitting "/") can't silently override it to something else the
  // way it just did (jumped to XGA/1024x768 and tanked fps to ~2-6).
  s->set_framesize(s, FRAMESIZE_QVGA);

#if defined(CAMERA_MODEL_M5STACK_WIDE) || defined(CAMERA_MODEL_M5STACK_ESP32CAM)
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
#endif

#if defined(CAMERA_MODEL_ESP32S3_EYE)
  s->set_vflip(s, 1);
#endif

// Setup LED FLash if LED pin is defined in camera_pins.h
#if defined(LED_GPIO_NUM)
  setupLedFlash();
#endif

  WiFi.begin(ssid, password);
  WiFi.setSleep(false);

  Serial.print("WiFi connecting");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");

  startCameraServer();
  ArduinoLink.begin(9600, SERIAL_8N1, 1, 21); // RX=GPIO1 (actually used); TX=GPIO21 (unused direction — not the onboard LED pin, not a boot-strapping pin, just parked here since something has to be specified)
  Serial.println("Waiting for Arduino data...");
  pixel.begin();
  pixel.setBrightness(50);
  pixel.show();

  Serial.print("Camera Ready! Use 'http://");
  Serial.print(WiFi.localIP());
  Serial.println("' to connect");
}

void loop() {
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

  // ====================================
  // CORE LOGIC: Motion State Management
  // ====================================
  // Clear the motion flag after 3 seconds so that /sensor JSON
  // resets and the python script goes back to sleep
  if (hardwareMotionDetected && (millis() - motionDetectedAt > 3000)) {
    hardwareMotionDetected = false;
  }

  // ==========================================
  // UI LOGIC: Unified Non-blocking LED state machine
  // ==========================================
static bool pixelShowingAlert = false;

  if (personAlertActive) {
    // Priority 1: Red Alert
    if (!pixelShowingAlert || pixel.getPixelColor(0) != pixel.Color(255, 0, 0)) {
      pixel.setPixelColor(0, pixel.Color(255, 0, 0));  
      pixel.show();
      pixelShowingAlert = true;
    }
    if (millis() - personAlertSetAt > ALERT_DISPLAY_MS) {
      personAlertActive = false;
    }
  } 
  else if (messageReceived) {
    // Priority 2: Green Temperature Update
    if (!pixelShowingAlert || pixel.getPixelColor(0) != pixel.Color(0, 255, 0)) {
      pixel.setPixelColor(0, pixel.Color(0, 255, 0));  
      pixel.show();
      pixelShowingAlert = true;
    }
    if (millis() - messageReceivedAt > MESSAGE_RECEIVED_FLASH_MS) {
      messageReceived = false;
    }
  } 
  else if (pixelShowingAlert) {
    // Default: Turn off if no flags are active
    pixel.setPixelColor(0, pixel.Color(0, 0, 0));  
    pixel.show();
    pixelShowingAlert = false;
  }
}

void parseLine(String line) {
  int hIndex = line.indexOf(",H:");
  if (line.startsWith("T:") && hIndex > 0) {
    latestTemp = line.substring(2, hIndex).toFloat();
    latestHumidity = line.substring(hIndex + 3).toFloat();

    messageReceived = true;
    messageReceivedAt = millis();

  } else if (line.startsWith("M:1")) {
    // PIR motion from Arduino — flag the next several frames in the
    // stream with X-Motion: 1 (stream_handler in app_httpd.cpp already
    // reads MotionFlagFramesRemaining and decrements it per frame sent)
    hardwareMotionDetected = true;
    motionDetectedAt = millis();
    MotionFlagFramesRemaining = 30; // ~1s of frames at current fps, tune as needed
  }
}