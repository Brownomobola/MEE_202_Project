# ESP32-S3 Security Camera with Person Detection

A live-view security camera system built for MEE 202, combining an ESP32-S3
camera board, an Arduino Uno for sensor input, and a PC-side Python script
that runs YOLO-based person detection and sends Telegram alerts.

## Hardware

- **ESP32-S3 N16R8 module** (16MB flash, 8MB octal PSRAM) with an OV3660
  camera sensor, using the `CAMERA_MODEL_ESP32S3_EYE` pin mapping.
- **Two separate USB ports** on the ESP32-S3 board:
  - `USB-UART` — via an onboard CH343 bridge chip. Used for flashing and
    debug text (`Serial`). Optional during normal operation.
  - `USB_OTG` — the SoC's own native USB peripheral (`USBSerial`). This is
    the required data link: video frames and event markers flow over it.
- **Arduino Uno** running a separate sketch that reads:
  - A DHT11 temperature/humidity sensor, polled every 2 seconds.
  - A PIR motion sensor, via hardware interrupt with debounce (not
    polling).
  It reports these as text lines (`T:xx,H:xx` and `M:1`) over serial.
- The Arduino's serial line connects to the ESP32-S3 on a **dedicated
  second UART peripheral** — `HardwareSerial ArduinoLink(1)` on GPIO1 (RX)
  / GPIO21 (TX) — kept fully independent from the CH343's UART0, so debug
  text and sensor data never collide on the same hardware.

## Architecture

```
   PC                          ESP32-S3                    Arduino Uno
 ┌──────────┐   USB_OTG      ┌───────────┐    UART1       ┌───────────┐
 │  Python  │◄──(required)──►│  Camera   │◄──(TX + GND)───│ DHT11+PIR │
 │  script  │                │  firmware │                └───────────┘
 │          │   USB-UART     │           │
 │          │◄──(optional)───│  (debug)  │
 └──────────┘                └───────────┘
```

There is **no WiFi and no HTTP server** in the current design. An earlier
version streamed video over WiFi using Espressif's stock `CameraWebServer`
example, but it suffered from slow WiFi association and poor framerate
that did not improve at lower resolution — symptoms consistent with a
flaky hotspot link, marginal power delivery, and/or the OV3660 sensor
running hot. Rather than keep chasing that, the project moved to USB
serial as the video transport, using the ESP32-S3's native `USB_OTG` port
for much higher, more reliable throughput.

## Communication protocol

Everything between the PC and the ESP32-S3 rides on the single `USBSerial`
connection, distinguished by 4-byte magic markers:

| Marker | Direction | Meaning | Payload |
|---|---|---|---|
| `AA 55 AA 55` (`FRAME_MAGIC`) | ESP → PC | A video frame follows | 4-byte little-endian length, then that many raw JPEG bytes |
| `BB 66 BB 66` (`MOTION_MAGIC`) | ESP → PC | PIR motion detected | none |
| `CC 33 CC 33` (`ALERT_MAGIC`) | PC → ESP | Person confirmed by YOLO | none (triggers NeoPixel alert) |
| `DD 44 DD 44` (`ENV_MAGIC`) | ESP → PC | Temperature/humidity | 2 little-endian floats (temp, humidity) |

## Firmware setup (Arduino IDE)

1. Board: **ESP32S3 Dev Module**.
2. Flash Size: **16MB** (must match the actual N16R8 chip — a stale 4MB
   setting has caused build/config drift in the past).
3. PSRAM: **OPI PSRAM**.
4. USB Mode: **Hardware CDC and JTAG** (required for `USBSerial` to work). 
   Ensure USB CDC on boot is enable, if not the video stream won't be 
   transmitted.
5. Flash `MEE202_CamOverSerial.ino` (see repo for current source). 
6. The board exposes `Serial` (CH343/debug, optional to have plugged in),
   `Serial` (native USB_OTG, required — this is what the Python script
   connects to), and `ArduinoLink` (UART1 on GPIO1/21, wired to the
   Arduino).
7. Board: **Arduino Uno R3**
8. Flash `arduino_code.ino` (see repo for current source).

## Python script setup

```
pip install -r requirements.txt
```

Create a `.env` file with:
```
TELEGRAM_BOT_TOKEN=your_bot_token
TELEGRAM_BOT_ID=your_chat_id
```

Set `SERIAL_PORT` in the script to the `USB_OTG` port's own COM number
(check Device Manager — it will differ from the CH343's COM port).

Run with:
```
python person_detect.py
```

### How detection works

1. A background thread continuously drains the serial port and parses out
   frames/events — this must never block on inference, or the incoming
   byte stream backs up and playback falls behind live video.
2. On a motion marker (subject to a cooldown, to ignore a flaky PIR
   double-firing), a separate background thread runs YOLO on the next 8
   new frames, stopping early if a person is confirmed.
3. A confirmed detection saves a snapshot, sends it to Telegram in its own
   background thread, and writes `ALERT_MAGIC` back to the ESP32-S3 to
   trigger a NeoPixel alert.
4. All detection/alert work happens off the main thread specifically so
   the live "Live" window never freezes during a burst check.

## Known limitations / future work

- No true USB Video Class (UVC) support — the ESP32-S3 does not enumerate
  as a standard webcam; a custom Python reader is required. True UVC is
  possible via Espressif's `usb_device_uvc` ESP-IDF component, but would
  require migrating this firmware off the Arduino IDE.
- The OV3660 sensor has been observed running hot during extended WiFi
  operation; thermal behavior under sustained USB operation remains the same.