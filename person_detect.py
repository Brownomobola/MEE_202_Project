"""
Person detection, serial version.

WHY THIS VERSION EXISTS: the ESP32-S3 no longer runs WiFi or an HTTP
server at all -- video frames and motion events both arrive over the
native USB_OTG port as a single continuous serial byte stream. This
replaces the old /sensor polling and /capture fetching with a background
thread that continuously drains that stream, since inference (which can
take tens of milliseconds) must never be what decides how fast bytes get
read off the port -- if it were, the incoming stream would back up and
this script would fall further and further behind live video.

Usage:
    pip install -r requirements.txt
    python person_detect.py
"""

import os
import time
import threading
import queue
import struct

import cv2
import numpy as np
import requests
import serial
from ultralytics import YOLO
from dotenv import load_dotenv

load_dotenv()

# --- Configuration ---------------------------------------------------

SERIAL_PORT = "COM7"       # The USB_OTG port's own COM number (Device Manager) --
                            # different from COM5, which is the CH343 UART port.
BAUD_RATE = 115200          # Formality for native USB CDC; not a throughput limit.

FRAME_MAGIC  = bytes([0xAA, 0x55, 0xAA, 0x55])
MOTION_MAGIC = bytes([0xBB, 0x66, 0xBB, 0x66])
ALERT_MAGIC  = bytes([0xCC, 0x33, 0xCC, 0x33])
ENV_MAGIC    = bytes([0xDD, 0x44, 0xDD, 0x44])
_serial_conn = None
latest_temp = 0.0
latest_hum = 0.0

MOTION_COOLDOWN_S = 3.0     # Ignore further motion triggers for this long after
                            # handling one -- guards against the PIR sensor
                            # double-firing or acting up on its own.
ALERT_COOLDOWN_SECONDS = 5  # Separate from the above: only throttles how often
                            # we actually SEND an alert, even if bursts keep
                            # confirming a person after the motion cooldown ends.

BURST_FRAME_COUNT = 8
PERSON_CLASS_ID = 0
CONFIDENCE_THRESHOLD = 0.5

TELEGRAM_BOT_TOKEN = os.environ.get("TELEGRAM_BOT_TOKEN", None)
TELEGRAM_BOT_ID = os.environ.get("TELEGRAM_BOT_ID", None)

_last_alert_time = 0.0

# --- Shared state between the reader thread and the main thread -------

latest_frame = None
frame_lock = threading.Lock()
frame_counter = 0           # Incremented on every new frame. Lets the burst
                            # logic prove each frame it checks is genuinely
                            # new, never the same one counted twice.
motion_queue = queue.Queue()
stop_event = threading.Event()
burst_lock = threading.Lock()
burst_in_progress = False
detection_display_queue = queue.Queue()


def serial_reader(ser):
    """
    Runs continuously in its own thread. 
    Reads data in large chunks and uses C-optimized searching to prevent CPU lag.
    """
    global latest_frame, frame_counter
    buffer = bytearray()

    while not stop_event.is_set():
        # 1. Gulp: Read whatever is waiting (or wait for at least 1 byte)
        waiting = ser.in_waiting
        chunk = ser.read(waiting if waiting > 0 else 1)
        if not chunk:
            continue
            
        # 2. Bucket: Add the new chunk to our master buffer
        buffer.extend(chunk)

        # 3. Process: Loop through the buffer to extract all complete packets
        while True:
            # Handle Motion Markers First
            if MOTION_MAGIC in buffer:
                idx = buffer.find(MOTION_MAGIC)
                motion_queue.put(time.monotonic())
                # Discard everything up to and including this marker
                buffer = buffer[idx + len(MOTION_MAGIC):]
                continue # Re-evaluate the buffer

            # Handle Environment Data (DHT11)
            if ENV_MAGIC in buffer:
                idx = buffer.find(ENV_MAGIC)
                # Ensure we have the 4 magic bytes + two 4-byte floats (12 bytes total)
                if len(buffer) >= idx + 12:
                    env_bytes = buffer[idx + 4 : idx + 12]
                    global latest_temp, latest_hum
                    # Unpack two Little-Endian floats
                    latest_temp, latest_hum = struct.unpack('<ff', env_bytes)
                    
                    buffer = buffer[idx + 12:]
                    continue # Re-evaluate the buffer

            # Handle Frame Markers
            if FRAME_MAGIC in buffer:
                idx = buffer.find(FRAME_MAGIC)
                
                # Check if the 4-byte length header has arrived yet
                if len(buffer) >= idx + 4 + 4:
                    length_bytes = buffer[idx + 4 : idx + 8]
                    frame_len = int.from_bytes(length_bytes, "little")

                    
                    # Check if the full JPEG payload has arrived yet
                    total_packet_size = idx + 8 + frame_len
                    if len(buffer) >= total_packet_size:
                        # Extract the exact JPEG bytes
                        jpg_data = buffer[idx + 8 : total_packet_size]
                        
                        # Decode and update the global frame
                        frame = cv2.imdecode(np.frombuffer(jpg_data, dtype=np.uint8), cv2.IMREAD_COLOR)
                        if frame is not None:
                            with frame_lock:
                                latest_frame = frame
                                frame_counter += 1
                                
                        # Discard the processed frame from the buffer
                        buffer = buffer[total_packet_size:]
                        continue # Re-evaluate the buffer for another frame

            # Memory Protection: 
            # If there's garbage data with no markers, don't let the buffer grow infinitely.
            # We keep the last few bytes just in case a marker was cut in half by the chunk read.
            if len(buffer) > 1000000: 
                buffer = buffer[-4:]
                
            # If we don't have enough data to complete a packet, break and wait for more chunks
            break

def burst_worker(model, start_counter):
    global burst_in_progress
    found, frame = run_detection_burst(model, start_counter)
    if found:
        maybe_send_alert(frame)
        detection_display_queue.put(frame)
    with burst_lock:
        burst_in_progress = False

def get_new_frame(since_counter, timeout=2.0):
    """Blocks until a frame newer than `since_counter` has arrived, or gives
    up after `timeout` seconds (guards against a stalled camera mid-burst)."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        with frame_lock:
            if frame_counter > since_counter:
                return latest_frame.copy(), frame_counter
        time.sleep(0.01)
    return None, since_counter


def make_person_detector():
    model = YOLO("yolo11n.pt")
    dummy = np.zeros((480, 640, 3), dtype=np.uint8)
    model(dummy, verbose=False)
    return model


def detect_person(model, frame):
    results = model(frame, classes=[PERSON_CLASS_ID], verbose=False)[0]
    person_found = False

    for box in results.boxes:
        confidence = float(box.conf[0])
        if confidence >= CONFIDENCE_THRESHOLD:
            person_found = True
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, f"person {confidence:.2f}", (x1, y1 - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    return person_found, frame


def _send_telegram_worker(image_path):
    """Runs in a background thread so sending the alert never blocks the
    detection loop while waiting on the network."""
    url = f"https://api.telegram.org/bot{TELEGRAM_BOT_TOKEN}/sendPhoto"
    try:
        with open(image_path, "rb") as image_file:
            files = {"photo": image_file}
            data = {"chat_id": TELEGRAM_BOT_ID, "caption": "Alert: Person Detected!"}
            resp = requests.post(url, data, files=files, timeout=15)
            resp.raise_for_status()
        print("Telegram alert sent successfully")
    except Exception as e:
        print(f"Could not send Telegram alert because of error {str(e)}")


def maybe_send_alert(frame):
    global _last_alert_time
    now = time.monotonic()
    if now - _last_alert_time < ALERT_COOLDOWN_SECONDS:
        return
    _last_alert_time = now

    os.makedirs("detections", exist_ok=True)
    filename = f"detections/intruder_{int(time.time())}.jpg"
    cv2.imwrite(filename, frame)
    print(f"Saved the evidence to {filename}")

    threading.Thread(target=_send_telegram_worker, args=(filename,), daemon=True).start()

    if _serial_conn is not None:
        try:
            _serial_conn.write(ALERT_MAGIC)
        except serial.SerialException as e:
            print(f"Failed to send alert to ESP32: {e}")

def run_detection_burst(model, start_counter):
    """Runs detection on the next BURST_FRAME_COUNT frames arriving from this
    point on -- never whatever's currently on screen, and never the same
    frame twice, since each one is confirmed newer via frame_counter."""
    counter = start_counter
    for i in range(BURST_FRAME_COUNT):
        frame, counter = get_new_frame(counter)
        if frame is None:
            print(f"  frame {i + 1}/{BURST_FRAME_COUNT}: capture timed out")
            continue

        found, annotated = detect_person(model, frame)
        print(f"  frame {i + 1}/{BURST_FRAME_COUNT}: {'PERSON DETECTED' if found else 'no person'}")

        if found:
            return True, annotated

    return False, None


def main():
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    global _serial_conn, burst_in_progress
    _serial_conn = ser
    reader_thread = threading.Thread(target=serial_reader, args=(ser,), daemon=True)
    reader_thread.start()

    model = make_person_detector()
    print("Watching for motion over serial...")

    last_motion_handled = 0.0
    last_fps_print_time = 0.0
    
    # --- FPS Tracking Variables ---
    prev_time = time.monotonic()
    smoothed_fps = 0.0
    last_drawn_counter = 0

    try:
        while True:
            with frame_lock:
                if latest_frame is not None:
                    display_frame = latest_frame.copy()
                    current_counter = frame_counter
                else:
                    display_frame = None

            if display_frame is not None:
                if current_counter > last_drawn_counter:
                    now = time.monotonic()
                    dt = max(now - prev_time, 0.001)
                    current_fps = 1.0 / dt
                    smoothed_fps = (smoothed_fps * 0.9) + (current_fps * 0.1)
                    prev_time = now
                    last_drawn_counter = current_counter

                    # Print FPS to terminal exactly once per second
                    if now - last_fps_print_time > 1.0:
                        print(f"Stream FPS: {smoothed_fps:.1f}")
                        last_fps_print_time = now

                # Draw DHT11 data on the video frame
                if latest_temp != 0.0:
                    env_text = f"Temp: {latest_temp:.1f}C  Hum: {latest_hum:.1f}%"
                    cv2.putText(display_frame, env_text, (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                
                cv2.imshow("Live", display_frame)
                
            cv2.waitKey(1)

                        # Show any detection result that finished in the background,
            # without ever blocking the live feed to wait for one.
            try:
                detection_frame = detection_display_queue.get_nowait()
                cv2.imshow("Detection", detection_frame)
            except queue.Empty:
                pass

            try:
                motion_time = motion_queue.get_nowait()
            except queue.Empty:
                motion_time = None
                time.sleep(0.03)
                continue

            if motion_time - last_motion_handled < MOTION_COOLDOWN_S:
                print("Motion seen, but still in cooldown -- ignoring.")
                continue

            with burst_lock:
                if burst_in_progress:
                    print("Motion seen, but a burst is already running -- ignoring.")
                    continue
                burst_in_progress = True

            last_motion_handled = motion_time
            with frame_lock:
                start_counter = frame_counter

            print("Motion detected -- running burst check...")
            threading.Thread(target=burst_worker, args=(model, start_counter), daemon=True).start()

    finally:
        stop_event.set()
        reader_thread.join(timeout=2)
        ser.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Stopped.")