"""
Person detection, event-driven version.

WHY THIS VERSION EXISTS: the previous version kept a persistent MJPEG
/stream connection open, the same kind the browser uses to watch live
video. Running two of those at once (browser + this script) asks the
ESP32 to run two full streaming loops simultaneously -- serious load on
a microcontroller's CPU, camera driver, and WiFi radio all at once. That
was the actual bottleneck, not the Python-side inference speed.

This version instead:
  1. Polls /sensor every POLL_INTERVAL_S -- a tiny JSON response, no
     camera involvement at all, negligible load.
  2. Only when that poll reports motion=1, fetches ONE frame via
     /capture (a single JPEG, not a continuous stream) and runs
     detection on just that frame.

Net effect: the ESP32 only ever serves one continuous connection (the
browser's /stream), plus occasional cheap one-off requests from this
script. No more resource contention between the two.

Usage:
    pip install requests opencv-python ultralytics
    python person_detection_poll.py
"""

import time

import cv2
import numpy as np
import requests
from ultralytics import YOLO

# --- Configuration ---------------------------------------------------

ESP32_BASE = "http://192.168.137.52"          # ESP32's main IP (port 80 endpoints)
SENSOR_URL = f"{ESP32_BASE}/sensor"
CAPTURE_URL = f"{ESP32_BASE}/capture"
ALERT_URL = f"{ESP32_BASE}/alert"

POLL_INTERVAL_S = 0.3            # how often to check /sensor for motion
ALERT_COOLDOWN_SECONDS = 5       # minimum gap between alerts

BURST_FRAME_COUNT = 8            # frames to check per motion event, to catch
                                  # the person even if one or two snapshots
                                  # are blurred/mistimed
BURST_FRAME_DELAY_S = 0.15       # gap between captures in a burst -- gives the
                                  # sensor time to actually produce a *new*
                                  # frame rather than fetching the same buffered
                                  # one twice in a row

PERSON_CLASS_ID = 0
CONFIDENCE_THRESHOLD = 0.5

_last_alert_time = 0.0


def make_person_detector():
    model = YOLO("yolo11n.pt")
    dummy = np.zeros((480, 640, 3), dtype=np.uint8)
    model(dummy, verbose=False)  # warm up, so first real timing isn't skewed
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


def maybe_send_alert():
    global _last_alert_time
    now = time.monotonic()
    if now - _last_alert_time < ALERT_COOLDOWN_SECONDS:
        return
    _last_alert_time = now
    try:
        requests.get(ALERT_URL, timeout=3)
        print("Alert sent to ESP32.")
    except requests.RequestException as e:
        print(f"Failed to send alert: {e}")


def fetch_snapshot():
    """One-shot JPEG fetch via /capture -- not a stream, closes right after."""
    try:
        resp = requests.get(CAPTURE_URL, timeout=5)
        resp.raise_for_status()
        img_array = np.frombuffer(resp.content, dtype=np.uint8)
        return cv2.imdecode(img_array, cv2.IMREAD_COLOR)
    except requests.RequestException as e:
        print(f"Snapshot fetch failed: {e}")
        return None


def poll_motion():
    """Lightweight check -- just a small JSON response, no camera load."""
    try:
        resp = requests.get(SENSOR_URL, timeout=3)
        resp.raise_for_status()
        return resp.json().get("motion", 0) == 1
    except requests.RequestException as e:
        print(f"Poll failed: {e}")
        return False


def run_detection_burst(model):
    """
    Fetches up to BURST_FRAME_COUNT snapshots in quick succession and runs
    detection on each, stopping as soon as one confirms a person (no point
    burning through the rest of the burst once we already know). If none
    of the frames show a person, we've genuinely checked BURST_FRAME_COUNT
    different moments, not just one unlucky snapshot.
    """
    for i in range(BURST_FRAME_COUNT):
        frame = fetch_snapshot()
        if frame is None:
            continue

        found, annotated = detect_person(model, frame)
        print(f"  frame {i + 1}/{BURST_FRAME_COUNT}: {'PERSON DETECTED' if found else 'no person'}")

        if found:
            cv2.imshow("Detection", annotated)
            cv2.waitKey(1)
            return True

        if i < BURST_FRAME_COUNT - 1:
            time.sleep(BURST_FRAME_DELAY_S)

    return False


def main():
    model = make_person_detector()
    print("Watching for motion (polling, not streaming)...")

    while True:
        if poll_motion():
            print("Motion detected -- running burst check...")
            if run_detection_burst(model):
                maybe_send_alert()

        time.sleep(POLL_INTERVAL_S)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Stopped.")
        cv2.destroyAllWindows()