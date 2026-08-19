"""
Hand-tracking pan/tilt controller for CONTINUOUS ROTATION servos
--------------------------------------------------------------------
These servos have no position sense - sending a value away from 90 spins
them at a speed/direction, not to an angle. So instead of computing a
target position, this script continuously checks the hand's offset from
frame center (the "error") and sends a speed command proportional to that
error, every frame. When the hand is centered, it sends "stop" (90,90).
The camera itself is the feedback loop, replacing a position sensor.

Install dependencies:
    pip install opencv-python mediapipe pyserial

You also need the hand landmark model file. Download it once:
    curl -L -o hand_landmarker.task https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/1/hand_landmarker.task
Place hand_landmarker.task in the same folder as this script.

Arduino side: pairs with pan_tilt_simple.ino (or any sketch that just
does panServo.write(value) / tiltServo.write(value) directly from the
received "pan,tilt\n" line, no smoothing).
"""

import cv2
import mediapipe as mp
from mediapipe.tasks import python as mp_python
from mediapipe.tasks.python import vision as mp_vision
import serial
import time
import os

# ---------------- CONFIG ----------------
SERIAL_PORT = "COM8"       # Windows e.g. "COM5"; Linux/Mac e.g. "/dev/ttyACM0" or "/dev/ttyUSB0"
BAUD_RATE = 115200
CAM_INDEX = 0

MODEL_PATH = "hand_landmarker.task"

STOP_VALUE = 90  # value that stops a continuous rotation servo

# Max speed offset from STOP_VALUE (e.g. 90 +/- 40 = range 50-130).
# Keep well under 0-180 so you're not accidentally pushing extreme speed.
MAX_SPEED_OFFSET = 40

# How strongly error translates to speed. Larger error (hand further from
# center) = faster spin, up to MAX_SPEED_OFFSET.
PAN_GAIN = -40
TILT_GAIN = 40

# Dead zone: if the hand is within this fraction of center, stop the servo.
# This is your "close enough" tolerance - too small and it'll hunt/oscillate
# forever chasing tiny noise; too large and tracking feels imprecise.
DEAD_ZONE = 0.05  # fraction of frame width/height

# How often to send commands (seconds). Sending every single frame is fine
# for USB serial, but you can throttle this if you want coarser pulses.
SEND_INTERVAL = 0.05
# -----------------------------------------


def clamp(val, lo, hi):
    return max(lo, min(hi, val))


def main():
    if not os.path.exists(MODEL_PATH):
        print(f"Model file not found at '{MODEL_PATH}'.")
        print("Download it with:")
        print("  curl -L -o hand_landmarker.task https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/1/hand_landmarker.task")
        return

    try:
        arduino = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        time.sleep(2)
        print(f"Connected to Arduino on {SERIAL_PORT}")
    except Exception as e:
        print(f"Could not open serial port {SERIAL_PORT}: {e}")
        print("Continuing without Arduino connection (preview only).")
        arduino = None

    base_options = mp_python.BaseOptions(model_asset_path=MODEL_PATH)
    options = mp_vision.HandLandmarkerOptions(
        base_options=base_options,
        running_mode=mp_vision.RunningMode.VIDEO,
        num_hands=1,
        min_hand_detection_confidence=0.6,
        min_hand_presence_confidence=0.6,
        min_tracking_confidence=0.6,
    )
    landmarker = mp_vision.HandLandmarker.create_from_options(options)

    cap = cv2.VideoCapture(CAM_INDEX)
    if not cap.isOpened():
        print("Could not open webcam.")
        return

    start_time = time.time()
    last_send_time = 0

    def send(pan_val, tilt_val):
        if arduino is not None:
            try:
                arduino.write(f"{int(pan_val)},{int(tilt_val)}\n".encode())
            except Exception as e:
                print(f"Serial write failed: {e}")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        frame = cv2.flip(frame, 1)
        h, w, _ = frame.shape
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)

        frame_timestamp_ms = int((time.time() - start_time) * 1000)
        result = landmarker.detect_for_video(mp_image, frame_timestamp_ms)

        pan_speed = STOP_VALUE
        tilt_speed = STOP_VALUE

        if result.hand_landmarks:
            hand_landmarks = result.hand_landmarks[0]

            for lm in hand_landmarks:
                px, py = int(lm.x * w), int(lm.y * h)
                cv2.circle(frame, (px, py), 3, (0, 200, 255), -1)

            wrist = hand_landmarks[0]
            middle_mcp = hand_landmarks[9]
            cx = (wrist.x + middle_mcp.x) / 2
            cy = (wrist.y + middle_mcp.y) / 2

            # Error: how far off-center the hand is (-0.5 to 0.5)
            err_x = cx - 0.5
            err_y = cy - 0.5

            cv2.circle(frame, (int(cx * w), int(cy * h)), 8, (0, 255, 0), -1)

            # Outside dead zone -> spin toward the hand, speed scaled by error
            if abs(err_x) > DEAD_ZONE:
                offset = clamp(err_x * PAN_GAIN, -MAX_SPEED_OFFSET, MAX_SPEED_OFFSET)
                pan_speed = STOP_VALUE + offset
            if abs(err_y) > DEAD_ZONE:
                offset = clamp(err_y * TILT_GAIN, -MAX_SPEED_OFFSET, MAX_SPEED_OFFSET)
                tilt_speed = STOP_VALUE + offset
        # If no hand detected at all, pan_speed/tilt_speed stay at STOP_VALUE
        # (servo stops rather than continuing to spin blindly)

        now = time.time()
        if now - last_send_time >= SEND_INTERVAL:
            send(pan_speed, tilt_speed)
            last_send_time = now

        cv2.putText(frame, f"Pan speed: {int(pan_speed)}  Tilt speed: {int(tilt_speed)}  (q = quit)",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.imshow("Hand Tracking Pan-Tilt (Continuous Rotation)", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Always stop the servos before exiting
    send(STOP_VALUE, STOP_VALUE)

    cap.release()
    cv2.destroyAllWindows()
    landmarker.close()
    if arduino is not None:
        arduino.close()


if __name__ == "__main__":
    main()
