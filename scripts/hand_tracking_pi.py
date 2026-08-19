"""
Hand-tracking pan/tilt controller - PI control version
-----------------------------------------------------------
Continuous-rotation servos have no position feedback, so the camera itself
closes the loop: each frame we measure the hand's angular offset from
center (converted from normalized frame position using measured FOV),
and run that error through a PI controller to compute a speed command.

Plant model (derived from open-loop testing):
    dTheta/dt = Kv * (u - STOP_VALUE)      i.e. G(s) = Kv / s

Measured: Kv ~= 1 deg/sec per unit offset (from 20-offset, 1s pulse -> ~20 deg)

Controller: C(s) = Kp + Ki/s
Designed for closed-loop damping ratio ~0.8, natural frequency ~2 rad/s:
    Kp = 2*zeta*wn / Kv = 3.2
    Ki = wn^2 / Kv       = 4.0

Install dependencies:
    pip install opencv-python mediapipe pyserial

You also need the hand landmark model file. Download it once:
    curl -L -o hand_landmarker.task https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/1/hand_landmarker.task
Place hand_landmarker.task in the same folder as this script.

Arduino side: pairs with pan_tilt_simple.ino (writes values directly,
no on-Arduino smoothing - all control logic lives here in Python).
"""

import cv2
import mediapipe as mp
from mediapipe.tasks import python as mp_python
from mediapipe.tasks.python import vision as mp_vision
import serial
import time
import os

# ---------------- CONFIG ----------------
SERIAL_PORT = "COM8"
BAUD_RATE = 115200
CAM_INDEX = 0

MODEL_PATH = "hand_landmarker.task"

STOP_VALUE = 90
MAX_SPEED_OFFSET = 40  # servo command range stays within 90 +/- 40

# Measured field of view (degrees), from your W/D and H/D measurements
FOV_HORIZONTAL = 41.8
FOV_VERTICAL = 29.5

# PI gains, derived from Kv ~= 1 deg/sec per unit offset, zeta=0.8, wn=2 rad/s
KP_PAN = 1.92#3.2
KI_PAN = 1.44#4.0
KP_TILT = 1.92#3.2
KI_TILT = 1.44#4.0

# Dead zone in degrees now (converted from normalized frame error via FOV).
# Below this angular error, treat as "centered enough" and stop.
DEAD_ZONE_DEG = 2.0

# Anti-windup: clamp the integral term's *contribution* to prevent it from
# accumulating unboundedly while the output is saturated at MAX_SPEED_OFFSET.
INTEGRAL_LIMIT = MAX_SPEED_OFFSET / max(KI_PAN, KI_TILT)  # conservative shared limit

SEND_INTERVAL = 0.05  # seconds between serial writes
# -----------------------------------------


def clamp(val, lo, hi):
    return max(lo, min(hi, val))


class PIController:
    """Simple PI controller with integral anti-windup via clamping."""

    def __init__(self, kp, ki, output_limit, integral_limit):
        self.kp = kp
        self.ki = ki
        self.output_limit = output_limit
        self.integral_limit = integral_limit
        self.integral = 0.0

    def update(self, error_deg, dt):
        # Proportional term
        p_term = self.kp * error_deg

        # Integral term - only accumulate if not already saturated in the
        # same direction as this error (basic anti-windup: stop growing the
        # integral once output is clamped, so it doesn't overshoot on release)
        prospective_integral = self.integral + error_deg * dt
        prospective_integral = clamp(prospective_integral,
                                      -self.integral_limit, self.integral_limit)

        unclamped_output = p_term + self.ki * prospective_integral

        if abs(unclamped_output) < self.output_limit or \
           (unclamped_output * error_deg < 0):
            # Not saturated, or error is pulling back the other way -> integrate normally
            self.integral = prospective_integral

        output = self.kp * error_deg + self.ki * self.integral
        output = clamp(output, -self.output_limit, self.output_limit)
        return output

    def reset(self):
        self.integral = 0.0


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

    pan_pi = PIController(KP_PAN, KI_PAN, MAX_SPEED_OFFSET, INTEGRAL_LIMIT)
    tilt_pi = PIController(KP_TILT, KI_TILT, MAX_SPEED_OFFSET, INTEGRAL_LIMIT)

    def send(pan_val, tilt_val):
        if arduino is not None:
            try:
                arduino.write(f"{int(pan_val)},{int(tilt_val)}\n".encode())
            except Exception as e:
                print(f"Serial write failed: {e}")

    start_time = time.time()
    last_send_time = 0
    last_loop_time = time.time()

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

        now = time.time()
        dt = now - last_loop_time
        last_loop_time = now

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

            cv2.circle(frame, (int(cx * w), int(cy * h)), 8, (0, 255, 0), -1)

            # Normalized error -> real angular error using measured FOV
            err_x_norm = cx - 0.5
            err_y_norm = cy - 0.5
            err_x_deg = -err_x_norm * FOV_HORIZONTAL
            err_y_deg = err_y_norm * FOV_VERTICAL

            if abs(err_x_deg) > DEAD_ZONE_DEG:
                offset = pan_pi.update(err_x_deg, dt)
                pan_speed = STOP_VALUE + offset
            else:
                pan_pi.reset()  # centered - clear integral so it doesn't drift next time

            if abs(err_y_deg) > DEAD_ZONE_DEG:
                offset = tilt_pi.update(err_y_deg, dt)
                tilt_speed = STOP_VALUE + offset
            else:
                tilt_pi.reset()
        else:
            # Hand lost - stop and clear integrators so we don't lurch when
            # tracking resumes
            pan_pi.reset()
            tilt_pi.reset()

        if now - last_send_time >= SEND_INTERVAL:
            send(pan_speed, tilt_speed)
            last_send_time = now

        cv2.putText(frame, f"Pan: {int(pan_speed)}  Tilt: {int(tilt_speed)}  (q=quit)",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.imshow("Hand Tracking Pan-Tilt (PI Control)", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    send(STOP_VALUE - (), STOP_VALUE)

    cap.release()
    cv2.destroyAllWindows()
    landmarker.close()
    if arduino is not None:
        arduino.close()


if __name__ == "__main__":
    main()
