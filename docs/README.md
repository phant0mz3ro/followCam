# Hand-Tracking Pan-Tilt Camera Rig

A camera mounted on a pan-tilt bracket that visually tracks a hand in frame, driven by a closed-loop PI controller — using the camera itself as the position sensor, since the servos used have no feedback of their own.

## Overview

- **Input:** USB webcam, hand detected via MediaPipe's HandLandmarker
- **Compute:** Python (PC-side) — detection, control loop, serial communication
- **Actuation:** Arduino Uno/Nano driving two continuous-rotation servos (pan + tilt)
- **Control:** PI controller per axis, gains derived from an empirically measured plant model, with anti-windup

---

## System Architecture

```
┌─────────────┐   frames    ┌──────────────────────────┐   "pan,tilt\n"   ┌──────────┐   PWM   ┌─────────────┐
│  USB Webcam │ ──────────> │   Python (PC)             │ ───────────────> │ Arduino  │ ──────> │ Pan/Tilt    │
└─────────────┘             │  - MediaPipe hand detect  │    over serial   │ (Uno)    │         │ Servos      │
                             │  - error computation       │                  └──────────┘         └─────────────┘
                             │  - PI controller x2         │
                             │  - anti-windup               │
                             └──────────────────────────┘
                                       ▲                                                                │
                                       └──────────────── visual feedback (next frame) ───────────────────┘
```

The camera closes the loop. There's no encoder or potentiometer on the servos — each frame, Python re-measures how far off-center the hand is and corrects again. This is the plant's only feedback path.

---

## Hardware

| Component | Notes |
|---|---|
| Arduino Uno/Nano | Handles servo PWM output only — no vision processing on-device |
| 2x MG996R (continuous rotation variant) | Confirmed via open-loop testing, not just the datasheet label — see [Plant Identification](#plant-identification) |
| Pan-tilt bracket | Standard 2-axis hobby bracket |
| USB webcam | FOV measured empirically (see [Camera Calibration](#camera-calibration)) |
| 5V power for servos | Powered from Arduino 5V rail during development; external supply recommended for production use to avoid brownout resets |

### A note on servo type

The servos in this build turned out to be **continuous-rotation**, not standard positional servos, despite the MG996R label typically referring to a positional part. This was discovered empirically: commanding a "position" caused the servo to spin continuously and never settle. Confirmed by testing — writing a value away from 90 spins the servo at a speed/direction proportional to the offset, rather than moving it to an angle. `90` = stop.

This changes the whole control problem: instead of "move to angle θ," the system controls **velocity**, and angular position only exists implicitly, as the time-integral of applied velocity. This is why closed-loop vision feedback is necessary — no other way to know where the camera is pointed.

---

## Wiring

```
Pan servo   signal -> Arduino pin 9
Tilt servo  signal -> Arduino pin 10
Servo V+    -> 5V (Arduino, or external supply)
Servo GND   -> Arduino GND (shared ground required if using external supply)
```

**Shared ground is critical** if using an external power supply for the servos — without it, the PWM signal has no consistent voltage reference, and the servo will move erratically/randomly (this exact symptom was diagnosed during the build: unprompted, noisy movement traced back to a missing ground connection between the external supply and the Arduino).

---

## Software Setup

1. **Arduino:** upload `pan_tilt_simple.ino` — a minimal receive-only sketch that parses `"pan,tilt\n"` over serial (115200 baud) and writes both values directly to the servos. All control logic (including any smoothing) intentionally lives in Python, not on the Arduino, since the plant is velocity-controlled and the control loop needs the camera's feedback every cycle anyway.

2. **Python dependencies:**
   ```bash
   pip install opencv-python mediapipe pyserial
   ```

3. **Hand landmark model** (MediaPipe Tasks API):
   ```bash
   curl -L -o hand_landmarker.task https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/1/hand_landmarker.task
   ```

4. Set `SERIAL_PORT` in the Python script to match the Arduino's port.

5. Run:
   ```bash
   python hand_tracking_pi.py
   ```

---

## Control Theory

### Plant Identification

Since the servo is velocity-controlled, the relationship between the serial command and the camera's angular position is:

```
dθ/dt = Kv · (u − 90)
```

In the Laplace domain:

```
G(s) = Θ(s) / U(s) = Kv / s
```

This is a **Type-1 (single integrator) plant** — command in, velocity out, angle is the integral of that.

**Measuring Kv:** applied a fixed offset (u = 110, i.e. +20 from stop) for a fixed 1-second pulse, then measured the actual angular rotation with a reference pointer against a marked scale.

| Axis | Offset | Duration | Measured rotation | Kv |
|---|---|---|---|---|
| Pan | +20 | 1.0 s | ~20° | 1.0 deg/sec per unit offset |
| Tilt | +20 | 1.0 s | ~20° | 1.0 deg/sec per unit offset |

### Camera Calibration (Field of View)

To convert the hand's normalized frame position (0–1) into a real angular error, the camera's field of view was measured directly:

```
FOV = 2 · arctan((measured width / 2) / distance)
```

| Axis | Width/Height | Distance | FOV |
|---|---|---|---|
| Horizontal | 26 in | 34 in | 41.8° |
| Vertical | 10 in | 19 in | 29.5° |

```python
angular_error_x = normalized_error_x * 41.8   # degrees
angular_error_y = normalized_error_y * 29.5   # degrees
```

### Controller Design

With `C(s) = Kp + Ki/s` and `G(s) = Kv/s`, the closed-loop transfer function is:

```
T(s) = Kv(Kp·s + Ki) / (s² + Kv·Kp·s + Kv·Ki)
```

Matching to the standard second-order form `s² + 2ζωn·s + ωn²`:

```
Kp = 2ζωn / Kv
Ki = ωn² / Kv
```

**Design targets:**
- ζ = 0.8 (well-damped, minimal overshoot — avoids the camera swinging past the hand)
- ωn = 1.5 rad/s (tuned down from an initial 2.0 rad/s after the first pass felt too aggressive/twitchy in practice — see [Tuning Log](#tuning-log))

```
Kp = 2 × 0.8 × 1.5 / 1.0 = 2.4
Ki = 1.5² / 1.0          = 2.25
```

Applied identically to both axes (pan and tilt), since measured Kv was equal for both.

### Why PI, not just P

The plant is already a Type-1 system, so proportional control alone gives zero theoretical steady-state error for a constant target. In practice, however, the servo exhibits **deadband/stiction** near the stop point — a small range of command values close to 90 where friction prevents the motor from actually turning. Pure P control can leave the system stuck just inside this deadband, generating too small a command to overcome it. The integral term accumulates over time and pushes the effective command through the deadband, which P alone could not do.

### Anti-Windup

Since the controller's output is clamped (`MAX_SPEED_OFFSET = 40`, keeping commands within 50–130), an unclamped integral term can continue accumulating while the output is already saturated — causing overshoot once the error direction reverses. Implemented as clamped conditional integration: the integral only continues accumulating if doing so wouldn't push the output further into saturation in the same direction as the current error.

The integral is also explicitly reset to zero whenever the hand is centered (inside the dead zone) or lost from frame entirely, preventing stale accumulated error from causing an unexpected lurch when tracking resumes.

### Dead Zone

A ±2° angular dead zone prevents the controller from continuously "hunting" — chasing tiny frame-to-frame landmark jitter that isn't real hand movement.

---

## Tuning Log

| Attempt | ωn (rad/s) | Kp | Ki | Result |
|---|---|---|---|---|
| 1 | 2.0 | 3.2 | 4.0 | Too responsive/twitchy — reacted aggressively to small movements |
| 2 (current) | 1.5 | 2.4 | 2.25 | Noticeably smoother, still tracks responsively |

**Diagnostic principle used:** ζ (damping ratio) governs *overshoot/oscillation shape*; ωn (natural frequency) governs *speed of reaction*. Since the reported issue was general over-eagerness rather than overshoot-and-correct oscillation, ωn was the correct parameter to reduce — this scales Kp and Ki down together while preserving the damping ratio (and therefore the same relative response shape), just slower.

---

## Known Issues / Build Notes

- **Mirrored frame sign inversion:** `cv2.flip()` (used to make the on-screen preview feel natural) inverts the relationship between apparent horizontal hand position and the servo's real-world pan direction. Fixed by negating the horizontal error term before it enters the controller.
- **Vertical FOV measurement correction:** an initial vertical FOV measurement used an inconsistent distance from the horizontal measurement, producing an implausible aspect ratio; remeasured at a consistent, corrected distance.
- **Brownout-induced "phantom" movement:** early testing showed servos moving without any Python process running. Traced to the Arduino resetting under power sag when servos shared the Arduino's 5V rail (`setup()` explicitly re-centers servos to 90° on every reset, which looked like unprompted movement). Resolved by using external power *with* a shared ground back to the Arduino — the shared ground was the actual missing piece, not the external supply alone.

---

## Files

| File | Purpose |
|---|---|
| `hand_tracking_pi.py` | Main control loop — hand detection, error computation, PI control, serial output |
| `pan_tilt_simple.ino` | Arduino sketch — receives `"pan,tilt\n"`, writes directly to both servos |
| `hand_landmarker.task` | MediaPipe hand landmark model (downloaded separately, not committed) |

---

## Possible Future Work

- Log Kv at multiple offset levels to check plant linearity near the deadband, rather than assuming a single-point measurement holds across the whole operating range
- Add a physical position sensor (potentiometer tap or magnetic encoder) to enable true position control and eliminate dependence on vision-frame-rate-limited feedback
- Extend to multi-hand / gesture-based mode switching
- Feedforward term for known camera pan velocity limits to reduce reliance on integral action alone through the deadband
