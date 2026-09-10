# Hand-tracking Cam
A webcam on a pan-tilt bracket that communicates with a Python script to visually track a hand in frame, utilizing a PI controller to send signals to the bracket's servos."

## Why this project

I've been fond of optics and computational imaging of recent. This includes working with cameras. I thought it would be cool to have a camera setup that automatically follows you and adjusts you in the frame.
It's a unique project that showcases tracking without the use of a traditional sensor.
It boasts an interconnection of different parts of engineering such as computer vision, control theory, robotics and programming.

## System architecture

The webcam sends video frames to the computer via USB. A Python script is programmed to receive each frame and performs hand detection. This is done using Python's Mediapipe module. It calculates the center of the hand on the frame using the wrist and lower middle finger landmarks.
Once it knows the position of the hand on the frame, it checks how far it is from the center of the frame.

Depending on it's offset from the center of the frame (above, below, left or right), it sends a message to arduino through serial connection on how to move the Pan&Tilt servos to ensure the hand stays at the center of the frame.

To improve the responsiveness, fluidity and reduce steady state error, a PI controller is incorporated in the algorithm. With proper tuning (theoritically and experimentally), the system can deliver fluid real-time response to hand motion. 

If you're wondering, the feedback comes back from the camera feed. So, in a way both the input and controlled output is measured from the same entity. Cool!

## Why do we measure the FOV?
Unlike Position servos, continous servos can't track their postion, so how do we determine its displacement.
Since there is a camera mounted directly on the servo brackets, movement of the servos will cause a displacement of an object viewed from video feed.
Therefore, tracking the displacement of an object in the video frame could give you an accurate estimate of the displacement of the servos.
However since the displacement of an object in the frame is rectilinear, we convert it to angular displacement by using the FOV of the camera lens which was determined experimentally.



## Control theory

The first step is understanding the system.
The servos are continuous-rotation servos.

On each frame, the angular offset from the center is measured and normalized making use of the FOV for both axis. That error is passed through the loop.
 
#### Plant model: 

$\dot{\theta} = Kv * (u - STOP\_VALUE)$ \
     $G(s) = Kv / s$ \
    where u is the serial command \
          $\theta$ is the pan/tilt angle in degrees \
    Using a PI controller \
    $C(s) = Kp + Ki/s$\
    Closed Loop: \
    $T(s) = Kv(Kp\cdot s + Ki) / (s^2 + Kv\cdot Kp\cdot s + Kv\cdot Ki)$

Matching the denominator to the standard 2nd-order form \
$s^2 + 2\zeta \omega n + \omega n^2$ \
$Ki = \omega n^2/Kv$ \
$Kp = 2 \zeta \omega n /Kv$ 

Kv was measured experimentally to give a unit gain.
Using damping ratio, $\zeta=0.8$ and natural frequency, $\omega=1.2$, the controller gains are, \
$Kp = 1.92, Ki = 1.44$ 
