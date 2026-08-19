/*
  Pan-Tilt Servo Controller
  --------------------------
  Receives lines like "90,120\n" over serial (pan,tilt angles in degrees)
  and moves two servos to match, with light smoothing on the Arduino side
  as a second layer of jitter protection (Python already smooths too).

  Wiring:
    Pan servo signal  -> Pin 9
    Tilt servo signal -> Pin 10
    Servo power (V+)  -> external 5V supply (NOT the Arduino 5V pin if
                         you have more than one servo or a heavier cam)
    Servo GND         -> common ground with Arduino GND
*/

#include <Servo.h>

Servo panServo;
Servo tiltServo;

const int PAN_PIN = 9;
const int TILT_PIN = 10;

float currentPan = 90;
float currentTilt = 90;
float targetPan = 90;
float targetTilt = 90;

const float SERVO_SMOOTHING = 0.6;  // 0-1, lower = smoother

String inputBuffer = "";

void setup() {
  Serial.begin(115200);
  panServo.attach(PAN_PIN);
  tiltServo.attach(TILT_PIN);

  panServo.write((int)currentPan);
  tiltServo.write((int)currentTilt);
}

void loop() {
  // Read incoming serial data line by line
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n') {
      parseCommand(inputBuffer);
      inputBuffer = "";
    } else {
      inputBuffer += c;
    }
  }

  // Smooth movement toward target each loop
  //currentPan += (targetPan - currentPan) * SERVO_SMOOTHING;
  //currentTilt += (targetTilt - currentTilt) * SERVO_SMOOTHING;

  //panServo.write((int)currentPan);
  //tiltServo.write((int)currentTilt);

  //delay(15);  // ~60Hz update loop
}

void parseCommand(String line) {
  int commaIndex = line.indexOf(',');
  if (commaIndex == -1) return;  // malformed line, ignore

  String panStr = line.substring(0, commaIndex);
  String tiltStr = line.substring(commaIndex + 1);

  int pan = panStr.toInt();
  int tilt = tiltStr.toInt();

  // Clamp to safe servo range
  pan = constrain(pan, 0, 180);
  tilt = constrain(tilt, 0, 180);

  //targetPan = pan;
  //targetTilt = tilt;
  panServo.write(pan);
  tiltServo.write(tilt);
}
