/*
  pca_serial (v4.4)
  Type a channel and an angle in the Serial Monitor (e.g. "2 90") and that servo moves there
  in small steps, starting from that joint's last commanded angle.
  Each joint has its own step size and pause: base, shoulder and elbow 1 deg every 10 ms;
  wrist and gripper 2 deg every 20 ms (same speed, 100 deg/s), because the SG90 dead band
  makes 1-deg steps jerky. After the steps, one last pulse lands on the exact angle.
  Channels: 0 base, 1 shoulder, 2 elbow, 3 wrist, 4 gripper.
  Each joint has its own accepted range (ANGLE_MIN, ANGLE_MAX, measured on the arm):
  base 40-140 (cables), shoulder and elbow 10-170 (full range), wrist 40-110 (it buzzes outside),
  gripper 30-150. Anything else prints "ERROR!" and nothing moves.
  Monitor setting: "New Line", 9600 baud.

  At power-up every joint goes to its NEUTRAL angle, one joint at a time. This is the only jump:
  place the arm by hand near the neutral pose before switching the supply on.

  Calibration, command c -> real joint angle in degrees:
    base      psi = 0.754 * (c - 90)     counterclockwise seen from above when c grows
    shoulder  theta = 0.865 * c - 30.1   link 1 from the table
    elbow     q2 = 0.836 * c - 143.0     forearm from link 1 (0 = in line), forearm in front of vertical
    wrist     q3 = -0.910 * c + 49.4     gripper from forearm (0 = in line)
  Backlash: about 1 deg, 2-3 deg at the base, about 7 deg when the load flips past vertical.

  Wiring: Arduino 5V -> PCA VCC, GND -> GND, A4 -> SDA, A5 -> SCL (I2C).
  Servo power comes from the 5 V supply on the PCA screw terminal (V+), never from the Arduino.
*/

#include <Adafruit_PWMServoDriver.h>  // PCA9685 library (it includes Wire, the I2C library)

// PCA channel of each servo: the name says which joint, the value where it is plugged in.
const int BASE = 0;
const int SHOULDER = 1;
const int ELBOW = 2;
const int WRIST = 3;
const int GRIPPER = 4;
const int NUM_JOINTS = 5;

// Pulse length at 0 and 180 deg: the same straight line the Servo library uses,
// so 90 here gives the same pulse (1472 us) as servo_serial.
const int PULSE_MIN_US = 544;
const int PULSE_MAX_US = 2400;

// Step size (deg) and pause per step (ms), one value per channel (index = channel).
// Speed = step / pause: 1 deg / 10 ms = 2 deg / 20 ms = 100 deg/s.
const int STEP_DEG[NUM_JOINTS]      = { 1,  1,  1,  2,  2};
const int STEP_DELAY_MS[NUM_JOINTS] = {10, 10, 10, 20, 20};

// Accepted command range of each joint (deg), one value per channel (index = channel).
// Measured with the other joints in the neutral pose.
const int ANGLE_MIN[NUM_JOINTS] = { 40,  10,  10,  40,  30};
const int ANGLE_MAX[NUM_JOINTS] = {140, 170, 170, 110, 150};

// Neutral pose, one angle per channel (index = channel): the arm as close to vertical as the
// elbow allows. Shoulder at 130: link 1 about 82 deg from the table; elbow at 165: forearm about
// 5 deg from in line with link 1; wrist at 54: gripper in line with the forearm. All inside
// ANGLE_MIN/MAX.
const int NEUTRAL[NUM_JOINTS] = {90, 130, 165, 54, 90};

Adafruit_PWMServoDriver pca;  // one object for the whole board (default I2C address 0x40)

// Last commanded angle of each joint (index = channel). Global, so it survives between loop() calls.
int last_position[NUM_JOINTS];

void setup() {
  Serial.begin(9600);
  pca.begin();
  pca.setPWMFreq(50);  // 50 pulses per second = one pulse every 20 ms, what hobby servos expect

  // Send every joint to its neutral angle, one at a time with a pause, so they don't all jump
  // together (smaller current peak), and start each joint's memory from there.
  for (int joint = BASE; joint <= GRIPPER; joint++) {
    int pulse_us = map(NEUTRAL[joint], 0, 180, PULSE_MIN_US, PULSE_MAX_US);
    pca.writeMicroseconds(joint, pulse_us);
    last_position[joint] = NEUTRAL[joint];
    delay(500);
  }
  Serial.println("Ready: all joints at neutral.");
}

void loop() {
  if (Serial.available() > 0) {
    char next = Serial.peek();  // look at the first character without removing it

    if (next == '\n') {
      Serial.read();  // drop the line ending, so parseInt never sees it alone
    }

    else {
      // First number = channel, second = angle. parseInt stops at the first non-digit
      // (the space, then the line ending) and returns 0 after a 1 s timeout if no digits came.
      int joint = Serial.parseInt();
      int angle = Serial.parseInt();

      // Valid channel first, then the angle inside that joint's range: && stops at the first
      // false, so ANGLE_MIN[joint] is read only with a valid index. A timeout gives joint = 0
      // (a valid channel) and angle = 0, below every ANGLE_MIN: nothing moves.
      if (joint >= BASE && joint <= GRIPPER && angle >= ANGLE_MIN[joint] && angle <= ANGLE_MAX[joint]) {

        // Steps from this joint's last angle toward the new one, with this joint's step and pause.
        // The loops stop before the target (< and >): with 2-deg steps the target may fall
        // between two steps, so the exact angle is sent once at the end.
        if (angle >= last_position[joint]) {
          for (int step_angle = last_position[joint]; step_angle < angle; step_angle += STEP_DEG[joint]) {
            // Angle -> pulse length: 0..180 deg mapped onto 544..2400 us (90 -> 1472).
            int step_pulse_us = map(step_angle, 0, 180, PULSE_MIN_US, PULSE_MAX_US);
            pca.writeMicroseconds(joint, step_pulse_us);
            delay(STEP_DELAY_MS[joint]);
          }
        }

        else {
          for (int step_angle = last_position[joint]; step_angle > angle; step_angle -= STEP_DEG[joint]) {
            int step_pulse_us = map(step_angle, 0, 180, PULSE_MIN_US, PULSE_MAX_US);
            pca.writeMicroseconds(joint, step_pulse_us);
            delay(STEP_DELAY_MS[joint]);
          }
        }

        // Last pulse on the exact target, then remember it: the next move starts from here.
        int target_pulse_us = map(angle, 0, 180, PULSE_MIN_US, PULSE_MAX_US);
        pca.writeMicroseconds(joint, target_pulse_us);
        last_position[joint] = angle;

        // One print per piece, println on the last one: "Joint: 2  Angle: 90"
        Serial.print("Joint: ");
        Serial.print(joint);
        Serial.print("  Angle: ");
        Serial.println(angle);
      }

      else {
        Serial.println("ERROR!");
      }
    }
  }
}