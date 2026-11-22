"""Drive the whole arm with an Xbox-style gamepad, with excavator-style controls.

  Left stick  left/right -> base (swing)       Left stick  up/down    -> elbow (arm: up = out)
  Right stick up/down    -> shoulder (boom)    Right stick left/right -> wrist (bucket)
  Hold X -> gripper closes                     Hold B -> gripper opens
  Start or Ctrl+C -> stop

Sticks and buttons set the speed of a joint, not its position: release them and the joint stays put.
Usage: python gamepad_arm.py [--port COM3]
"""
import argparse
import time

import pygame

from marco import CHANNELS, DEFAULT_PORT, LIMITS, NEUTRAL, move_joint, open_arm

DEAD_ZONE = 0.15   # stick values between -0.15 and +0.15 count as 0
LOOP_S = 0.01      # while a joint moves, the firmware ramp already sets the pace

# pygame numbers for an Xbox controller on Windows.
AXIS_LEFT_X, AXIS_LEFT_Y, AXIS_RIGHT_X, AXIS_RIGHT_Y = 0, 1, 2, 3
BUTTON_B, BUTTON_X, BUTTON_START = 1, 2, 7

# Stick-driven joints: which axis, direction (flip the sign if a joint moves the wrong way),
# degrees per loop with the stick fully pushed.
STICKS = {
    "base":     {"axis": AXIS_LEFT_X,  "sign": -1, "step": 3.0},
    "shoulder": {"axis": AXIS_RIGHT_Y, "sign": -1, "step": 2.0},
    "elbow":    {"axis": AXIS_LEFT_Y,  "sign": -1, "step": 2.0},
    "wrist":    {"axis": AXIS_RIGHT_X, "sign": +1, "step": 3.0},
}
GRIPPER_STEP = 3.0
GRIPPER_CLOSE_SIGN = -1   # X lowers the gripper command (closes); flip if X opens it


class Joint:
    """Command state of one joint: a decimal angle, so small stick values still add up."""

    def __init__(self, name, step):
        self.name = name
        self.channel = CHANNELS[name]
        self.low, self.high = LIMITS[name]
        self.step = step
        self.angle = float(NEUTRAL[name])
        self.sent = NEUTRAL[name]

    def update(self, speed, arm):
        """Move by speed * step (speed in -1..+1), stay inside the limits, send if the whole angle changed."""
        self.angle = min(self.high, max(self.low, self.angle + self.step * speed))
        target = round(self.angle)
        if target != self.sent:
            reply = move_joint(arm, self.channel, target)
            if reply != f"Joint: {self.channel}  Angle: {target}":
                print(f"\nUnexpected reply from the Arduino: {reply}")
            self.sent = target


def main():
    parser = argparse.ArgumentParser(description="Drive Marco with a gamepad.")
    parser.add_argument("--port", default=DEFAULT_PORT, help="serial port of the Arduino")
    args = parser.parse_args()

    pygame.init()
    if pygame.joystick.get_count() == 0:
        raise SystemExit("No gamepad found: plug it in and run again.")
    pad = pygame.joystick.Joystick(0)
    print("Gamepad:", pad.get_name())

    arm = open_arm(args.port)
    print("Arduino ready, arm at neutral.")
    joints = {name: Joint(name, cfg["step"]) for name, cfg in STICKS.items()}
    gripper = Joint("gripper", GRIPPER_STEP)

    try:
        while True:
            pygame.event.pump()
            if pad.get_button(BUTTON_START):
                break

            for name, cfg in STICKS.items():
                x = pad.get_axis(cfg["axis"])
                if abs(x) < DEAD_ZONE:
                    x = 0.0
                joints[name].update(cfg["sign"] * x, arm)

            speed = (pad.get_button(BUTTON_X) - pad.get_button(BUTTON_B)) * GRIPPER_CLOSE_SIGN
            gripper.update(speed, arm)

            status = "  ".join(f"{j.name} {j.sent:3d}" for j in (*joints.values(), gripper))
            print(status, end="\r")
            time.sleep(LOOP_S)
    except KeyboardInterrupt:
        pass
    finally:
        arm.close()
        pygame.quit()
        print("\nStopped, port closed.")


if __name__ == "__main__":
    main()
