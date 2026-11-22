"""Move three joints one after the other and print the arm's replies.

Usage: python demo.py [--port COM3]
"""
import argparse

from marco import CHANNELS, DEFAULT_PORT, move_joint, open_arm

parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
parser.add_argument("--port", default=DEFAULT_PORT, help="serial port of the Arduino")
args = parser.parse_args()

arm = open_arm(args.port)
print("Arduino ready, arm at neutral.")
try:
    for joint, command in (("elbow", 70), ("shoulder", 110), ("base", 60)):
        print(move_joint(arm, CHANNELS[joint], command))
finally:
    arm.close()   # the joints stay where they are
