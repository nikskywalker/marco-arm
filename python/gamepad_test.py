"""Print the left stick of the gamepad (no robot involved), to check the controller and its axes.

Move the left stick and watch the two numbers; Ctrl+C to stop.
"""
import time
import pygame

pygame.init()  # start pygame (it also starts the joystick module)

if pygame.joystick.get_count() == 0:
    print("No gamepad found: plug it in and run again.")
    raise SystemExit

pad = pygame.joystick.Joystick(0)  # the first gamepad Windows sees
print("Gamepad:", pad.get_name())

try:
    while True:
        pygame.event.pump()   # let pygame read the latest state from the gamepad
        x = pad.get_axis(0)   # left stick, left-right: -1 full left, 0 centre, +1 full right
        y = pad.get_axis(1)   # left stick, up-down:    -1 full up,   0 centre, +1 full down
        print(f"x = {x:+.2f}   y = {y:+.2f}", end="\r")  # \r: rewrite the same line
        time.sleep(0.05)      # 20 readings per second
except KeyboardInterrupt:
    print("\nStopped.")
