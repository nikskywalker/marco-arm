"""Interference check for the gripper at closed, half and fully open jaws (with an SG90 dummy)."""
import itertools, runpy
from pathlib import Path
from build123d import *
g = runpy.run_path(str(Path(__file__).resolve().parent / "50_gripper.py"), run_name="lib")
box_at = g["box_at"]
# SG90 dummy (nominal): tabs under the palm
sg_body = box_at(g["body_cx"] - g["SG_L"] / 2, g["body_cx"] + g["SG_L"] / 2, -g["SG_W"] / 2, g["SG_W"] / 2,
                 -g["PALM_T"] - 19.9, -g["PALM_T"] + 22.7 - 19.9)
sg_tabs = box_at(g["body_cx"] - 16.1, g["body_cx"] + 16.1, -g["SG_W"] / 2, g["SG_W"] / 2, -g["PALM_T"] - 2.5, -g["PALM_T"])
sg_shaft = Pos(0, 0, (g["pin_z0"] - 0.0 + g["shaft_top"]) / 2) * Cylinder(2.4, abs(g["shaft_top"] - (-g["PALM_T"] + 4.3)) + 0.01)
horn = Pos(0, 0, g["pin_z0"] + 1.0) * (Cylinder(3.6, 2.0) + Box(19.0, 5.0, 2.0))
problems = []
CLOSED = 2 * g["JAW_FRONT"] + 0.5   # the jaws touch at a rack gap of 2 * JAW_FRONT: fully closed
for gap in ((CLOSED, 20, 40) if __name__ != "render" else ()):
    parts = {"frame": g["frame"], "cover": g["cover"], "pinion": g["pinion_at"](gap),
             "rack_R": g["rack_at"](gap, 1), "rack_L": g["rack_at"](gap, -1),
             "finger_R": g["finger_at"](gap, 1), "finger_L": g["finger_at"](gap, -1),
             "sg90": sg_body + sg_tabs, "horn": horn}
    for (na, a), (nb, b) in itertools.combinations(parts.items(), 2):
        if {na, nb} in ({"pinion", "horn"}, {"rack_R", "finger_R"}, {"rack_L", "finger_L"}):
            continue                      # meant to sit inside each other
        try:
            v = (a & b).volume
        except Exception:
            v = 0
        if v > 0.5:
            problems.append((gap, na, nb, v))
print("\n".join(f"COLLISION gap {gp}: {a} x {b}: {v:.1f} mm3" for gp, a, b, v in problems) or "no collisions")
