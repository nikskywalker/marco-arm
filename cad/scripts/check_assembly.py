"""Interference check: builds the whole arm in the arm frame, with dummies for the bought parts
(servos, lazy susan, boards), and reports every pair of solids that overlap.
Link 1 is swept through several shoulder angles. Run it after changing any part."""
from pathlib import Path
import itertools, runpy
from build123d import *

HERE = Path(__file__).resolve().parent

def load(script, *names):
    g = runpy.run_path(str(HERE / script), run_name="check")
    return [g[n] for n in names]

# ---------- dummies of the bought parts ----------
def mz996():
    """MZ996 in its own frame: shaft along +Z at the origin, tab top face at Z=0, body along +X."""
    body = Pos(-9.75 + 40.4 / 2, 0, (-28 + 10) / 2) * Box(40.4, 20.0, 38.0)
    tabs = Pos(-9.75 + 40.4 / 2, 0, -1.25) * Box(40.4 + 14, 20.0, 2.5)
    spline = Pos(0, 0, 12.5) * Cylinder(3.0, 5.0)
    horn = Pos(0, 0, 16 - 1.15) * Cylinder(10.0, 2.3)
    return {"body": body + tabs, "horn": spline + horn}

def lazy_susan(z_bot, z_top):
    bore = Cylinder(27, 2)
    bottom = Pos(0, 0, z_bot + 0.5) * (Box(95, 95, 1.0) - bore)
    top = Pos(0, 0, z_top - 0.5) * (Rot(0, 0, 45) * Box(95, 95, 1.0) - bore)
    ring = Pos(0, 0, (z_bot + z_top) / 2) * (Cylinder(42, z_top - z_bot - 2) - Cylinder(27, z_top - z_bot - 2))
    return bottom + top + ring

base_box, base_floor, Z_DECK_TOP, z_pad_bot = load("05_base.py", "box", "floor", "Z_DECK_TOP", "z_pad_bot")
plate, PLATE_T = load("10_turntable_plate.py", "part", "PLATE_T")
servo_up, bear_up = load("20_shoulder_uprights.py", "servo", "bear")
crown, = load("15_plate_crown.py", "part")
leg_horn, leg_pin, spacer, AXIS_H, x_horn_in, x_pin_in, SPACER_Y = load(
    "30_link1.py", "leg_horn", "leg_pin", "spacer", "AXIS_H", "x_horn_in", "x_pin_in", "SPACER_Y")

yaw = mz996()
yaw_body = Pos(0, 0, z_pad_bot) * yaw["body"]    # tab top on the pad bottoms
yaw_horn = Pos(0, 0, z_pad_bot) * yaw["horn"]
sh = mz996()
SH = Pos(-45 + 2.5, 0, 46) * Rot(0, 90, 0)       # tab underside on the upright inner face
sh_body, sh_horn = SH * sh["body"], SH * sh["horn"]

uno = Pos(-91 + 53.3 / 2, -62 + 68.6 / 2, -57 + 0.8) * Box(53.3, 68.6, 1.6)
uno_usb = Pos(-91 + 53.34 - 37.5, -62 - 6.5 + 8, -57 + 1.6 + 5.5) * Box(12, 16, 11)   # USB on the -X side
# PCA9685: board 1.6 on 6 mm standoffs, 14 mm of parts on top (capacitor), +12 for the servo plugs
pca = Pos(-91 + 62.3 / 2, 38 + 25.4 / 2, -58 + (1.6 + 14 + 12) / 2) * Box(62.3, 25.4, 1.6 + 14 + 12)

# measured: deck top 9.6 below the plate bottom (drawn 9.0), yaw horn face 9.5 below it
REAL_DZ = -0.6
_D = Pos(0, 0, REAL_DZ)
base_box, base_floor, yaw_body, uno, uno_usb, pca = (_D * s for s in (base_box, base_floor, yaw_body, uno, uno_usb, pca))
yaw_horn = Pos(0, 0, REAL_DZ + 0.1) * yaw_horn
fixed = {
    "base_box": base_box, "base_floor": base_floor, "lazy_susan": lazy_susan(Z_DECK_TOP + REAL_DZ, -PLATE_T),
    "yaw_servo": yaw_body, "yaw_horn": yaw_horn, "plate": plate,
    "upright_servo": servo_up, "upright_bearing": bear_up,
    "plate_crown": crown,
    "shoulder_servo": sh_body, "shoulder_horn": sh_horn,
    "uno": uno + uno_usb, "pca9685": pca,
}

def link_at(angle):
    """Link 1 + elbow servo, rotated by `angle` deg about the shoulder axis (0 = horizontal, + = up)."""
    el = mz996()
    EL = Pos(x_horn_in + 2.5, 140, 0) * Rot(-90, 0, 0) * Rot(0, 90, 0)   # tab underside on the leg inner face
    parts = {"leg_horn": leg_horn, "leg_pin": leg_pin,
             "spacer_1": Pos((x_horn_in + x_pin_in) / 2, SPACER_Y[0], 0) * spacer,
             "spacer_2": Pos((x_horn_in + x_pin_in) / 2, SPACER_Y[1], 0) * spacer,
             "elbow_servo": EL * el["body"], "elbow_horn": EL * el["horn"]}
    T = Pos(0, 0, AXIS_H) * Rot(angle, 0, 0)
    return {k: T * v for k, v in parts.items()}

# parts that touch on purpose (bolted or seated): only overlaps above TOL count anyway
TOL = 1.0   # mm^3
def overlaps(a, b):
    try:
        return (a & b).volume
    except Exception:
        return 0.0

problems = []
for (na, a), (nb, b) in itertools.combinations(fixed.items(), 2):
    v = overlaps(a, b)
    if v > TOL:
        problems.append(("fixed", na, nb, v))
ANGLES = (-20, -15, -10, 0, 45, 90, 135)
for ang in ANGLES:
    link = link_at(ang)
    for (na, a), (nb, b) in itertools.product(link.items(), fixed.items()):
        v = overlaps(a, b)
        if v > TOL:
            problems.append((f"shoulder {ang:+d}°", na, nb, v))
    if ang == 0:
        for (na, a), (nb, b) in itertools.combinations(link.items(), 2):
            v = overlaps(a, b)
            if v > TOL:
                problems.append(("link", na, nb, v))

if problems:
    for where, a, b, v in problems:
        print(f"COLLISION [{where}] {a} x {b}: {v:.0f} mm3")
else:
    print("no collisions")
