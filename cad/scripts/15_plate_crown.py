"""Crown ring for Marco: a thin ring on top of the turntable plate that covers the 4 lazy susan
screw tips (M3 x 12, about 3 mm above the plate) and carries a small engraved "marco" signature
on the back-right side (between the -Y and +X screws; link 1 works toward +Y).

Held by friction: 4 blind holes slightly smaller than the screw thread, pushed onto the screw tips.
Printed flat, bottom on the bed, text up."""
from pathlib import Path
import math
from build123d import *

HERE = Path(__file__).resolve().parent
CAD = HERE.parent

# lazy susan screws on the plate (10_turntable_plate.py): on the axes at radius 58
LS_HOLE_R = 82.1 * 2**0.5 / 2
LS_HOLES = [(LS_HOLE_R, 0), (-LS_HOLE_R, 0), (0, LS_HOLE_R), (0, -LS_HOLE_R)]

RING_W, RING_H = 12.0, 5.5    # room for the deeper holes
R_IN, R_OUT = LS_HOLE_R - RING_W / 2, LS_HOLE_R + RING_W / 2
CHAMFER = 0.6

# screw tip: about 3 mm above the plate (washers 1 + sheet 1 + plate 7 = 9, screw 12)
TIP_D = 3.2            # prints about 2.9: a push fit on the M3 thread, on purpose
TIP_DEPTH = 4.6        # 1.6 more than the screw tip (~3), 0.9 of roof above it
LEAD_IN = 0.4

# servo upright foot (20_shoulder_uprights.py): x from -49 to -30, y +-28. Its corners
# sit at radius 56.4, inside the ring band: notch the ring there, 1 mm of clearance.
FOOT_X0, FOOT_X1, FOOT_HW = -49.0, -30.0, 28.0
NOTCH_CLEAR = 1.0

# signature
TEXT = "marco"
FONT = "/usr/share/fonts/opentype/inter/Inter-SemiBold.otf"
FONT_SIZE = 7.0        # lowercase letters about 3.8 mm tall
TEXT_DEPTH = 0.5
LETTER_GAP = 1.2       # gap between letters along the arc (wider than normal: spaced out)
TEXT_R = LS_HOLE_R     # letters centered on the band
TEXT_ANGLE = -45.0     # back-right (south-east), between two screws

ring = Cylinder(R_OUT, RING_H, align=(Align.CENTER, Align.CENTER, Align.MIN)) \
     - Cylinder(R_IN, RING_H, align=(Align.CENTER, Align.CENTER, Align.MIN))
ring = chamfer(ring.edges().filter_by(GeomType.CIRCLE).group_by(Axis.Z)[-1], CHAMFER)

# notches for the servo upright foot
x0, x1 = FOOT_X0 - NOTCH_CLEAR, FOOT_X1 + NOTCH_CLEAR
hw = FOOT_HW + NOTCH_CLEAR
ring -= Pos((x0 + x1) / 2, 0, RING_H / 2) * Box(x1 - x0, 2 * hw, RING_H * 3)

# blind holes on the screw tips, with a small cone at the mouth
for x, y in LS_HOLES:
    ring -= Pos(x, y, 0) * Cylinder(TIP_D / 2, TIP_DEPTH, align=(Align.CENTER, Align.CENTER, Align.MIN))
    ring -= Pos(x, y, 0) * Cone(TIP_D / 2 + LEAD_IN, TIP_D / 2, LEAD_IN,
                                align=(Align.CENTER, Align.CENTER, Align.MIN))

# engraved signature: one letter at a time, placed along the arc, read from outside the ring
letters = []
for ch in TEXT:
    with BuildSketch() as sk:
        Text(ch, font_size=FONT_SIZE, font_path=FONT, align=(Align.CENTER, Align.CENTER))
    letters.append((sk.sketch, sk.sketch.bounding_box().size.X))
total = sum(w for _, w in letters) + LETTER_GAP * (len(letters) - 1)
s_pos = -total / 2                               # arc length from the text center
for sketch, w in letters:
    s_mid = s_pos + w / 2
    th = math.radians(TEXT_ANGLE) + s_mid / TEXT_R   # counterclockwise: left to right for the reader
    place = Pos(TEXT_R * math.cos(th), TEXT_R * math.sin(th), RING_H - TEXT_DEPTH) \
        * Rot(0, 0, math.degrees(th) + 90)
    ring -= place * extrude(sketch, TEXT_DEPTH + 0.01)
    s_pos += w + LETTER_GAP

part = ring
export_stl(part, CAD / "stl" / "15_plate_crown.stl")
export_step(part, CAD / "step" / "15_plate_crown.step")
bb = part.bounding_box()
print(f"{bb.size.X:.1f} x {bb.size.Y:.1f} x {bb.size.Z:.1f} mm, {part.volume/1000*1.24:.0f} g solid PLA")
