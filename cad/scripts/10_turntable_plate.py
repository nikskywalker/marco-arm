"""Turntable plate for Marco: sits on the lazy susan, carries the shoulder uprights.
Origin: plate center, top face at Z=0 (same origin as the other parts). X = shoulder axis.
Lazy susan dimensions measured on the real part."""
from pathlib import Path
from build123d import *

CAD = Path(__file__).resolve().parents[1]   # marco-arm/cad

# plate
PLATE_D = 150.0
PLATE_T = 7.0          # room for the M3 nut pockets on both faces
# the base (yaw) servo sits under the lazy susan, shaft up: its round horn is screwed
# to the plate underside at the center, so the cables go down through two off-center holes
HORN_PCD = 15.0        # measured (13.1 + 1.9)
HORN_SCREW_D = 2.8     # M2 free through plate and boss (2.4 prints too tight), biting into the horn
HORN_CENTER_D = 8.0    # reach the horn screw from above
# boss under the plate: goes down inside the lazy susan bore (54) to meet the yaw horn,
# so the yaw servo body can sit below the lazy susan instead of hitting it
BOSS_D = 40.0
# Measured on the real parts: yaw horn face (horn pushed on the shaft) 9.5 below the top face of the
# lazy susan upper sheet = the plate bottom. Boss 9.3: 0.2 short, no shims.
BOSS_H = 9.3
# Measured: the deck top is 9.6 below the plate bottom (lazy susan height L), the horn face 9.5:
# the horn sits 0.1 above the deck. The 40 mm boss stops 0.7 above the deck (it turns, the deck does not),
# the last 0.4 mm is a 20 mm nose on the horn (the deck bore is 22).
LS_H = 9.6
NOSE_D = 20.0
NOSE_H = BOSS_H - (LS_H - 0.7)            # 0.4
# M2 x 16 from the top: 7 + 9.3 - 3.0 = 13.3 of plastic, then the 2.3 horn disc, tip 0.4 below the disc
# (3.2 of room under the disc, measured)
SCREW_CBORE_D, SCREW_CBORE_H = 4.5, 3.0
CABLE_D = 12.0
CABLE_R = 18.0         # inside the lazy susan bore (54): holes span 12..24 from the center

# M3 nuts in hex pockets (self-aligned), no heat-set inserts
M3_D, NUT_AF, NUT_POCKET = 3.4, 5.5 + 0.2, 3.2

# lazy susan: 4 holes on an 82.1 x 82.1 square, plates rotated 45 deg.
# The upper plate holes end up on the axes at radius 82*sqrt(2)/2 = 58 mm.
LS_HOLE_R = 82.1 * 2**0.5 / 2   # measured: 77.9 + 4.2
LS_HOLES = [(LS_HOLE_R, 0), (-LS_HOLE_R, 0), (0, LS_HOLE_R), (0, -LS_HOLE_R)]

# shoulder uprights feet (20_shoulder_uprights.py): 2 holes each, 43 apart along Y, outside the servo
FOOT_HOLE_DY = 43.0
SERVO_FOOT_X = (-45.0 + -30.0) / 2     # -37.5
BEARING_FOOT_X = (10.5 + 25.5) / 2     #  18.0
FOOT_HOLES = [(x, s * FOOT_HOLE_DY / 2) for x in (SERVO_FOOT_X, BEARING_FOOT_X) for s in (-1, 1)]

with BuildPart() as plate:
    with Locations((0, 0, -PLATE_T / 2)):
        Cylinder(PLATE_D / 2, PLATE_T)
    with Locations((0, 0, -PLATE_T - (BOSS_H - NOSE_H) / 2)):
        Cylinder(BOSS_D / 2, BOSS_H - NOSE_H)
    with Locations((0, 0, -PLATE_T - BOSS_H + NOSE_H / 2)):
        Cylinder(NOSE_D / 2, NOSE_H)
    Cylinder(HORN_CENTER_D / 2, (PLATE_T + BOSS_H) * 3, mode=Mode.SUBTRACT)
    import math
    for a in (45, 135, 225, 315):
        with Locations((HORN_PCD / 2 * math.cos(math.radians(a)), HORN_PCD / 2 * math.sin(math.radians(a)), 0)):
            Cylinder(HORN_SCREW_D / 2, (PLATE_T + BOSS_H) * 3, mode=Mode.SUBTRACT)
        with Locations((HORN_PCD / 2 * math.cos(math.radians(a)), HORN_PCD / 2 * math.sin(math.radians(a)), -SCREW_CBORE_H / 2)):
            Cylinder(SCREW_CBORE_D / 2, SCREW_CBORE_H, mode=Mode.SUBTRACT)
    for sy in (-1, 1):
        with Locations((0, sy * CABLE_R, 0)):
            Cylinder(CABLE_D / 2, (PLATE_T + BOSS_H) * 3, mode=Mode.SUBTRACT)
    # uprights: M3 x 12 from the top through the foot, nut in a hex pocket under the plate (below)
    for x, y in FOOT_HOLES:
        with Locations((x, y, -PLATE_T / 2)):
            Cylinder(M3_D / 2, PLATE_T * 2, mode=Mode.SUBTRACT)
    # lazy susan upper sheet: M3 x 12 (+2 washers) from below through the sheet, nut in a hex pocket on the top face (below)

part = plate.part
def hex_prism(x, y, z0, h):
    with BuildSketch() as s:
        RegularPolygon(NUT_AF / 2 / 0.8660254, 6)
    return Pos(x, y, z0) * extrude(s.sketch, h)
for x, y in FOOT_HOLES:
    part -= hex_prism(x, y, -PLATE_T - 0.01, NUT_POCKET + 0.01)          # under the plate
for x, y in LS_HOLES:
    part -= Pos(x, y, -PLATE_T / 2) * Cylinder(M3_D / 2, PLATE_T * 2)
    part -= hex_prism(x, y, -2.6, 2.61)                                  # on the top face
export_stl(Rot(180, 0, 0) * part,  # printed top face down, boss up
            CAD / "stl" / "10_turntable_plate.stl")
export_step(part, CAD / "step" / "10_turntable_plate.step")
bb = part.bounding_box()
print(f"{bb.size.X:.1f} x {bb.size.Y:.1f} x {bb.size.Z:.1f} mm, {part.volume/1000*1.24:.0f} g solid PLA")
