"""Forearm (link 2) and wrist for Marco.
  - fa_plate      : flat plate bolted to the elbow servo horn
  - fa_body       : channel (floor + two walls) with the bosses for the plate, a captive M5 nut for the
                    elbow pin and the cradle of the wrist SG90
  - wrist_bracket : U bracket from the wrist SG90 (horn on one side, pivot on the other) to the gripper palm
  - wrist_bushing : sleeve for the pivot screw (the bracket turns on it)
Forearm frame: elbow axis = X through the origin, forearm along +Y, wrist axis at Y = L2.
X positions are in the arm frame, the same as in 30_link1.py.
Arm frame (STEP): link 1 and forearm horizontal along +Y."""
from pathlib import Path
import math, runpy
from build123d import *

HERE = Path(__file__).resolve().parent
CAD = HERE.parent
_l1 = runpy.run_path(str(HERE / "30_link1.py"), run_name="lib")
_gr = runpy.run_path(str(HERE / "50_gripper.py"), run_name="lib")
L1, AXIS_H, x_horn_in, x_pin_in = _l1["L1"], _l1["AXIS_H"], _l1["x_horn_in"], _l1["x_pin_in"]
HORN_PCD, HORN_SCREW_D, HORN_CENTER_D = _l1["HORN_PCD"], _l1["HORN_SCREW_D"], _l1["HORN_CENTER_D"]
M5_D, SCREW_D, INSERT_D, PIN_GAP = _l1["M5_D"], _l1["SCREW_D"], _l1["INSERT_D"], _l1["PIN_GAP"]
SG_L, SG_W, SG_SHAFT_OFF, SG_TAB_HOLES, SG_CLEAR = (_gr[k] for k in ("SG_L", "SG_W", "SG_SHAFT_OFF", "SG_TAB_HOLES", "SG_CLEAR"))
HORN_RECESS, horn_recess = _gr["HORN_RECESS"], _gr["horn_recess"]
G_BODY_CX, G_PALM_T, MOUNT_DX, MOUNT_Y = _gr["body_cx"], _gr["PALM_T"], _gr["MOUNT_DX"], _gr["MOUNT_Y"]

L2 = 140.0             # elbow axis -> wrist axis
H = 26.0               # forearm height (the round servo horn is 20)
R = H / 2

# --- X stack at the elbow ---
HORN_STACK = 19.0      # servo: plate face under the tabs -> horn face (shoulder, measured: -45 -> -26)
PLATE_T = 4.0
FLOOR_T = 4.0
x_horn_face = x_horn_in + HORN_STACK      # -3.0: outer face of fa_plate
x_plate_in = x_horn_face + PLATE_T        # +1.0
x_out = x_pin_in - PIN_GAP                # 17.5: outer face of fa_body (1 mm bushing to the elbow bearing)
x_floor = x_out - FLOOR_T                 # 13.5
BODY_DEPTH = x_out - x_plate_in           # 16.5: check it against the real gap before printing fa_body

# --- plate <-> body: 4 x M3 x 8 through the plate into inserts in the bosses ---
PLATE_LEN = 66.0
BOSS_Y = (28.0, 58.0)  # far enough from the elbow axis: reachable with the elbow at 90 deg
BOSS = 8.0
WALL = 3.0
WALL_Y0 = 16.0
ZB = R - BOSS / 2
INSERT_DEPTH = 6.5

# --- elbow pin: M5 x 20 from outside (through the bearing in leg_pin) into a nut held in a collar ---
NUT_AF, NUT_H = 8.0 + 0.1, 5.0

# --- wrist SG90 (measured): shaft along -X on the wrist axis, body bottom on a pad ---
PAD_T = 6.0                               # floor under the servo: room for the pivot insert
SG_BOT_TO_TAB_TOP, SG_TAB_T, SG_BOT_TO_SHAFT_TOP = 19.9, 2.5, 31.3
x_sg_bot = x_out - PAD_T                  # 11.5
x_tab_bot = x_sg_bot - (SG_BOT_TO_TAB_TOP - SG_TAB_T)   # -5.9: top of the posts
x_shaft_top = x_sg_bot - SG_BOT_TO_SHAFT_TOP            # -19.8
yc = L2 + SG_SHAFT_OFF - SG_L / 2         # servo body center (shaft at the distal end)
y_a, y_b = yc - SG_L / 2 - SG_CLEAR, yc + SG_L / 2 + SG_CLEAR
POST_A, POST_B, POST_W = 7.0, 6.0, 14.0
CABLE = 2.5                               # notch in the posts: the cable turns sideways there

# --- wrist bracket ---
# The horn hub has to slide 3.2 mm along the wrist axis onto the shaft, so the bracket needs room to move
# sideways during assembly. Measured on the real parts:
# SG90 shaft top -> outer face of the floor 38.2 (drawn 37.3), horn face (horn pushed on) -> floor 40.0,
# horn face -> hub end 5.0, printed gap between the arms 0.35 smaller than drawn.
# Arm A is placed on the measured horn face; arm B is ASM_SHIFT further out, so the bracket can slide
# toward arm A, take the horn onto the shaft and come back; then a spacer ring goes in sideways and a long
# bushing goes through arm B and the ring onto the floor (M3 x 12 into the pivot insert, no washer).
D_W = 36.0             # wrist axis -> underside of the gripper palm (the gripper servo clears the forearm tip)
ARM_A_T, ARM_B_T, ARM_W, BASE_T, BASE_HW = 6.0, 3.5, 24.0, 3.0, 19.0
SHAFT_TO_FLOOR, HORN_FACE_TO_FLOOR = 38.2, 40.0      # measured
ASM_SHIFT, PRINT_COMP = 4.5, 0.35
x_shaft_top_real = x_out - SHAFT_TO_FLOOR            # -20.7 (the drawn servo said -19.8)
SG_REAL_DX = x_shaft_top_real - x_shaft_top          # real servo vs drawn servo, along X
XA1 = x_out - HORN_FACE_TO_FLOOR + HORN_RECESS       # inner face of arm A: horn face on the recess bottom
XA0 = XA1 - ARM_A_T
XB0 = x_out + 0.15 + ASM_SHIFT + PRINT_COMP          # pivot side, room to slide onto the shaft
XB1 = XB0 + ARM_B_T
Xc = (x_shaft_top + 1.0 + x_out + 0.15) / 2          # gripper centred between the servo shaft top and the floor
X_GRIP = Xc + G_BODY_CX                   # arm-frame X of the gripper center (gripper x axis = -X)
BUSH_OD = 7.0
BUSH_L = XB1 + 0.3 - x_out                # from the floor face to 0.3 past arm B: the screw clamps the bushing
SPACER_T = XB0 - x_out - 0.5              # ring between floor and arm B, slid in from the side
SPACER_OD, SPACER_ID = 12.0, BUSH_OD + 0.4

def bx(x0, x1, y0, y1, z0, z1):
    return Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * Box(x1 - x0, y1 - y0, z1 - z0)

def cx(x0, x1, y, z, d):
    """Cylinder along X."""
    return Pos((x0 + x1) / 2, y, z) * Rot(0, 90, 0) * Cylinder(d / 2, x1 - x0)

def stadium(x0, x1, y0, y1, r):
    return bx(x0, x1, y0, y1, -r, r) + cx(x0, x1, y0, 0, 2 * r) + cx(x0, x1, y1, 0, 2 * r)

def boss_pts():
    return [(y, s * ZB) for y in BOSS_Y for s in (-1, 1)]

# ---------- fa_plate ----------
fa_plate = stadium(x_horn_face, x_plate_in, 0, PLATE_LEN, R)
for a in (45, 135, 225, 315):
    fa_plate -= cx(x_horn_face - 1, x_plate_in + 1, HORN_PCD / 2 * math.cos(math.radians(a)),
                   HORN_PCD / 2 * math.sin(math.radians(a)), HORN_SCREW_D)
fa_plate -= cx(x_horn_face - 1, x_plate_in + 1, 0, 0, HORN_CENTER_D)
for y, z in boss_pts():
    fa_plate -= cx(x_horn_face - 1, x_plate_in + 1, y, z, SCREW_D)

# ---------- fa_body ----------
fa_body = stadium(x_floor, x_out, 0, L2, R)
for s in (-1, 1):
    z0, z1 = (R - WALL, R) if s > 0 else (-R, -R + WALL)
    fa_body += bx(x_plate_in, x_floor, WALL_Y0, L2, z0, z1)
    zb0, zb1 = (R - BOSS, R) if s > 0 else (-R, -R + BOSS)
    for y in BOSS_Y:
        fa_body += bx(x_plate_in, x_floor, y - BOSS / 2, y + BOSS / 2, zb0, zb1)
for y, z in boss_pts():
    fa_body -= cx(x_plate_in - 0.01, x_plate_in + INSERT_DEPTH, y, z, INSERT_D)
    fa_body -= cx(x_plate_in - 0.01, x_plate_in + 1.0, y, z, 5.1)            # insert seat
# nut collar on the elbow axis
fa_body += cx(x_floor - NUT_H, x_floor, 0, 0, 13.0)
with BuildSketch() as _hx:
    RegularPolygon(NUT_AF / 2 / 0.8660254, 6)
fa_body -= Pos(x_floor - NUT_H - 0.01, 0, 0) * Rot(0, 90, 0) * extrude(_hx.sketch, NUT_H + 0.01)
fa_body -= cx(x_floor - NUT_H - 1, x_out + 1, 0, 0, M5_D)
# wrist servo cradle: pad under the body, a post under each tab
fa_body += bx(x_sg_bot, x_floor, y_a, y_b, -SG_W / 2 - 0.5, SG_W / 2 + 0.5)
fa_body += bx(x_tab_bot, x_floor, y_a - POST_A, y_a, -POST_W / 2, POST_W / 2)
fa_body += bx(x_tab_bot, x_floor, y_b, y_b + POST_B, -POST_W / 2, POST_W / 2)
for y in (yc - SG_TAB_HOLES / 2, yc + SG_TAB_HOLES / 2):
    fa_body -= cx(x_tab_bot - 1, x_tab_bot + 9, y, 0, 1.8)                    # pilots for the SG90 screws
fa_body -= bx(x_sg_bot - 4.5, x_sg_bot, y_a - CABLE, y_a + 0.01, -POST_W / 2 - 1, POST_W / 2 + 1)
fa_body -= bx(x_sg_bot - 4.5, x_sg_bot, y_b - 0.01, y_b + CABLE, -POST_W / 2 - 1, POST_W / 2 + 1)
# pivot insert on the wrist axis, pressed in from the outer face
fa_body -= cx(x_sg_bot - 1, x_out + 1, L2, 0, INSERT_D)
fa_body -= cx(x_out - 1.0, x_out + 0.01, L2, 0, 5.1)

# ---------- wrist bracket (drawn at wrist angle 0: gripper in line with the forearm) ----------
yb0, yb1 = L2 + D_W - BASE_T, L2 + D_W
GUS = 6.0
def _arm(x0, x1):
    return bx(x0, x1, L2, yb0, -ARM_W / 2, ARM_W / 2) + cx(x0, x1, L2, 0, ARM_W)
wrist_bracket = _arm(XA0, XA1) + _arm(XB0, XB1) + bx(XA0 - GUS, XB1 + GUS, yb0, yb1, -BASE_HW, BASE_HW)
for xa, sgn in ((XA0, -1), (XB1, 1)):                                          # gussets outside the arms
    tri = Polygon((xa, yb0), (xa + sgn * GUS, yb0), (xa, yb0 - 2 * GUS), align=None)
    wrist_bracket += extrude(tri, ARM_W / 2, both=True)
# horn recess on the inner face of arm A (double horn, bar along the arm) and the horn screw
wrist_bracket -= Pos(XA1 - HORN_RECESS, L2, 0) * Rot(0, 90, 0) * Rot(0, 0, 90) * horn_recess(depth=HORN_RECESS + 0.01)
wrist_bracket -= cx(XA0 - 1, XA1 + 1, L2, 0, 2.6)
wrist_bracket -= cx(XA0 - 0.01, XA0 + 1.0, L2, 0, 5.2)
# pivot hole (turns on the bushing)
wrist_bracket -= cx(XB0 - 1, XB1 + 1, L2, 0, BUSH_OD + 0.3)
# window for the gripper servo and its tabs, 4 x M3 into the palm inserts
wrist_bracket -= bx(Xc - 16.7, Xc + 16.7, yb0 - 1, yb1 + 1, -SG_W / 2 - 0.7, SG_W / 2 + 0.7)
for sx in (-1, 1):
    for sz in (-1, 1):
        wrist_bracket -= Pos(Xc + sx * MOUNT_DX, (yb0 + yb1) / 2, sz * MOUNT_Y) * Rot(90, 0, 0) * Cylinder(SCREW_D / 2, BASE_T + 2)
# printed holes come out ~0.3 small: the bushing is 6.4 so it slides in the 7.3 hole of arm B
BUSH_PRINT_OD = 6.4
wrist_bushing = Cylinder(BUSH_PRINT_OD / 2, BUSH_L) - Cylinder(3.6 / 2, BUSH_L)
wrist_spacer = Cylinder(SPACER_OD / 2, SPACER_T) - Cylinder(SPACER_ID / 2, SPACER_T)

# gripper placement (gripper frame -> forearm frame): x -> -X, y -> +Z, z (fingers) -> +Y
GRIP_T = Pos(X_GRIP, L2 + D_W + G_PALM_T, 0) * Rot(0, 180, 0) * Rot(-90, 0, 0)
L3 = D_W + G_PALM_T + _gr["z_pad0"] + _gr["FINGER_H"] / 2      # wrist axis -> middle of the jaws

if __name__ == "__main__":
    def to_bed(p):
        bb = p.bounding_box(); return Pos(-bb.center().X, -bb.center().Y, -bb.min.Z) * p
    parts = {"plate": to_bed(Rot(0, -90, 0) * fa_plate),          # horn face on the bed
             "body": to_bed(Rot(0, 90, 0) * fa_body),            # outer face on the bed, cradle up
             "wrist_bracket": to_bed(Rot(-90, 0, 0) * wrist_bracket),   # palm face on the bed, arms up
             "wrist_bushing": to_bed(wrist_bushing),
             "wrist_spacer": to_bed(wrist_spacer)}
    names = {"plate": "40_forearm_plate", "body": "40_forearm_body",
             "wrist_bracket": "40_wrist_bracket", "wrist_bushing": "40_wrist_bushing",
             "wrist_spacer": "40_wrist_spacer"}
    for k, p in parts.items():
        export_stl(p, CAD / "stl" / f"{names[k]}.stl")
        b = p.bounding_box()
        print(f"{k}: {b.size.X:.1f} x {b.size.Y:.1f} x {b.size.Z:.1f}  ({p.volume / 1000 * 1.24:.0f} g solid)")
    asm = Pos(0, L1, AXIS_H) * Compound([fa_plate, fa_body, wrist_bracket])
    export_step(asm, CAD / "step" / "40_forearm.step")
    print(f"body depth {BODY_DEPTH:.1f}, plate X {x_horn_face:.1f}..{x_plate_in:.1f}, body X {x_floor:.1f}..{x_out:.1f}")
    print(f"wrist: shaft top X {x_shaft_top:.1f}, arms X {XA0:.1f}..{XA1:.1f} / {XB0:.2f}..{XB1:.2f}, gripper center X {X_GRIP:.2f}")
    print(f"L3 (wrist axis -> middle of the jaws) {L3:.1f}")
