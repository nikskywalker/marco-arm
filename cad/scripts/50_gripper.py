"""Marco gripper: parallel jaws, one SG90 drives a pinion between two racks.
Parts (all printed flat): frame, cover, pinion, rack (x2), finger (x2).
Frame: palm in the XY plane, pinion axis = Z (servo shaft up), racks slide along X,
fingers point +Z. Origin on the servo shaft axis, palm top Z = 0.
SG90 dimensions measured on the real servo."""
from pathlib import Path
import math
from build123d import *
from gearlib import spur_gear_face, rack_face

CAD = Path(__file__).resolve().parents[1]

# --- gear: module 1.5 (FDM sweet spot 1.5-2), 16 teeth, +0.2 on the center distance ---
M, Z = 1.5, 16
RP = M * Z / 2                     # 12.0
BACKLASH_CD = 0.2
FACE = 6.0                         # pinion and rack height (Z)
P = math.pi * M

# --- SG90 (measured) ---
SG_L, SG_W = 22.5, 12.2            # body, long side along X
SG_SHAFT_OFF = 8.5 - 4.9 / 2      # shaft center from the near body end: 6.05
SG_TAB_HOLES = 25.1 + 2.5          # tab hole to tab hole: inner edges + hole (27.6)
SG_TAB_TO_SHAFT_TOP = 31.3 - 19.9  # tab top face -> top of the shaft: 11.4
SG_CLEAR = 0.3
PALM_T = 4.0
shaft_top = -PALM_T + SG_TAB_TO_SHAFT_TOP      # tabs under the palm
body_cx = -SG_SHAFT_OFF + SG_L / 2

# --- heights ---
GAP = 0.25                         # sliding clearance (FDM sliding fit 0.2-0.3 per side)
pin_z0 = shaft_top - 1.0           # pinion bottom: horn inside a 3 mm recess
pin_z1 = pin_z0 + FACE
bed_top = pin_z0 - GAP
cover_z0 = pin_z1 + GAP
COVER_T = 3.0

# --- racks (the +Y one; the -Y one is the same part turned 180 deg about Z) ---
pitch_y = RP + BACKLASH_CD                     # 12.2
RACK_BACK = 4.0
rack_tip_y = pitch_y - M
rack_root_y = pitch_y + 1.25 * M
rack_back_y = rack_root_y + RACK_BACK
OPEN = 46.0                        # max rack gap; each rack travels OPEN/2 (jaw opening = OPEN - 2 * JAW_FRONT = 42)
POST = 5.0                         # post at the jaw end, rises through the cover
RACK_L = 12.0 + OPEN / 2 + POST    # teeth cover the contact zone (+-6 around X = 0) over the whole travel
WALL = 3.0
wall_y0 = rack_back_y + GAP
wall_y1 = wall_y0 + WALL
X_HALF = 41.0                      # frame half length

def box_at(x0, x1, y0, y1, z0, z1):
    return Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * Box(x1 - x0, y1 - y0, z1 - z0)

def cyl_z(x, y, z0, z1, d):
    return Pos(x, y, (z0 + z1) / 2) * Cylinder(d / 2, z1 - z0)

# ---------- frame ----------
frame = box_at(-X_HALF, X_HALF, -wall_y1, wall_y1, -PALM_T, 0)
for s in (-1, 1):
    # bed under the rack and outer wall, one block per side
    y_in, y_out = rack_tip_y - 0.5, wall_y1
    ya, yb = (y_in, y_out) if s > 0 else (-y_out, -y_in)
    frame += box_at(-X_HALF, X_HALF, ya, yb, 0, bed_top)
    wa, wb = (wall_y0, wall_y1) if s > 0 else (-wall_y1, -wall_y0)
    frame += box_at(-X_HALF, X_HALF, wa, wb, bed_top, cover_z0)
    for x in (-X_HALF + 4, X_HALF - 4):          # cover screws: M2 x 20 through a boss, nut under the palm
        yc = s * (wall_y0 + WALL / 2)
        frame += cyl_z(x, yc, 0, cover_z0, 7.0)
        frame -= cyl_z(x, yc, -PALM_T - 1, cover_z0 + 1, 2.4)
        with BuildSketch() as hx:
            RegularPolygon((4.0 + 0.3) / 2 / 0.8660254, 6)
        frame -= Pos(x, yc, -PALM_T - 0.01) * extrude(hx.sketch, 1.9)
# servo pocket and tab screws (SG90 screws or M2 into pilots, from below)
frame -= box_at(body_cx - SG_L / 2 - SG_CLEAR, body_cx + SG_L / 2 + SG_CLEAR,
                -SG_W / 2 - SG_CLEAR, SG_W / 2 + SG_CLEAR, -PALM_T - 1, 1)
for x in (body_cx - SG_TAB_HOLES / 2, body_cx + SG_TAB_HOLES / 2):
    frame -= cyl_z(x, 0, -PALM_T - 1, 0.5, 1.8)
# wrist bracket mount (40_forearm.py): 4 x M3 heat-set inserts in the palm, pressed in from below,
# centered on the servo body so the bracket arms straddle the servo
MOUNT_DX, MOUNT_Y = 11.0, 14.5
for sx in (-1, 1):
    for sy in (-1, 1):
        frame -= cyl_z(body_cx + sx * MOUNT_DX, sy * MOUNT_Y, -PALM_T - 0.01, -PALM_T + 6.0, 4.0)
        frame -= cyl_z(body_cx + sx * MOUNT_DX, sy * MOUNT_Y, -PALM_T - 0.01, -PALM_T + 1.0, 5.1)   # insert seat

# ---------- cover ----------
cover = box_at(-X_HALF, X_HALF, -wall_y1, wall_y1, cover_z0, cover_z0 + COVER_T)
post_y = (rack_tip_y + 1.0, rack_back_y - 1.0)          # post cross-section on the rack (Y)
SLOT_C = 0.4
for s in (-1, 1):
    ya, yb = post_y if s > 0 else (-post_y[1], -post_y[0])
    xa, xb = (-SLOT_C, OPEN / 2 + POST + SLOT_C) if s > 0 else (-(OPEN / 2 + POST + SLOT_C), SLOT_C)
    cover -= box_at(xa, xb, ya - SLOT_C, yb + SLOT_C, cover_z0 - 1, cover_z0 + COVER_T + 1)
    for x in (-X_HALF + 4, X_HALF - 4):
        cover -= cyl_z(x, s * (wall_y0 + WALL / 2), cover_z0 - 1, cover_z0 + COVER_T + 1, 2.4)
cover -= cyl_z(0, 0, cover_z0 - 1, cover_z0 + COVER_T + 1, 7.0)   # reach the horn screw

# ---------- pinion ----------
# SG90 double horn, measured: hub D 7.2, arms tangent to the hub that taper to 5.5 wide where
# they are cut (r 9.2 from the center). Printed holes come out ~0.3 small, so the clearance is per side.
HORN_HUB_D, HORN_TIP_W, HORN_CUT_R, HORN_RECESS = 7.2, 5.5, 9.2, 3.0
HORN_C = 0.30          # side clearance, chosen on a printed fit coupon (0.30 fits, no play)
HORN_C_END = 0.4       # clearance at the arm ends (fixed: the tooth roots are at r 10.1)
HORN_ARM_R = HORN_CUT_R + HORN_C_END
def horn_recess(c=None, depth=HORN_RECESS):
    """Recess for the cut SG90 double horn: hub circle + two tapered arms, bar along X, from Z = 0 up."""
    c = HORN_C if c is None else c
    hw0, hw1 = HORN_HUB_D / 2 + c, HORN_TIP_W / 2 + c
    with BuildSketch() as s:
        Circle(hw0)
        Polygon((-HORN_ARM_R, -hw1), (0, -hw0), (HORN_ARM_R, -hw1), (HORN_ARM_R, hw1), (0, hw0), (-HORN_ARM_R, hw1), align=None)
    return extrude(s.sketch, depth)
def pinion_part(bore_d):
    g = extrude(spur_gear_face(M, Z), FACE)
    g -= horn_recess()
    g -= Pos(0, 0, FACE / 2) * Cylinder(bore_d / 2, FACE * 2)
    if bore_d < 3:                       # horn screw head
        g -= Pos(0, 0, FACE - 1) * Cylinder(2.6, 2.01)
    return g
pinion = Rot(0, 0, 90) * pinion_part(2.6)     # tooth toward +Y: the +Y rack has a space at X = 0
pinion_asm = Pos(0, 0, pin_z0) * pinion
def pinion_at(gap):
    """Pinion turned so that the racks sit at this jaw gap (opening = pinion turning clockwise)."""
    return Pos(0, 0, pin_z0) * Rot(0, 0, -math.degrees(gap / 2 / RP)) * pinion

# ---------- rack (+Y) with its post; jaw end at +X ----------
def rack_part(length, with_post=True):
    x1 = 0.0; x0 = -length
    f = rack_face(M, x0, x1, pitch_y, rack_back_y, phase=P / 2 - POST)   # space at X = 0 when the jaws are closed
    r = extrude(f, FACE)
    if with_post:
        post = box_at(-POST, 0, post_y[0], post_y[1], FACE, FACE + COVER_T + GAP + 6.0)
        top = FACE + COVER_T + GAP + 6.0
        post -= Pos(-POST / 2, (post_y[0] + post_y[1]) / 2, top - 2.8) * Rot(90, 0, 0) * Cylinder(1.2, 20)   # M2 across (Y)
        r += post
    return r
rack = rack_part(RACK_L)

def rack_at(gap, side):
    """Rack placed in the frame for a jaw gap; side +1 = +Y rack (right jaw), -1 = mirrored."""
    r = Pos(gap / 2 + POST, 0, pin_z0) * rack
    return r if side > 0 else Rot(0, 0, 180) * r

# ---------- finger: a foot that caps the post (one M2 across into a captive nut) + the jaw plate ----------
# Jaw 40 tall and 36 wide (the whole frame width). The M2 goes from the outer face of the foot through the
# post into a nut held in a hex slot, open on the back of the foot (the nut slides in from behind). The jaw
# also covers the post (front wall 1.7): the jaws touch at a rack gap of 2 * JAW_FRONT, before the racks run out.
FINGER_H, FINGER_T, JAW_HW, JAW_FRONT = 40.0, 5.0, 18.0, 2.0
FOOT_X, FOOT_ROOF, FOOT_Y0, FOOT_Y1 = 7.5, 2.2, 6.5, 20.0
SOCK_C = 0.3                                   # clearance around the post
M2_NUT_AF, M2_NUT_T = 4.0, 1.6
GROOVE_D, GROOVE_W, GROOVE_P = 0.6, 1.2, 3.0   # grip grooves on the jaw face
z_pad0 = cover_z0 + COVER_T + 0.5
def finger_part():
    sock_h = 6.0 - 0.5 + 0.3                   # post sticks out 5.5 above the finger base
    z_hole = 6.0 - 0.5 - 2.8                   # the post's cross hole, from the finger base
    jaw = box_at(-JAW_FRONT, FINGER_T, -JAW_HW, JAW_HW, 0, FINGER_H)
    foot = box_at(-JAW_FRONT, FOOT_X, FOOT_Y0, FOOT_Y1, 0, sock_h + FOOT_ROOF)
    f = jaw + foot
    # socket for the post (post at x 0..POST in this frame), open at the bottom
    f -= box_at(-SOCK_C, POST + SOCK_C, post_y[0] - SOCK_C, post_y[1] + SOCK_C, -0.01, sock_h)
    # M2 across (Y): from the outer face of the foot, through the post, into the nut
    f -= Pos(POST / 2, 1.0 + (FOOT_Y1 + 1.0 - 1.0) / 2, z_hole) * Rot(90, 0, 0) * Cylinder(1.2, FOOT_Y1 + 1.0 - 1.0)
    # captive nut slot between the post and the jaw, flats up and down, open on the back of the foot
    ny1 = post_y[0] - SOCK_C - 0.7
    ny0 = ny1 - (M2_NUT_T + 0.3)
    f -= box_at(POST / 2 - (M2_NUT_AF / 0.8660254) / 2 - 0.15, FOOT_X + 0.1, ny0, ny1,
                z_hole - (M2_NUT_AF + 0.3) / 2, z_hole + (M2_NUT_AF + 0.3) / 2)
    # grip grooves across the jaw face
    z = 11.0
    while z + GROOVE_W <= FINGER_H - 2.0:
        f -= box_at(-JAW_FRONT - 0.01, -JAW_FRONT + GROOVE_D, -JAW_HW - 0.01, JAW_HW + 0.01, z, z + GROOVE_W)
        z += GROOVE_P
    return f
finger = finger_part()
def finger_at(gap, side):
    f = Pos(gap / 2, 0, z_pad0) * finger
    return f if side > 0 else Rot(0, 0, 180) * f

if __name__ == "__main__" or __name__ == "check":
    # print orientations: everything flat
    def to_bed(p):
        bb = p.bounding_box(); return Pos(-bb.center().X, -bb.center().Y, -bb.min.Z) * p
    export_stl(to_bed(frame), CAD / "stl" / "50_gripper_frame.stl")
    export_stl(to_bed(Rot(180, 0, 0) * cover), CAD / "stl" / "50_gripper_cover.stl")
    export_stl(to_bed(Rot(180, 0, 0) * pinion), CAD / "stl" / "50_gripper_pinion.stl")   # top face on the bed
    export_stl(to_bed(rack), CAD / "stl" / "50_gripper_rack.stl")
    export_stl(to_bed(Rot(0, -90, 0) * finger), CAD / "stl" / "50_gripper_finger.stl")   # x = 0 (flat face) on the bed, so the jaw does not overhang
    asm = Compound([frame, cover, pinion_at(20), rack_at(20, 1), rack_at(20, -1), finger_at(20, 1), finger_at(20, -1)])
    export_step(asm, CAD / "step" / "50_gripper.step")
    mass = sum(p.volume for p in (frame, cover, pinion, rack, rack, finger, finger)) / 1000 * 1.24
    print(f"frame {2*X_HALF:.0f} x {2*wall_y1:.1f}, rack {RACK_L:.0f} long, pinion z {pin_z0:.1f}..{pin_z1:.1f}, "
          f"solid mass {mass:.0f} g (printed less)")
