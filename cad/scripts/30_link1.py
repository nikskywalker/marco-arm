"""Link 1 (upper arm) for Marco, printed as flat parts and screwed together.
  - leg_horn: bolts to the shoulder servo horn (proximal), holds the elbow servo (distal)
  - leg_pin : M5 pin into the shoulder bearing (proximal), 625ZZ seat for the elbow (distal)
  - spacer  : two blocks that hold the legs apart (print 2)
Link frame: shoulder axis = X through the origin, link along +Y, elbow axis at Y = L1.
Arm frame (STEP): link pointing horizontally along +Y, shoulder axis at Z = AXIS_H."""
from pathlib import Path
from build123d import *

CAD = Path(__file__).resolve().parents[1]   # marco-arm/cad

L1 = 140.0             # shoulder axis -> elbow axis
AXIS_H = 46.0
H = 30.0               # leg height (link "thickness" seen from the side)
END_R = H / 2          # rounded ends around the joint axes

# X positions (same frame as the shoulder uprights)
HORN_FACE_X = -26.0    # measured: servo upright outer face (-49) to leg_horn inner face = 27.0
LEG_HORN_T = 4.0
PIN_GAP = 1.0          # inner bushing between leg_pin and the bearing upright
X_BEAR_UPRIGHT = 25.5  # inner face of the bearing upright
LEG_PIN_T = 6.0        # 5 mm bearing seat + 1 mm lip at the elbow

x_horn_out = HORN_FACE_X
x_horn_in = HORN_FACE_X + LEG_HORN_T
x_pin_out = X_BEAR_UPRIGHT - PIN_GAP
x_pin_in = x_pin_out - LEG_PIN_T
INNER_W = x_pin_in - x_horn_in          # spacer length

# servo horn (round disc, measured): holes on a 15.0 circle, ~1.9 holes, disc 2.3
HORN_PCD = 13.1 + 1.9
HORN_SCREW_D = 2.4     # M2 clearance; the screws bite into the horn holes
HORN_CENTER_D = 8.0    # access to the horn screw

# elbow servo (same MZ996): pocket C, body 40.4, shaft 9.75 from the body end, tab holes 48.7
POCKET_L, POCKET_W = 41.2, 20.6
BODY_L = 40.4
CLEAR_L = (POCKET_L - BODY_L) / 2
SHAFT_OFF = 9.75
TAB_HOLE_SPAN = 48.7
TAB_HOLE_DY = 10.0
SCREW_D = 3.4

# bearings and pin
BEARING_D = 16.0 + 0.15
BEARING_T = 5.0
PIN_THROUGH_D = 10.0   # lip hole: room for the outer bushing and the nut
M5_D = 5.3

# spacers: two blocks between the legs, M3 screws through the legs into inserts
SPACER_Y = (45.0, 90.0)
SPACER_DY = 14.0
SPACER_H = 20.0
INSERT_D, INSERT_DEPTH = 4.0, 7.0   # deep: M3 x 8 (horn leg) / M3 x 12 (pin leg)
SPACER_SCREW_DZ = 10.0  # two screws per block end, 10 apart

def leg_outline(y_end):
    """Stadium from the shoulder axis to y_end, height H."""
    with BuildSketch(Plane.YZ) as s:
        with Locations((y_end / 2, 0)):
            SlotCenterToCenter(y_end, H)
    return s.sketch

def holes_through(part, pts_yz, d, x0, t):
    for y, z in pts_yz:
        part -= Pos(x0 + t / 2, y, z) * Rot(0, 90, 0) * Cylinder(d / 2, t * 3)
    return part

def spacer_pts():
    return [(y, s * SPACER_SCREW_DZ / 2) for y in SPACER_Y for s in (-1, 1)]

# ---------- leg_horn ----------
pocket_far = L1 + SHAFT_OFF + CLEAR_L          # shaft-side end of the pocket (distal)
pocket_near = pocket_far - POCKET_L
tab_far = L1 + SHAFT_OFF + (TAB_HOLE_SPAN - BODY_L) / 2
tab_near = tab_far - TAB_HOLE_SPAN
leg_horn_len = tab_far + 7.0 - END_R           # slot length: 7 mm of material past the last tab hole
leg_horn = extrude(leg_outline(leg_horn_len), LEG_HORN_T)
leg_horn = Pos(x_horn_out, 0, 0) * leg_horn
# horn: 4 screws at 45 deg + center access
import math
horn_pts = [(HORN_PCD / 2 * math.cos(math.radians(a)), HORN_PCD / 2 * math.sin(math.radians(a)))
            for a in (45, 135, 225, 315)]
leg_horn = holes_through(leg_horn, horn_pts, HORN_SCREW_D, x_horn_out, LEG_HORN_T)
leg_horn = holes_through(leg_horn, [(0, 0)], HORN_CENTER_D, x_horn_out, LEG_HORN_T)
# elbow servo pocket (long side along the link) and tab holes
leg_horn -= Pos(x_horn_out + LEG_HORN_T / 2, (pocket_far + pocket_near) / 2, 0) * Box(LEG_HORN_T * 3, POCKET_L, POCKET_W)
leg_horn = holes_through(leg_horn, [(y, s * TAB_HOLE_DY / 2) for y in (tab_far, tab_near) for s in (-1, 1)],
                         SCREW_D, x_horn_out, LEG_HORN_T)
leg_horn = holes_through(leg_horn, spacer_pts(), SCREW_D, x_horn_out, LEG_HORN_T)

# ---------- leg_pin ----------
leg_pin = Pos(x_pin_in, 0, 0) * extrude(leg_outline(L1), LEG_PIN_T)
leg_pin = holes_through(leg_pin, [(0, 0)], M5_D, x_pin_in, LEG_PIN_T)
leg_pin -= Pos(x_pin_in + BEARING_T / 2, L1, 0) * Rot(0, 90, 0) * Cylinder(BEARING_D / 2, BEARING_T)
leg_pin = holes_through(leg_pin, [(L1, 0)], PIN_THROUGH_D, x_pin_in, LEG_PIN_T)
leg_pin = holes_through(leg_pin, spacer_pts(), SCREW_D, x_pin_in, LEG_PIN_T)

# ---------- spacer (one block, centered) ----------
spacer = Box(INNER_W, SPACER_DY, SPACER_H)
for sx in (-1, 1):
    for sz in (-1, 1):
        spacer -= Pos(sx * (INNER_W / 2 - INSERT_DEPTH / 2), 0, sz * SPACER_SCREW_DZ / 2) * Rot(0, 90, 0) * Cylinder(INSERT_D / 2, INSERT_DEPTH)
        # 0.5 mm lead-in chamfer at the mouth (cone axis along X, wide end on the face)
        spacer -= Pos(sx * INNER_W / 2, 0, sz * SPACER_SCREW_DZ / 2) * Rot(0, sx * 90, 0) * Cone(INSERT_D / 2, INSERT_D / 2 + 1.0, 1.0)

# ---------- assembly in the arm frame ----------
spacers_asm = [Pos((x_horn_in + x_pin_in) / 2, y, 0) * spacer for y in SPACER_Y]
asm = Pos(0, 0, AXIS_H) * Compound([leg_horn, leg_pin] + spacers_asm)
export_step(asm, CAD / "step" / "30_link1.step")

def flat(part):
    """Lay a leg on its outer face (normal X) for printing."""
    q = Rot(0, -90, 0) * part
    bb = q.bounding_box()
    return Pos(-bb.center().X, -bb.center().Y, -bb.min.Z) * q

lh = flat(leg_horn)
lp = Rot(0, 180, 0) * flat(leg_pin)       # bearing seat facing up
bb = lp.bounding_box(); lp = Pos(0, 0, -bb.min.Z) * lp
sp = Pos(0, 0, SPACER_DY / 2) * Rot(90, 0, 0) * spacer   # insert holes horizontal, block on its side
export_stl(lh, CAD / "stl" / "30_link1_leg_horn.stl")
export_stl(lp, CAD / "stl" / "30_link1_leg_pin.stl")
export_stl(sp, CAD / "stl" / "30_link1_spacer.stl")
for n, p in (("leg_horn", lh), ("leg_pin", lp), ("spacer", sp)):
    b = p.bounding_box(); print(f"{n}: {b.size.X:.1f} x {b.size.Y:.1f} x {b.size.Z:.1f}  ({p.volume/1000*1.24:.0f} g solid)")
print(f"inner width {INNER_W:.1f}, pocket Y {pocket_near:.1f}..{pocket_far:.1f}, tab holes Y {tab_near:.2f} / {tab_far:.2f}")
