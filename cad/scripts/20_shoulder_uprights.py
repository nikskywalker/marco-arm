"""Shoulder uprights for Marco.
Design frame: origin at plate center, plate top at Z=0, X = shoulder axis, X=0 = arm plane.
Servo upright on -X (servo enters from inside the U, tabs on the inner face, body out to -X).
Bearing upright on +X (625ZZ seat on the inner face, M5 pin).
Exports each upright lying on its outer face, ready to print, plus a STEP in design frame."""
from pathlib import Path
from build123d import *

CAD = Path(__file__).resolve().parents[1]   # marco-arm/cad

# --- servo pocket: three sizes tried on a printed fit test, C is the one that fits ---
COMB = {"A": (39.6, 19.6), "B": (40.4, 20.1), "C": (41.2, 20.6)}
CHOICE = "C"           # body 40.4 x 20.0, only C fits
POCKET_L, POCKET_W = COMB[CHOICE]       # along Z, along Y (clearance included)
BODY_L = 40.4          # measured (width 20.0)
CLEAR_L = (POCKET_L - BODY_L) / 2       # gap at each end of the body

# --- shoulder geometry ---
AXIS_H = 46.0          # shoulder axis height above plate top
SHAFT_OFF = 9.75       # measured: 12.7 - 5.9/2
UP_W = 30.0            # upright width along Y
TOP_MARGIN = 12.0      # servo upright: material above the pocket
WALL_SERVO = 4.0
WALL_BEAR = 6.0
X_SERVO_INNER = -45.0  # inner face of the servo upright
X_BEAR_INNER = 25.5    # inner face of the bearing upright
FOOT_D = 15.0          # foot length toward the U center
# The foot is wider than the wall, so its two screws sit outside the servo and the link, where a key
# reaches them (Y = +-21.5, the servo is +-10 wide). M3 x 12 from the top through foot and plate, nut in a hex
# pocket under the plate (no inserts). FOOT_T 5.6: the screw tip stays 0.6 above the plate bottom.
FOOT_T = 5.6
FOOT_W = UP_W + 26.0   # foot width along Y (the wall is UP_W)
FOOT_HOLE_DY = UP_W + 13.0
SCREW_D = 3.4          # M3 clearance (feet into the plate nuts, servo tabs with nuts)
CORNER_R = 3.0         # fillet inside the L corner

# --- servo tab screws: M3 through slots, nut on the outer face ---
TAB_HOLE_SPAN = 48.7   # measured: hole center to hole center (44.2 + 4.5)
TAB_HOLE_OFF = (TAB_HOLE_SPAN - BODY_L) / 2   # hole center beyond the body end
TAB_HOLE_DY = 10.0     # across the width
TAB_SLOT_L = 1.0       # short slot: small tolerance on the measured hole span

# --- bearing 625ZZ (5 x 16 x 5) ---
BEARING_D = 16.0 + 0.15  # printed holes shrink: small press fit
BEARING_T = 5.0
PIN_HOLE_D = 10.0        # through: room for the outer bushing (OD 7) and the M5 nut
BEAR_TOP = 62.0

pocket_top = AXIS_H + SHAFT_OFF + CLEAR_L
pocket_bot = pocket_top - POCKET_L
servo_h = pocket_top + TOP_MARGIN

def l_upright(x_outer, x_inner, height, foot_dir):
    """L shape: wall between x_outer and x_inner, foot pointing toward the U center."""
    t = abs(x_inner - x_outer)
    with BuildPart() as p:
        with Locations(((x_outer + x_inner) / 2, 0, height / 2)):
            Box(t, UP_W, height)
        fx = x_inner + foot_dir * FOOT_D / 2
        # the foot also runs under the wall to its outer face, so the wings wider than the wall
        # sit on the bed when the upright is printed on its outer face
        fx0, fx1 = sorted((x_outer, x_inner + foot_dir * FOOT_D))
        with Locations(((fx0 + fx1) / 2, 0, FOOT_T / 2)):
            Box(fx1 - fx0, FOOT_W, FOOT_T)
        corner = p.edges().filter_by(Axis.Y).filter_by(
            lambda e: abs(e.center().X - x_inner) < 1e-6 and abs(e.center().Z - FOOT_T) < 1e-6
            and abs(e.length - UP_W) < 1e-3)
        fillet(corner, CORNER_R)
        for s in (-1, 1):
            with Locations((fx, s * FOOT_HOLE_DY / 2, FOOT_T / 2)):
                Cylinder(SCREW_D / 2, FOOT_T * 2, mode=Mode.SUBTRACT)
    return p.part

# servo upright
servo = l_upright(X_SERVO_INNER - WALL_SERVO, X_SERVO_INNER, servo_h, +1)
xc = X_SERVO_INNER - WALL_SERVO / 2
servo -= Pos(xc, 0, (pocket_top + pocket_bot) / 2) * Box(WALL_SERVO * 3, POCKET_W, POCKET_L)
for z_end, sign in ((pocket_top, +1), (pocket_bot, -1)):
    zc = z_end + sign * (TAB_HOLE_OFF - CLEAR_L)
    for s in (-1, 1):
        slot = Pos(xc, s * TAB_HOLE_DY / 2, zc) * Rot(0, 90, 0) * extrude(
            SlotCenterToCenter(TAB_SLOT_L, SCREW_D), WALL_SERVO * 3, both=True)
        servo -= slot

# bearing upright
bear = l_upright(X_BEAR_INNER + WALL_BEAR, X_BEAR_INNER, BEAR_TOP, -1)
seat = Pos(X_BEAR_INNER + BEARING_T / 2, 0, AXIS_H) * Rot(0, 90, 0) * Cylinder(BEARING_D / 2, BEARING_T)
pin = Pos(X_BEAR_INNER + WALL_BEAR / 2, 0, AXIS_H) * Rot(0, 90, 0) * Cylinder(PIN_HOLE_D / 2, WALL_BEAR * 3)
bear = bear - seat - pin

# bushings for the M5 pin: they touch only the bearing inner ring (OD 7 < shield)
def bushing(length):
    return Cylinder(3.5, length) - Cylinder(5.3 / 2, length)
export_stl(Pos(0, 0, 0.5) * bushing(1.0), CAD / "stl" / "20_pin_bushing_inner_1mm.stl")
export_stl(Pos(0, 0, 0.75) * bushing(1.5), CAD / "stl" / "20_pin_bushing_outer_1p5mm.stl")

export_step(Compound([servo, bear]), CAD / "step" / "20_shoulder_uprights.step")

def lay_on_outer(part, outer_is_min_x):
    """Rotate so the outer face is on the bed (Z=0)."""
    r = Rot(0, -90, 0) if outer_is_min_x else Rot(0, 90, 0)
    q = r * part
    bb = q.bounding_box()
    return Pos(-bb.center().X, -bb.center().Y, -bb.min.Z) * q

servo_p = lay_on_outer(servo, True)
bear_p = lay_on_outer(bear, False)
export_stl(servo_p, CAD / "stl" / "20_shoulder_upright_servo.stl")
export_stl(bear_p, CAD / "stl" / "20_shoulder_upright_bearing.stl")
for name, p in (("servo", servo_p), ("bearing", bear_p)):
    bb = p.bounding_box()
    print(f"{name}: {bb.size.X:.1f} x {bb.size.Y:.1f} x {bb.size.Z:.1f} mm (print), {p.volume/1000*1.24:.0f} g solid")
print(f"pocket Z {pocket_bot:.1f} .. {pocket_top:.1f}, servo upright height {servo_h:.1f}")
