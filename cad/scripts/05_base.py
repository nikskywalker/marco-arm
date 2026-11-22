"""Base for Marco: box with the yaw servo and the electronics, clamped to the desk.
  - base_box : deck (lazy susan screws on top) + walls + yaw servo pads, printed upside down
  - base_floor: bottom plate with the board standoffs and the two clamp tabs
Arm frame: origin at the arm axis, plate top Z = 0 (plate 7 thick). Lazy susan from Z -16 to -7.
Yaw servo: shaft up on the axis, tab top Z = -32, horn face about -16, just below the plate boss."""
from pathlib import Path
import math
from build123d import *

CAD = Path(__file__).resolve().parents[1]

# box
X0, X1, Y0, Y1 = -105.0, 52.0, -75.0, 75.0
Z_DECK_TOP, DECK_T = -16.0, 5.0
Z_FLOOR_TOP = -64.0
WALL = 3.0
FLOOR_T = 5.0
BORE_D = 22.0          # only the yaw horn (Ø20) passes: both servo pads sit fully under the deck (no overhang)
CABLE_SLOT = (7.0, 11.0, 26.0)   # half width, Y from, Y to: one slot per side, under the plate cable holes (Y = +-18)
# The plate cable holes (D12 at R18) turn with the base, and the plate boss runs 0.7 mm above the deck:
# the cables cannot bend in between, so each slot is widened into an arc that follows its hole.
CABLE_ARC_R, CABLE_ARC_D, CABLE_ARC_SPAN = 18.0, 12.5, 30.0   # hole circle, swept width, +- degrees from +-Y
CORNER_BOSS_D, CORNER_IN = 10.0, 8.0
INSERT_D, INSERT_DEPTH = 4.0, 5.0
SCREW_D = 3.4
# The 4 lazy susan screws go into M3 nuts in hex pockets under the deck, not heat-set inserts: the steel
# sheet leaves only 0.6 mm per side, so a slightly crooked insert binds the screw. The nuts self-align.
# M3 x 8: 1 sheet + 2.5 deck, then the whole nut (2.4).
NUT_AF, NUT_POCKET_H = 5.5 + 0.2, 2.5

# lazy susan bottom plate: 4 holes on an 82.1 square, axis aligned
LS = 82.1 / 2

# yaw servo (MZ996, measured): shaft on the axis, body along +X
SHAFT_OFF, BODY_L = 9.75, 40.4
TAB_TOP_Z = -31.0
PAD_EXTRA = 1.0        # pads 1 mm low, so the horn clears the plate boss (see Known issues in the README)
tab_x = (-SHAFT_OFF - (48.7 - BODY_L) / 2, -SHAFT_OFF + BODY_L + (48.7 - BODY_L) / 2)
TAB_DY = 10.0
body_x = (-SHAFT_OFF - 0.4, -SHAFT_OFF + BODY_L + 0.4)
PADS = [((-22.0, body_x[0]), (-13.0, 13.0)), ((body_x[1], 43.0), (-13.0, 13.0))]

# cable exit (USB + power) in the -Y wall, at floor level
EXIT_X, EXIT_H = (-92.0, -37.0), 22.0   # the USB-B plug is 10.5 tall, centre ~13 above the floor

z_deck_bot = Z_DECK_TOP - DECK_T
z_pad_bot = TAB_TOP_Z - PAD_EXTRA
corners = [(x, y) for x in (X0 + CORNER_IN, X1 - CORNER_IN) for y in (Y0 + CORNER_IN, Y1 - CORNER_IN)]

def box_at(x0, x1, y0, y1, z0, z1):
    return Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * Box(x1 - x0, y1 - y0, z1 - z0)

def cyl_at(x, y, z0, z1, d):
    return Pos(x, y, (z0 + z1) / 2) * Cylinder(d / 2, z1 - z0)

# Positioning seat at every insert hole mouth: a straight pocket
# just wider than the insert (D5), 1 mm deep, then a cone down to the D4 hole. The insert stands
# upright in it by itself before the iron touches it, so it goes in straight.
SEAT_D, SEAT_H, SEAT_CONE = 5.1, 1.0, 0.55

def lead_in(x, y, z, up):
    """Positioning seat at an insert hole mouth on the face at height z (up: face looks toward +Z)."""
    rs, ri = SEAT_D / 2, INSERT_D / 2
    s = -1 if up else 1                      # direction into the material
    pocket = cyl_at(x, y, min(z, z + s * SEAT_H) - (1 if not up else 0), max(z, z + s * SEAT_H) + (1 if up else 0), SEAT_D)
    zc = z + s * (SEAT_H + SEAT_CONE / 2)
    cone = Pos(x, y, zc) * Cone(ri if up else rs, rs if up else ri, SEAT_CONE)
    return pocket + cone

def nut_pocket(x, y, z_face):
    """M3 hex nut pocket going up into the material from a face that looks toward -Z."""
    with BuildSketch() as hx:
        RegularPolygon(NUT_AF / 2 / 0.8660254, 6)
    return Pos(x, y, z_face - 0.01) * extrude(hx.sketch, NUT_POCKET_H + 0.01)

# ---------- base_box ----------
box = box_at(X0, X1, Y0, Y1, Z_FLOOR_TOP, Z_DECK_TOP)
box -= box_at(X0 + WALL, X1 - WALL, Y0 + WALL, Y1 - WALL, Z_FLOOR_TOP - 1, z_deck_bot)
box -= cyl_at(0, 0, z_deck_bot - 1, Z_DECK_TOP + 1, BORE_D)
for sy in (-1, 1):   # cables come down beside the yaw servo, not onto it
    hw, ya, yb = CABLE_SLOT
    box -= box_at(-hw, hw, sy * ya if sy > 0 else -yb, sy * yb if sy > 0 else -ya, z_deck_bot - 1, Z_DECK_TOP + 1)
    centre = 90.0 if sy > 0 else 270.0
    for k in range(int(2 * CABLE_ARC_SPAN / 2) + 1):   # the hole swept along its arc, every 2 deg
        a = math.radians(centre - CABLE_ARC_SPAN + 2 * k)
        # through the deck only: where the arc passes over a servo pad, the pad stays whole
        box -= cyl_at(CABLE_ARC_R * math.cos(a), CABLE_ARC_R * math.sin(a), z_deck_bot, Z_DECK_TOP + 1, CABLE_ARC_D)
for sx in (-1, 1):
    for sy in (-1, 1):
        box -= cyl_at(sx * LS, sy * LS, z_deck_bot - 1, Z_DECK_TOP + 1, SCREW_D)
        box -= nut_pocket(sx * LS, sy * LS, z_deck_bot)
# access holes: the lazy susan TOP plate is screwed to the turntable plate from below, through
# its 4 corner holes (on the axes, r = 58). Three of them sit above the deck: a screwdriver
# reaches them from inside the box (before the floor goes on) through these holes.
LS_TOP_R = 82.1 * 2**0.5 / 2
ACCESS_D = 10.0         # M3 head, the 2 washers (OD 7) and the screwdriver pass
for x, y in ((LS_TOP_R, 0), (-LS_TOP_R, 0), (0, LS_TOP_R), (0, -LS_TOP_R)):
    if X0 < x < X1 and Y0 < y < Y1:
        box -= cyl_at(x, y, z_deck_bot - 1, Z_DECK_TOP + 1, ACCESS_D)
for (xa, xb), (ya, yb) in PADS:
    box += box_at(xa, xb, ya, yb, z_pad_bot, z_deck_bot)
for x in tab_x:
    for s in (-1, 1):
        box -= cyl_at(x, s * TAB_DY / 2, z_pad_bot - 1, z_pad_bot + 6.5, INSERT_D)   # M3 x 8 through the 2.5 tab
        box -= lead_in(x, s * TAB_DY / 2, z_pad_bot, False)
for x, y in corners:
    box += cyl_at(x, y, Z_FLOOR_TOP, z_deck_bot, CORNER_BOSS_D)
    box -= cyl_at(x, y, Z_FLOOR_TOP - 1, Z_FLOOR_TOP + 6.5, INSERT_D)   # M3 x 8 from below the floor
    box -= lead_in(x, y, Z_FLOOR_TOP, False)
box -= box_at(EXIT_X[0], EXIT_X[1], Y0 - 1, Y0 + WALL + 1, Z_FLOOR_TOP - 1, Z_FLOOR_TOP + EXIT_H)

# ---------- base_floor ----------
z_fb = Z_FLOOR_TOP - FLOOR_T
floor = box_at(X0, X1, Y0, Y1, z_fb, Z_FLOOR_TOP)
CLAMP_TABS = [(-62.0, -32.0), (32.0, 62.0)]   # Y ranges, tabs stick out 20 mm past X1
for ya, yb in CLAMP_TABS:
    floor += box_at(X1 - 1, X1 + 20, ya, yb, z_fb, Z_FLOOR_TOP)
for x, y in corners:
    floor -= cyl_at(x, y, z_fb - 1, Z_FLOOR_TOP + 1, SCREW_D)
    floor -= cyl_at(x, y, z_fb - 1, z_fb + 3.2, 6.5)        # head counterbore from below (a socket head is 3 tall)
# Arduino Uno, long side along Y, USB edge toward the -Y wall (cable exit).
# The USB-B and DC jacks stick out ~6 mm past the board edge: board edge 10 mm from the wall.
# Hole pattern: Arduino standard (not measured on this clone). No inserts, so the
# board does not depend on exact hole positions: M2 x 12 through the board (hole 3.2), the standoff
# (hole 3.0) and the floor, M2 nut in a loose hex pocket under the floor, as for the PCA9685.
# About +-0.5 mm of play at every hole. Seen from above with the USB edge toward -Y, the USB jack
# is on the -X side, so the short-side coordinate runs toward -X.
UNO_X0, UNO_Y0, UNO_W = -91.0, -62.0, 53.34
uno_holes = [(13.97, 2.54), (15.24, 50.8), (66.04, 7.62), (66.04, 35.56)]   # (along long side, along short side)
STANDOFF_H = 6.0
UNO_HOLE_D = 3.0
M2_NUT_POCKET = 4.0 + 0.3      # M2 nut across flats + clearance (tight, so the nut cannot spin; the 3.0 hole gives the play)
for bx, by in uno_holes:
    x, y = UNO_X0 + UNO_W - by, UNO_Y0 + bx
    floor += cyl_at(x, y, Z_FLOOR_TOP, Z_FLOOR_TOP + STANDOFF_H, 7.0)
    floor -= cyl_at(x, y, z_fb - 1, Z_FLOOR_TOP + STANDOFF_H + 1, UNO_HOLE_D)
    with BuildSketch() as hx:
        RegularPolygon(M2_NUT_POCKET / 2 / 0.8660254, 6)
    floor -= Pos(x, y, z_fb - 0.01) * extrude(hx.sketch, 1.9)
# PCA9685, measured: board 62.3 x 25.4, holes D2.3, centers 55.8 x 19.1 apart (centered).
# Under the board the solder joints stick out 2 mm (standoff 6); on top, 14 mm at the capacitor.
# Long side along X, behind the Uno.
# M2 x 12 through the standoff and the floor, nut in a hex pocket under the floor
# (the kit screws are machine screws: they do not cut their own thread in plastic).
PCA_X0, PCA_Y0, PCA_LX, PCA_WY = -91.0, 38.0, 62.3, 25.4
PCA_HX, PCA_HY = 55.8, 19.1   # hole center spacing
PCA_EX, PCA_EY = (PCA_LX - PCA_HX) / 2, (PCA_WY - PCA_HY) / 2
for x in (PCA_X0 + PCA_EX, PCA_X0 + PCA_EX + PCA_HX):
    for y in (PCA_Y0 + PCA_EY, PCA_Y0 + PCA_EY + PCA_HY):
        floor += cyl_at(x, y, Z_FLOOR_TOP, Z_FLOOR_TOP + STANDOFF_H, 6.0)
        floor -= cyl_at(x, y, z_fb - 1, Z_FLOOR_TOP + STANDOFF_H + 1, 2.4)
        with BuildSketch() as hx:
            RegularPolygon((4.0 + 0.3) / 2 / 0.8660254, 6)
        floor -= Pos(x, y, z_fb - 0.01) * extrude(hx.sketch, 1.9)

export_step(Compound([box, floor]), CAD / "step" / "05_base.step")
box_p = Rot(180, 0, 0) * box                       # deck on the bed
bb = box_p.bounding_box(); box_p = Pos(-bb.center().X, -bb.center().Y, -bb.min.Z) * box_p
bb = floor.bounding_box(); floor_p = Pos(-bb.center().X, -bb.center().Y, -bb.min.Z) * floor
export_stl(box_p, CAD / "stl" / "05_base_box.stl")
export_stl(floor_p, CAD / "stl" / "05_base_floor.stl")
for n, p in (("base_box", box_p), ("base_floor", floor_p)):
    b = p.bounding_box(); print(f"{n}: {b.size.X:.1f} x {b.size.Y:.1f} x {b.size.Z:.1f}  ({p.volume/1000*1.24:.0f} g solid)")
