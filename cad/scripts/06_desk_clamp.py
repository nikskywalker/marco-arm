"""Printed C-clamp for the base: holds one floor tab against the desk (print 2).
The upper jaw sits on the tab, the M5 bolt in the lower jaw pushes up against the desk underside.
Profile drawn in the XZ plane (X = toward the desk edge... spine beyond the edge), extruded along Y."""
from pathlib import Path
from build123d import *

CAD = Path(__file__).resolve().parents[1]

DESK_T = 27.0          # measured: 26-27 mm
TAB_T = 5.0            # floor tab thickness
BOLT_GAP = 6.0         # lower jaw top -> desk underside; M5 x 20 sticks out ~10 above the jaw
ARM_T = 10.0           # jaw thickness
SPINE_T = 12.0
REACH = 18.0           # how far the jaws reach in from the spine (tab is 20 long)
WIDTH = 25.0           # along Y (tab is 30 wide)
M5_D, M5_NUT_AF, M5_NUT_H = 5.5, 8.1, 4.5

throat = TAB_T + DESK_T + BOLT_GAP
H = throat + 2 * ARM_T
with BuildPart() as clamp:
    with BuildSketch(Plane.XZ):
        Rectangle(SPINE_T + REACH, H, align=(Align.MIN, Align.MIN))
        with Locations((0, ARM_T)):
            Rectangle(REACH, throat, align=(Align.MIN, Align.MIN), mode=Mode.SUBTRACT)
    extrude(amount=WIDTH / 2, both=True)
    # fillets inside the C corners (stress)
    inner = clamp.edges().filter_by(Axis.Y).filter_by(
        lambda e: abs(e.center().X - REACH) < 1e-6 and ARM_T - 1e-6 <= e.center().Z <= ARM_T + throat + 1e-6)
    fillet(inner, 3.0)
    # M5 bolt through the lower jaw, nut trapped on the jaw top (the nut is pushed down onto the pocket floor)
    bx = REACH / 2
    with Locations((bx, 0, ARM_T / 2)):
        Cylinder(M5_D / 2, ARM_T * 2, mode=Mode.SUBTRACT)

part = clamp.part
with BuildSketch() as hexs:
    RegularPolygon(M5_NUT_AF / 2 / 0.8660254, 6)
part -= Pos(bx, 0, ARM_T - M5_NUT_H) * extrude(hexs.sketch, M5_NUT_H + 0.1)
# print lying on its side: the C profile flat on the bed (layers in the plane of the C)
p = Rot(90, 0, 0) * part
bb = p.bounding_box(); p = Pos(-bb.center().X, -bb.center().Y, -bb.min.Z) * p
export_stl(p, CAD / "stl" / "06_desk_clamp.stl")
export_step(part, CAD / "step" / "06_desk_clamp.step")
b = p.bounding_box()
print(f"clamp: {b.size.X:.1f} x {b.size.Y:.1f} x {b.size.Z:.1f}, throat {throat:.0f} mm, {p.volume/1000*1.24:.0f} g solid")
