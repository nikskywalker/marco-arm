"""Involute spur gear and matching rack outlines (2D faces) for printed parts.
Backlash is NOT built into the teeth: it is set by moving the rack/gear 0.2 mm further away."""
import math
from build123d import *

def _inv(a):
    return math.tan(a) - a

def spur_gear_face(module, teeth, pressure_angle=20.0, steps=8):
    """Gear outline in the XY plane, centered at the origin, one tooth centered on +X."""
    pa = math.radians(pressure_angle)
    rp = module * teeth / 2
    rb = rp * math.cos(pa)
    ra = rp + module
    rf = rp - 1.25 * module
    half_p = math.pi / (2 * teeth) + _inv(pa)           # half angle of the tooth seen from the base circle

    def half(r):
        r = max(r, rb)
        return half_p - _inv(math.acos(rb / r))

    r0 = max(rb, rf)
    radii = [r0 + (ra - r0) * i / steps for i in range(steps + 1)]
    pts = []
    for k in range(teeth):
        c = 2 * math.pi * k / teeth
        if rf < rb:                                       # radial flank below the base circle
            pts.append((rf * math.cos(c - half(rb)), rf * math.sin(c - half(rb))))
        for r in radii:                                   # right flank, root -> tip
            a = c - half(r); pts.append((r * math.cos(a), r * math.sin(a)))
        for r in reversed(radii):                         # left flank, tip -> root
            a = c + half(r); pts.append((r * math.cos(a), r * math.sin(a)))
        if rf < rb:
            pts.append((rf * math.cos(c + half(rb)), rf * math.sin(c + half(rb))))
    with BuildSketch() as s:
        with BuildLine():
            Polyline(*pts, close=True)
        make_face()
    return s.sketch

def rack_face(module, x0, x1, pitch_y, back_y, pressure_angle=20.0, phase=0.0):
    """Rack along X between x0 and x1, teeth pointing toward -Y, pitch line at pitch_y,
    flat back at back_y (> pitch_y). Tooth centers at phase + k * pitch."""
    pa = math.radians(pressure_angle)
    p = math.pi * module
    ht, hr = module, 1.25 * module
    w_tip = p / 4 - ht * math.tan(pa)                     # half widths
    w_root = p / 4 + hr * math.tan(pa)
    y_tip, y_root = pitch_y - ht, pitch_y + hr
    k0 = math.floor((x0 - phase) / p) - 1
    k1 = math.ceil((x1 - phase) / p) + 1
    pts = [(x0, back_y)]
    prof = []
    for k in range(k0, k1 + 1):
        xc = phase + k * p
        prof += [(xc - w_root, y_root), (xc - w_tip, y_tip), (xc + w_tip, y_tip), (xc + w_root, y_root)]
    with BuildSketch() as teeth:
        with BuildLine():
            Polyline(*([(prof[0][0], back_y)] + prof + [(prof[-1][0], back_y)]), close=True)
        make_face()
    clip = Pos((x0 + x1) / 2, (y_tip + back_y) / 2) * Rectangle(x1 - x0, back_y - y_tip + 2)
    return teeth.sketch & clip
