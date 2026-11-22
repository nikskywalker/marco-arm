"""Interference check for the forearm, the wrist and the gripper (with servo dummies).
  1. parts that are bolted together must not overlap;
  2. elbow sweep: forearm + wrist + gripper against link 1 and the elbow servo;
  3. wrist sweep: bracket + gripper against the forearm and the wrist servo.
Angles: elbow 0 = forearm in line with link 1, negative = hand folding toward the front;
wrist 0 = gripper in line with the forearm."""
from pathlib import Path
import itertools, runpy, sys
from build123d import *

HERE = Path(__file__).resolve().parent
f = runpy.run_path(str(HERE / "40_forearm.py"), run_name="lib")
l1 = runpy.run_path(str(HERE / "30_link1.py"), run_name="lib")
g = runpy.run_path(str(HERE / "50_gripper.py"), run_name="lib")
bx, cx, L1, L2 = f["bx"], f["cx"], f["L1"], f["L2"]

def mz996():
    """MZ996 dummy (same as in check_assembly.py): shaft along +Z at the origin, tab top face at Z=0."""
    body = Pos(-9.75 + 40.4 / 2, 0, (-28 + 10) / 2) * Box(40.4, 20.0, 38.0)
    tabs = Pos(-9.75 + 40.4 / 2, 0, -1.25) * Box(40.4 + 14, 20.0, 2.5)
    return body + tabs

# link 1 in its own frame (shoulder axis at the origin, link along +Y)
EL = Pos(l1["x_horn_in"] + 2.5, L1, 0) * Rot(-90, 0, 0) * Rot(0, 90, 0)
link = {"leg_horn": l1["leg_horn"], "leg_pin": l1["leg_pin"], "elbow_servo": EL * mz996()}
for i, y in enumerate(l1["SPACER_Y"]):
    link[f"spacer_{i+1}"] = Pos((l1["x_horn_in"] + l1["x_pin_in"]) / 2, y, 0) * l1["spacer"]

# wrist SG90 dummy in the forearm frame
yc, xb = f["yc"], f["x_sg_bot"]
sg = bx(xb - 22.7, xb, yc - 11.25, yc + 11.25, -6.1, 6.1) + bx(f["x_tab_bot"] - 2.5, f["x_tab_bot"], yc - 16.1, yc + 16.1, -6.1, 6.1)
sg += cx(xb - 26.7, xb - 22.7, L2, 0, 11.8) + cx(f["x_shaft_top"], xb - 26.7, L2, 0, 4.8)
sg = Pos(f["SG_REAL_DX"], 0, 0) * sg                 # 16/11: servo where it really sits (measured)
forearm = {"fa_plate": f["fa_plate"], "fa_body": f["fa_body"], "wrist_servo": sg}

# hand: bracket + gripper (half open) + gripper servo dummy, in the forearm frame at wrist angle 0
T = f["GRIP_T"]
gs = g["box_at"](g["body_cx"] - 11.25, g["body_cx"] + 11.25, -6.1, 6.1, -g["PALM_T"] - 19.9, -g["PALM_T"] + 2.8)
gs += g["box_at"](g["body_cx"] - 16.1, g["body_cx"] + 16.1, -6.1, 6.1, -g["PALM_T"] - 2.5, -g["PALM_T"])
hand = {"wrist_bracket": f["wrist_bracket"], "grip_frame": T * g["frame"], "grip_cover": T * g["cover"],
        "grip_servo": T * gs, "finger_R": T * g["finger_at"](20, 1), "finger_L": T * g["finger_at"](20, -1)}

def hand_at(q3):
    W = Pos(0, L2, 0) * Rot(q3, 0, 0) * Pos(0, -L2, 0)
    return {k: W * v for k, v in hand.items()}

def vol(a, b):
    try:
        return (a & b).volume
    except Exception:
        return 0.0

TOL = 1.0
problems = []
def check(tag, A, B, skip=()):
    for (na, a), (nb, b) in itertools.product(A.items(), B.items()):
        if na == nb or {na, nb} in skip:
            continue
        v = vol(a, b)
        if v > TOL:
            problems.append(f"COLLISION [{tag}] {na} x {nb}: {v:.0f} mm3")

mode = sys.argv[1] if len(sys.argv) > 1 else ("none" if __name__ == "render" else "all")
if mode in ("all", "static"):
    h0 = hand_at(0)
    check("static", forearm, forearm)
    check("static", h0, h0, skip=({"grip_frame", "grip_servo"},))
    check("static", h0, forearm)
if mode in ("all", "assembly"):
    # 16/11: the bracket (with the gripper) must slide ASM_SHIFT toward arm A to take the horn onto the shaft
    for s in (1.0, 2.0, 3.0, 4.0):
        S = Pos(-s, 0, 0)
        check(f"assembly shift -{s:.0f}", {k: S * v for k, v in hand_at(0).items()},
              {"fa_plate": f["fa_plate"], "fa_body": f["fa_body"], "wrist_servo": sg})
if mode in ("all", "wrist"):
    for q3 in (-110, -90, -45, 45, 90, 110):
        check(f"wrist {q3:+d}", hand_at(q3), forearm)
if mode in ("all", "elbow"):
    for q2 in (-80, -60, -40, -10, 30, 72):   # the elbow range of the arm (servo command 10-170)
        E = Pos(0, L1, 0) * Rot(q2, 0, 0)
        moving = {k: E * v for k, v in {**forearm, **hand_at(0)}.items()}
        check(f"elbow {q2:+d}", moving, link)
print("\n".join(problems) or f"no collisions ({mode})")
