"""Every printed part must be ONE solid: a floating piece (not attached to the rest) would print in
mid-air and do nothing in the arm. Run after changing any part."""
import runpy
from pathlib import Path
HERE = Path(__file__).resolve().parent
parts = {
    "05_base.py": ["box", "floor"], "06_desk_clamp.py": ["part"], "10_turntable_plate.py": ["part"],
    "20_shoulder_uprights.py": ["servo", "bear"], "30_link1.py": ["leg_horn", "leg_pin", "spacer"],
    "50_gripper.py": ["frame", "cover", "pinion", "rack", "finger"],
}
bad = 0
for script, names in parts.items():
    g = runpy.run_path(str(HERE / script), run_name="check")
    for n in names:
        obj = g[n]
        obj = getattr(obj, "part", obj)
        k = len(obj.solids())
        flag = "OK" if k == 1 else "FLOATING PIECES"
        bad += k != 1
        print(f"{script:28s} {n:10s} solids: {k}  {flag}")
print("all parts in one piece" if not bad else f"{bad} part(s) to fix")
