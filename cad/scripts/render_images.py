"""Quick matplotlib renders of the assembly (cad/img/)."""
from pathlib import Path
import runpy
import numpy as np
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from build123d import *

HERE = Path(__file__).resolve().parent
IMG = HERE.parent / "img"
IMG.mkdir(exist_ok=True)
g = runpy.run_path(str(HERE / "check_assembly.py"), run_name="render")
F, link = g["fixed"], g["link_at"](30)
C = {"printed": "#7fc8b8", "printed2": "#a99be8", "plate": "#d9d9d9", "bought": "#3b3b3b",
     "board": "#3a9a5b", "pca": "#c95050", "lazy": "#9a9a9a"}

def draw(items, path, lim, view=(22, -50), labels=()):
    fig = plt.figure(figsize=(10, 7)); ax = fig.add_subplot(projection="3d")
    for shape, color, alpha in items:
        v, t = shape.tessellate(0.6)
        V = np.array([(q.X, q.Y, q.Z) for q in v])
        ax.add_collection3d(Poly3DCollection(V[np.array(t)], facecolor=color, edgecolor="none", alpha=alpha))
    (x0, x1), (y0, y1), (z0, z1) = lim
    ax.set_xlim(x0, x1); ax.set_ylim(y0, y1); ax.set_zlim(z0, z1)
    ax.set_box_aspect((x1 - x0, y1 - y0, z1 - z0)); ax.view_init(*view); ax.set_axis_off()
    for (x, y, z), txt in labels:
        ax.text(x, y, z, txt, fontsize=10, color="#222")
    plt.tight_layout(); plt.savefig(path, dpi=100); plt.close(fig)

# 1. whole arm
draw([(F["base_box"], C["printed"], .45), (F["base_floor"], C["printed"], .6), (F["lazy_susan"], C["lazy"], .6),
      (F["yaw_servo"], C["bought"], 1), (F["uno"], C["board"], 1), (F["pca9685"], C["pca"], 1),
      (F["plate"], C["plate"], .8), (F["upright_servo"], C["printed2"], 1), (F["upright_bearing"], C["printed2"], 1),
      (F["shoulder_servo"], C["bought"], 1)] + [(s, C["bought"] if k.startswith("elbow") else C["printed2"], 1) for k, s in link.items()],
     IMG / "01_overview.png", ((-110, 90), (-80, 200), (-70, 150)),
     labels=[((-100, -80, -40), "base"), ((60, 0, 0), "plate"), ((30, 0, 60), "shoulder"), ((0, 140, 130), "link 1 + elbow servo")])
# 2. base inside (no deck: floor + boards + yaw servo + pads seen from above)
draw([(F["base_floor"], C["printed"], .3), (F["uno"], C["board"], 1), (F["pca9685"], C["pca"], 1),
      (F["yaw_servo"], C["bought"], 1)],
     IMG / "02_base_inside.png", ((-110, 80), (-80, 80), (-70, 0)), view=(55, -60),
     labels=[((-80, -40, -45), "Arduino Uno"), ((-80, 55, -45), "PCA9685"), ((20, 0, -20), "yaw servo"),
             ((60, -60, -62), "clamp tab"), ((-60, -80, -55), "cable exit")])
# 3. turntable stack: lazy susan + plate + uprights + shoulder servo
draw([(F["lazy_susan"], C["lazy"], .8), (F["plate"], C["plate"], .9), (F["upright_servo"], C["printed2"], 1),
      (F["upright_bearing"], C["printed2"], 1), (F["shoulder_servo"], C["bought"], 1), (F["yaw_horn"], C["bought"], 1)],
     IMG / "03_turntable_shoulder.png", ((-90, 90), (-90, 90), (-30, 80)),
     labels=[((-75, 0, 70), "shoulder servo"), ((35, 0, 70), "bearing upright"), ((80, 0, -5), "plate"), ((60, -60, -20), "lazy susan")])
# 4. link 1
L0 = g["link_at"](0)
draw([(s, C["bought"] if k.startswith("elbow") else C["printed2"], 1) for k, s in L0.items()],
     IMG / "04_link1.png", ((-60, 60), (-20, 180), (0, 90)), view=(28, -40),
     labels=[((-60, 0, 75), "leg_horn (to shoulder horn)"), ((35, 0, 75), "leg_pin (M5 pin)"), ((0, 67, 75), "spacers"), ((-60, 140, 75), "elbow servo")])
print("images in", IMG)

# 5. gripper (half open) with the SG90 dummy
gr = runpy.run_path(str(HERE / "check_gripper.py"), run_name="render")
gg = gr["g"]
draw([(gg["frame"], C["printed"], .9), (gg["cover"], C["printed"], .35), (gg["pinion_at"](20), C["printed2"], 1),
      (gg["rack_at"](20, 1), C["printed2"], 1), (gg["rack_at"](20, -1), C["printed2"], 1),
      (gg["finger_at"](20, 1), C["pca"], 1), (gg["finger_at"](20, -1), C["pca"], 1),
      (gr["sg_body"] + gr["sg_tabs"], C["bought"], 1)],
     IMG / "05_gripper.png", ((-40, 40), (-30, 30), (-25, 50)), view=(25, -55),
     labels=[((-40, -30, 45), "fingers"), ((30, 25, 20), "cover"), ((30, -30, 5), "frame"), ((15, 0, -25), "SG90")])

# 6. forearm + wrist + gripper (forearm frame), 7. whole arm from the side
fo = runpy.run_path(str(HERE / "check_forearm.py"), run_name="render", init_globals={"__file__": str(HERE / "check_forearm.py")})
FA, HAND = fo["forearm"], fo["hand_at"](0)
fa_items = [(FA["fa_body"], C["printed"], 1), (FA["fa_plate"], C["printed2"], .9), (FA["wrist_servo"], C["bought"], 1),
            (HAND["wrist_bracket"], C["pca"], .9), (HAND["grip_frame"], C["plate"], .9), (HAND["grip_cover"], C["plate"], .5),
            (HAND["grip_servo"], C["bought"], 1), (HAND["finger_R"], C["printed2"], 1), (HAND["finger_L"], C["printed2"], 1)]
draw(fa_items, IMG / "06_forearm.png", ((-50, 50), (-20, 240), (-40, 40)), view=(28, -150),
     labels=[((-10, 10, 28), "fa_plate (to elbow horn)"), ((0, 95, 22), "fa_body"), ((-25, 125, 30), "wrist SG90"),
             ((-30, 165, -38), "wrist bracket"), ((0, 235, 35), "gripper")])
SH, EL = 60, -90
T1 = Pos(0, 0, g["AXIS_H"]) * Rot(SH, 0, 0)
T2 = T1 * Pos(0, fo["L1"], 0) * Rot(EL, 0, 0)
arm = [(F["plate"], C["plate"], .8), (F["upright_servo"], C["printed2"], 1), (F["upright_bearing"], C["printed2"], 1),
       (F["shoulder_servo"], C["bought"], 1)]
arm += [(T1 * s, C["bought"] if "servo" in k else C["printed2"], 1) for k, s in fo["link"].items()]
arm += [(T2 * s, c, a) for s, c, a in fa_items]
draw(arm, IMG / "07_arm_side.png", ((-90, 90), (-80, 260), (-10, 220)), view=(8, -12),
     labels=[((0, -70, 50), "shoulder"), ((0, 40, 180), "elbow"), ((0, 220, 150), "wrist")])
