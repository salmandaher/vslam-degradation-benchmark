#!/usr/bin/env python3
# plot_trajectory.py -- CHECKPOINT 1 visuals from a rendered clean dataset.
#   python3 scripts/plot_trajectory.py [data/clean]
# Produces: docs/figures/phase1_trajectory.png, docs/figures/phase1_sample_frames.png
import os, sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from PIL import Image

DATA = sys.argv[1] if len(sys.argv) > 1 else "/DataDrive/vslam_degradation_benchmark/data/clean"
FIG = "/DataDrive/vslam_degradation_benchmark/docs/figures"
os.makedirs(FIG, exist_ok=True)


def quat_to_R(qx, qy, qz, qw):
    n = np.sqrt(qx*qx+qy*qy+qz*qz+qw*qw)
    qx, qy, qz, qw = qx/n, qy/n, qz/n, qw/n
    return np.array([
        [1-2*(qy*qy+qz*qz), 2*(qx*qy-qz*qw),   2*(qx*qz+qy*qw)],
        [2*(qx*qy+qz*qw),   1-2*(qx*qx+qz*qz), 2*(qy*qz-qx*qw)],
        [2*(qx*qz-qy*qw),   2*(qy*qz+qx*qw),   1-2*(qx*qx+qy*qy)],
    ])


gt = np.loadtxt(os.path.join(DATA, "groundtruth_tum.txt"))
t, xyz, quat = gt[:, 0], gt[:, 1:4], gt[:, 4:8]
P = xyz
seglen = np.linalg.norm(np.diff(P, axis=0), axis=1)
path_len = float(seglen.sum())
print(f"frames={len(t)}  duration={t[-1]-t[0]:.2f}s  path_length={path_len:.2f} m")
print(f"bounds X[{P[:,0].min():.2f},{P[:,0].max():.2f}] "
      f"Y[{P[:,1].min():.2f},{P[:,1].max():.2f}] Z[{P[:,2].min():.2f},{P[:,2].max():.2f}]")

# ---- trajectory figure: top-down (XY) with heading arrows + side (XZ) ----
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 5.4))
sc = ax1.scatter(P[:, 0], P[:, 1], c=t, cmap="viridis", s=8)
ax1.plot(P[:, 0], P[:, 1], "-", lw=0.6, color="gray", alpha=0.5)
step = max(1, len(t)//24)
for i in range(0, len(t), step):
    R = quat_to_R(*quat[i]); f = R[:, 2]  # camera forward (optical z)
    ax1.arrow(P[i, 0], P[i, 1], f[0]*0.35, f[1]*0.35, head_width=0.08,
              color="crimson", alpha=0.8, length_includes_head=True)
ax1.plot(P[0, 0], P[0, 1], "o", ms=12, mfc="lime", mec="k", label="start")
ax1.plot(P[-1, 0], P[-1, 1], "s", ms=11, mfc="red", mec="k", label="end")
ax1.set_aspect("equal"); ax1.set_xlabel("X (m)"); ax1.set_ylabel("Y (m)")
ax1.set_title("Top-down camera trajectory (color=time, red=heading)")
ax1.legend(loc="upper right"); ax1.grid(alpha=0.3)
plt.colorbar(sc, ax=ax1, label="time (s)", shrink=0.85)

ax2.plot(t, P[:, 0], label="X"); ax2.plot(t, P[:, 1], label="Y"); ax2.plot(t, P[:, 2], label="Z")
ax2.set_xlabel("time (s)"); ax2.set_ylabel("position (m)")
ax2.set_title("Position vs time"); ax2.legend(); ax2.grid(alpha=0.3)
plt.tight_layout()
plt.savefig(os.path.join(FIG, "phase1_trajectory.png"), dpi=130)
print("wrote phase1_trajectory.png")

# ---- sample stereo frames montage ----
left_dir = os.path.join(DATA, "left")
names = sorted(os.listdir(left_dir))
if names:
    picks = [names[int(k)] for k in np.linspace(0, len(names)-1, 3)]
    fig, axes = plt.subplots(3, 2, figsize=(9, 9))
    for r, nm in enumerate(picks):
        for c, side in enumerate(["left", "right"]):
            img = np.array(Image.open(os.path.join(DATA, side, nm)))
            axes[r, c].imshow(img); axes[r, c].set_title(f"{side}/{nm}", fontsize=9)
            axes[r, c].axis("off")
    plt.tight_layout()
    plt.savefig(os.path.join(FIG, "phase1_sample_frames.png"), dpi=120)
    print(f"wrote phase1_sample_frames.png (samples: {picks})")
