#!/usr/bin/env python3
# plot_render_stats.py -- render-budget (VRAM/timing) + stereo anaglyph for docs.
#   python3 scripts/plot_render_stats.py
import os, json
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from PIL import Image

DATA = "/DataDrive/vslam_degradation_benchmark/data/clean"
FIG = "/DataDrive/vslam_degradation_benchmark/docs/figures"
os.makedirs(FIG, exist_ok=True)

meta_path = os.path.join(DATA, "render_meta_smoke.json")
if not os.path.exists(meta_path):
    meta_path = os.path.join(DATA, "render_meta_full.json")
meta = json.load(open(meta_path))
vram = meta["vram_mib"]

order = ["after_app_start", "after_scene_load", "after_warmup", "after_render"]
labels = ["app start", "scene load", "warmup", "render"]
vals = [vram[k] for k in order]

fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 4.5))
bars = ax1.bar(labels, vals, color="#4C78A8")
ax1.axhline(8188, color="crimson", ls="--", label="8188 MiB (RTX 4060 total)")
ax1.set_ylabel("VRAM used (MiB)"); ax1.set_ylim(0, 8500)
ax1.set_title("VRAM by stage — peak %d MiB (%.0f%% of 8 GB)" % (max(vals), 100*max(vals)/8188))
for b, v in zip(bars, vals):
    ax1.text(b.get_x()+b.get_width()/2, v+120, str(v), ha="center", fontsize=9)
ax1.legend(); ax1.grid(axis="y", alpha=0.3)

ax2.axis("off")
txt = (f"Isaac Sim:        {meta['isaac_sim']}\n"
       f"Resolution:       {meta['resolution'][0]}x{meta['resolution'][1]} stereo @ {meta['fps']} Hz\n"
       f"Frames (this run):{meta['frames_captured']}  (full set: {meta['frames_full']})\n"
       f"Mean render:      {meta['mean_ms_per_frame']:.0f} ms/frame\n"
       f"Projected full:   {meta['mean_ms_per_frame']*meta['frames_full']/1000/60:.1f} min\n"
       f"meters_per_unit:  {meta['meters_per_unit']}   up_axis: {meta['up_axis']}\n"
       f"Baseline:         {meta['baseline_m']} m    seed: {meta['seed']}\n"
       f"rt_subframes:     {meta['rt_subframes']}")
ax2.text(0.0, 0.98, txt, va="top", family="monospace", fontsize=11)
ax2.set_title("Phase 1 render budget", loc="left")
plt.tight_layout()
plt.savefig(os.path.join(FIG, "phase1_render_stats.png"), dpi=130)
print("wrote phase1_render_stats.png")

# ---- stereo anaglyph (visual proof of real horizontal disparity) ----
left_dir = os.path.join(DATA, "left")
names = sorted(os.listdir(left_dir)) if os.path.isdir(left_dir) else []
if names:
    nm = names[len(names)//2]
    L = np.array(Image.open(os.path.join(DATA, "left", nm)).convert("RGB"))
    R = np.array(Image.open(os.path.join(DATA, "right", nm)).convert("RGB"))
    ana = R.copy(); ana[:, :, 0] = L[:, :, 0]      # red=left, cyan(G,B)=right
    fig, axs = plt.subplots(1, 3, figsize=(15, 4))
    axs[0].imshow(L); axs[0].set_title("left  " + nm); axs[0].axis("off")
    axs[1].imshow(R); axs[1].set_title("right " + nm); axs[1].axis("off")
    axs[2].imshow(ana); axs[2].set_title("red-cyan anaglyph (fringe = stereo disparity)"); axs[2].axis("off")
    plt.tight_layout()
    plt.savefig(os.path.join(FIG, "phase1_stereo_anaglyph.png"), dpi=120)
    print(f"wrote phase1_stereo_anaglyph.png (frame {nm})")
else:
    print("no frames available for anaglyph (already cleaned?)")
