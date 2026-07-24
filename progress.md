# Progress Log — Visual SLAM Degradation Benchmark

## Phase 0 — Environment audit ✅ (CHECKPOINT 0 passed)
- Detected: Isaac Sim 5.1.0-rc.19, Isaac ROS 3.2.5 (cuVSLAM in `isaac_ros_dev` image),
  ROS 2 Humble, driver 570.195.03 / CUDA 12.8, RTX 4060 Laptop 8 GB, Ubuntu 22.04.5.
- GPU passthrough into Docker verified. `docs/environment.md` written.
- Reviewer decisions: (1) ORB-SLAM3 → build upstream, run stereo offline;
  (2) storage → clean-once + regenerate-per-condition, persist results + docs visuals;
  (3) evo → pinned venv. Resolution 640×480 stereo @ 30 Hz.

## Phase 1 — Scene + trajectory + ground truth 🔄 (in progress)
- Verified exact Isaac Sim 5.1 API from shipped standalone_examples (no guessing):
  `from isaacsim import SimulationApp`; assets via `isaacsim.storage.native.get_assets_root_path`;
  warehouse `/Isaac/Environments/Simple_Warehouse/warehouse.usd`; capture via
  `rep.create.render_product` + `rep.AnnotatorRegistry.get_annotator("rgb")` +
  `rep.orchestrator.step(rt_subframes=…)`.
- Caches redirected to /DataDrive (root fs has <700 MB free): XDG_CACHE_HOME,
  __GL_SHADER_DISK_CACHE_PATH, CUDA_CACHE_PATH, MPLCONFIGDIR.
- Wrote `scripts/render_scene.py`: deterministic figure-eight trajectory
  (x=A·sin t, y=B·sin 2t) → self-intersections give loop closure; forward-looking
  stereo rig, baseline 0.12 m; GT in LEFT-camera optical frame (RDF), TUM format.
- Smoke test PASSED (24 frames spanning full trajectory, exit 0):
  meters_per_unit=1.0, up_axis=Z (assumptions correct); warehouse 24×38.8 m so the
  ±3×±1.5 m figure-eight sits in open space; frames well-lit and feature-rich with
  clear stereo disparity; peak VRAM 2.8 GB; ~970 ms/frame → full ≈ 36 min, ≈1.4 GB.
  GT verified (starts 0,0,1.0; TUM 8-col); calib fx=fy=320 cx=320 cy=240 HFoV 90°.
- OPEN: smoke used DLSS (renders 320×240 internal → upscales to 640×480). Quality looks
  good but for stereo consistency plan to render the full set natively (DLSS off).
- Awaiting CHECKPOINT 1 go-ahead before the full 2250-frame render.
- Generated all Phase-1 figures (trajectory, sample frames, stereo anaglyph, render
  stats). Anaglyph confirms correct depth-dependent horizontal disparity.
- Wrote README.md + .gitignore; initialized git and committed the Phase 0–1 snapshot
  (code + docs + figures + calib/meta). Bulk frames, shader caches, verbose logs are
  gitignored (reproducible). Cleaned regenerable scratch (bench_cache, validation
  frames, __pycache__) afterward.

### Files
- `docs/environment.md` — Phase 0 audit
- `scripts/render_scene.py` — stereo render + TUM GT (config block at top)
- `scripts/plot_trajectory.py` — CHECKPOINT 1 visuals
- `docs/figures/phase1_{trajectory,sample_frames,overview_topdown}.png`
- `data/clean/` — 24-frame validation set (will be overwritten by full render)
