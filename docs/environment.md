# Environment Audit — Visual SLAM Degradation Benchmark

**Generated:** 2026-07-24 (Phase 0)
**Host working dir for project:** `/DataDrive/vslam_degradation_benchmark`
**Method:** Every value below was read from a live command on this machine
(`nvidia-smi`, `docker run`, `lscpu`, `cat VERSION`, `ros2 pkg list`, etc.).
No value is assumed or carried over from documentation.

---

## 1. Host system

| Item | Value | Source |
|---|---|---|
| OS | Ubuntu 22.04.5 LTS | `/etc/os-release` |
| CPU | AMD Ryzen 7 7435HS — 8 cores / 16 threads | `lscpu` |
| System RAM | 15 GiB total (6.3 GiB free at audit), 2 GiB swap | `free -h` |
| GPU | NVIDIA GeForce RTX 4060 **Laptop** | `nvidia-smi` |
| **VRAM** | **8188 MiB total** (~7.3 GiB free at idle) | `nvidia-smi` |
| NVIDIA driver | 570.195.03 | `nvidia-smi` |
| CUDA runtime (driver) | 12.8 | `nvidia-smi` |
| CUDA toolkit (host) | **not installed** on host PATH (`nvcc` absent) | `nvcc` |

**VRAM is the binding constraint** (8 GB). Isaac Sim rendering and a running
SLAM system will not coexist comfortably in 8 GB, so the pipeline is designed as
**render-to-disk first (Sim closed) → replay through SLAM**. VRAM is logged at
every stage per the project rules.

## 2. ROS 2

| Item | Value | Source |
|---|---|---|
| Distro | **Humble** (only one installed) | `/opt/ros/humble`, `$ROS_DISTRO` |
| Host ROS Python | `/usr/bin/python3` = **3.10.12** | `python3 --version` |
| RMW (workspace default) | `rmw_cyclonedds_cpp` | `isaac_ws/scripts/_env.sh` |

⚠️ **Python environment collision to be aware of:** the login shell has conda
`base` active, whose `python3` is **3.13.12** (miniconda). ROS 2 Humble needs the
**system 3.10**. Any host-side ROS work must run with conda deactivated. The
cleaner path (adopted here) is to run SLAM **inside the Isaac ROS container**,
where the environment is already correct.

## 3. Isaac Sim

| Item | Value | Source |
|---|---|---|
| Version | **5.1.0-rc.19** (`+release.26219.9c81211b.gl`) | `/DataDrive/isaac-sim/VERSION` |
| Install path | `/DataDrive/isaac-sim` | filesystem |
| Launcher | `isaac-sim.sh`, headless via `python.sh` | filesystem |
| ROS 2 bridge | present: `isaacsim.ros2.bridge`, `.sim_control`, `.tf_viewer`, `.urdf` | `exts/` |

Note: this is a **release candidate** build. Viability of headless rendering on
an 8 GB laptop GPU is unproven until Phase 1 — the Phase 1 checkpoint's VRAM +
wall-clock render numbers are the go/no-go evidence for the whole approach.

## 4. Isaac ROS / cuVSLAM

| Item | Value | Source |
|---|---|---|
| Delivery | Docker image `isaac_ros_dev-x86_64:latest` (~20 GB, built 2026-06-25) | `docker images` |
| Isaac ROS version | **3.2.5-0jammy** (apt packages) | `apt list --installed` in image |
| cuVSLAM package | `isaac_ros_visual_slam` **present, baked into image** | `ros2 pkg list` in image |
| nvblox | `isaac_ros_nvblox` + family present (not needed for this project) | `ros2 pkg list` in image |
| Container ROS distro | Humble | image |
| Container Python | **3.10.12** | image |
| Container CUDA toolkit | **12.6** (`cuda_12.6.r12.6`) | `nvcc` in image |
| GPU passthrough | ✅ verified — RTX 4060, 8188 MiB visible with `--gpus all` | `docker run --gpus all … nvidia-smi` |
| NVIDIA container runtime | ✅ available (`nvidia` runtime + CDI `nvidia.com/gpu=all`) | `docker info` |

**cuVSLAM (System-Under-Test #1) is ready.** It runs in the container on the
GPU; no build needed. Driver 570 / CUDA 12.8 host vs CUDA 12.6 toolkit in the
container is a compatible combination.

Existing workspace `/DataDrive/workspaces/isaac_ws` (mounts into the container at
`/workspaces/isaac_ros-dev`) already has working cuVSLAM launch files
(`ev_robot_bringup/launch/visual_slam.launch.py`) we can adapt for dataset replay.

## 5. ORB-SLAM3 (System-Under-Test #2)

| Item | Value | Source |
|---|---|---|
| Found at | `/home/salman/ros2_test/src/ros2_orb_slam3` (v1.5.0, built) | filesystem |
| Executables | `mono_node_cpp`, `mono_driver_node.py` | `install/.../lib` |
| **Stereo support** | ❌ **NONE — this wrapper is monocular only** | source tree, no stereo node |

🔴 **This is the single biggest gap.** The project requires **stereo**
ORB-SLAM3, and the only ORB-SLAM3 on this machine is a monocular ROS 2 wrapper.
It also lives on the root filesystem, which is nearly full (see §6). Options are
laid out in "Decisions needed" below.

## 6. Disk space 🔴

| Filesystem | Size | Free | Use% | Notes |
|---|---|---|---|---|
| `/DataDrive` (`nvme0n1p7`) | 129 G | **21 G** | 83% | Isaac Sim, Isaac ROS, this project |
| `/` (`nvme0n1p8`) | 64 G | **0.67 G** | **99%** | ⚠️ effectively full — no room for data/builds |

**Consequences:**
- All datasets, results, and any new builds **must** live on `/DataDrive`.
- Root `/` (which currently holds the ORB-SLAM3 wrapper) has no headroom; do not
  build or store there.
- **21 GB will not hold all 19 stereo datasets** if materialized as image files.
  Rough estimate at 640×480 PNG, 30 Hz, ~75 s (≈2250 stereo pairs = 4500 images):
  ~2 GB per condition × 19 conditions ≈ **~38 GB** — over budget. At 720p it is
  ~125 GB. This forces a storage strategy decision (below).

## 7. Analysis tooling

| Item | Value |
|---|---|
| `evo` (required for ATE/RPE) | ❌ **not installed** — will be pinned in a dedicated Python 3.10 env |
| Isaac Sim Python | bundled (via `python.sh`) — used only for rendering |
| Container Python 3.10 | used for SLAM |

Three Python environments stay deliberately separate to avoid the conda/ROS
collision: **Sim (bundled)** / **SLAM (container 3.10)** / **analysis (evo env)**.

---

## 8. Readiness summary

| Component | Status |
|---|---|
| Isaac Sim 5.1 (rendering + ground truth) | ✅ installed — headless viability to prove in Phase 1 |
| cuVSLAM stereo (SUT #1) | ✅ ready in container, GPU-verified |
| ORB-SLAM3 stereo (SUT #2) | 🔴 **missing** — only a monocular wrapper exists |
| `evo` metrics | 🟡 not installed — trivial to add (pinned) |
| GPU / container runtime | ✅ working |
| Disk for full dataset | 🔴 21 GB insufficient to materialize all 19 sets |
| VRAM for concurrent Sim+SLAM | 🟡 8 GB — handled by render-then-replay sequencing |

---

## 9. Decisions (raised at CHECKPOINT 0)

1. **ORB-SLAM3 stereo.** The existing wrapper is monocular. Recommended fix:
   build upstream **ORB-SLAM3 (UZ-SLAMLab)** on `/DataDrive` and run its
   **stereo** example binary offline on the rendered frames + timestamps → it
   emits a TUM `KeyFrameTrajectory.txt` that `evo` reads directly. This is more
   reproducible than a ROS wrapper (no bag-replay timing jitter) and still gives
   "same output format" for the comparison. Alternative: find/patch a stereo ROS 2
   wrapper. **Need your call on standalone-binary vs ROS-node.**

2. **Storage strategy.** 21 GB cannot hold all 19 materialized stereo datasets.
   Recommended: **render the clean set once (~2 GB), and for each degraded
   condition regenerate → benchmark all seeds/systems → delete before the next**,
   keeping only `data/clean/` + `results/` permanently. Peak disk stays < 6 GB
   and every run is still reproducible from a fixed seed + trajectory. This
   deviates from "save all degraded datasets to disk," so I want your agreement.
   (Resolution proposal: **640×480 stereo @ 30 Hz** — standard for VSLAM eval and
   friendly to both VRAM and disk.)

3. **evo install** — I'll pin it in a dedicated `python3.10 -m venv` on
   `/DataDrive` and record the exact version. No action needed from you unless
   you prefer conda.

Nothing above blocks starting Phase 1 except decisions **1** and **2**.

### Resolved by reviewer (2026-07-24)

1. **ORB-SLAM3 stereo → build upstream, run offline.** Build UZ-SLAMLab
   ORB-SLAM3 on `/DataDrive`; run its stereo binary on the rendered frames +
   timestamps → TUM trajectory for `evo`. Needed by Phase 4 (does not block 1–3).
2. **Storage → clean-once + regenerate-per-condition, and persist results +
   docs visuals.** Permanent on disk: `data/clean/`, `results/` (per-run logs,
   estimated trajectories, `summary.csv`), and `docs/figures/` (sample frames,
   contact sheets, trajectory + result plots for the GitHub README). Bulk
   degraded frame sets are generated per condition, benchmarked across all
   seeds/systems, have sample frames copied into `docs/figures/`, then are
   deleted before the next condition. Resolution: **640×480 stereo @ 30 Hz**.
3. **evo → pinned in a dedicated Python 3.10 venv on `/DataDrive`.**
