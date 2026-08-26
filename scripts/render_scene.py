#!/usr/bin/env python3
# =============================================================================
# render_scene.py  --  Phase 1: deterministic stereo render + TUM ground truth
# Run headless with Isaac Sim's python:
#   /DataDrive/isaac-sim/python.sh scripts/render_scene.py              # full
#   /DataDrive/isaac-sim/python.sh scripts/render_scene.py --smoke      # ~24 frames + overview
# All tunables live in CONFIG below (project rule 5: one visible config block).
# =============================================================================
import argparse, json, os, subprocess, sys, time
import numpy as np

# ------------------------------- CONFIG --------------------------------------
CONFIG = {
    # scene
    "warehouse_url":  "/Isaac/Environments/Simple_Warehouse/warehouse.usd",
    "env_prim":       "/World/Warehouse",
    # stereo camera intrinsics  (fx=fy=320, cx=320, cy=240, HFoV=90 deg at 640x480)
    "width":          640,
    "height":         480,
    "focal_length":   12.0,     # mm (ratio with aperture sets FoV; absolute value irrelevant)
    "h_aperture":     24.0,     # -> HFoV = 2*atan(24/(2*12)) = 90 deg
    "v_aperture":     18.0,     # -> square pixels for 4:3 (24/640 == 18/480)
    "baseline_m":     0.12,     # stereo baseline, left is reference, right is +baseline
    "near_far_m":     (0.05, 1000.0),
    # trajectory: planar figure-eight, x=A sin(t), y=B sin(2t); self-intersections
    # at the centre give genuine loop-closure revisits. Camera looks along motion.
    "center_m":       (0.0, 0.0),
    "amp_x_m":        3.0,
    "amp_y_m":        1.5,
    "height_m":       1.0,      # camera height above floor
    "duration_s":     75.0,
    "fps":            30,
    # render quality
    "rt_subframes":   32,       # RTX subframes accumulated per still frame (anti-ghost)
    "warmup_frames":  40,       # app.update() calls before first capture (load textures)
    # output
    "out_dir":        "/DataDrive/vslam_degradation_benchmark/data/clean",
    "seed":           0,
    # smoke test
    "smoke_frames":   24,       # spread across the WHOLE trajectory (not just the start)
    "overview_height_m": 7.0,   # top-down inspection camera; MUST stay below the
                                #   warehouse roof (~8 m) or it images the roof
}
# -----------------------------------------------------------------------------


def log(msg):
    print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)


def vram_used_mib():
    try:
        out = subprocess.check_output(
            ["nvidia-smi", "--query-gpu=memory.used,memory.total",
             "--format=csv,noheader,nounits"], text=True).strip().splitlines()[0]
        used, total = [int(x) for x in out.split(",")]
        return used, total
    except Exception:
        return -1, -1


def log_vram(tag, sink):
    u, t = vram_used_mib()
    sink[tag] = u
    log(f"VRAM [{tag}]: {u} / {t} MiB used")


def mat_to_quat_wxyz(R):
    """Rotation matrix (world<-local, column-vector) -> quaternion (w,x,y,z)."""
    m00, m11, m22 = R[0, 0], R[1, 1], R[2, 2]
    tr = m00 + m11 + m22
    if tr > 0:
        s = np.sqrt(tr + 1.0) * 2
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif m00 > m11 and m00 > m22:
        s = np.sqrt(1.0 + m00 - m11 - m22) * 2
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif m11 > m22:
        s = np.sqrt(1.0 + m11 - m00 - m22) * 2
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = np.sqrt(1.0 + m22 - m00 - m11) * 2
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    q = np.array([w, x, y, z])
    return q / np.linalg.norm(q)


def optical_basis(forward, world_up=np.array([0.0, 0.0, 1.0])):
    """Return R_optical (cols = world dirs of camera x-right, y-down, z-forward)."""
    z = forward / np.linalg.norm(forward)
    x = np.cross(z, world_up)
    nx = np.linalg.norm(x)
    if nx < 1e-9:                       # looking straight up/down: fall back
        x = np.array([1.0, 0.0, 0.0])
    else:
        x = x / nx
    y = np.cross(z, x)                  # x-right, y-down, z-forward (right-handed)
    return np.column_stack([x, y, z])


def trajectory(t, C):
    """Figure-eight pose at time t (s). Returns (pos_m[3], R_opt[3x3], right_axis[3])."""
    w = 2.0 * np.pi * t / C["duration_s"]
    cx, cy = C["center_m"]
    A, B = C["amp_x_m"], C["amp_y_m"]
    pos = np.array([cx + A * np.sin(w), cy + B * np.sin(2 * w), C["height_m"]])
    vel = np.array([A * np.cos(w), 2 * B * np.cos(2 * w), 0.0])   # dp/dw (heading)
    R_opt = optical_basis(vel)
    right_axis = R_opt[:, 0]            # world direction of +baseline (camera right)
    return pos, R_opt, right_axis


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--smoke", action="store_true")
    ap.add_argument("--out", default=CONFIG["out_dir"])
    args = ap.parse_args()
    C = dict(CONFIG); C["out_dir"] = args.out
    np.random.seed(C["seed"])

    out = C["out_dir"]
    os.makedirs(os.path.join(out, "left"), exist_ok=True)
    os.makedirs(os.path.join(out, "right"), exist_ok=True)
    fig_dir = "/DataDrive/vslam_degradation_benchmark/docs/figures"
    os.makedirs(fig_dir, exist_ok=True)

    t_start = time.time()
    vram = {}
    log(f"Launching Isaac Sim (headless). smoke={args.smoke}")

    from isaacsim import SimulationApp
    sim = SimulationApp({"headless": True, "renderer": "RayTracedLighting",
                         "width": C["width"], "height": C["height"]})

    # --- imports that require the app to be running ---
    import omni.replicator.core as rep
    from isaacsim.core.utils.stage import (create_new_stage, add_reference_to_stage,
                                           is_stage_loading, get_stage_units)
    from pxr import UsdGeom, Gf, UsdLux
    import omni.usd

    log_vram("after_app_start", vram)

    from isaacsim.storage.native import get_assets_root_path
    assets_root = get_assets_root_path()
    if not assets_root:
        log("FAILED: get_assets_root_path() returned None (asset server unreachable).")
        sim.close(); sys.exit(2)
    warehouse = assets_root + C["warehouse_url"]
    log(f"Assets root: {assets_root}")
    log(f"Loading warehouse: {warehouse}")

    create_new_stage()
    add_reference_to_stage(usd_path=warehouse, prim_path=C["env_prim"])
    # wait for async load
    for _ in range(500):
        if not is_stage_loading():
            break
        sim.update()
    stage = omni.usd.get_context().get_stage()
    units = get_stage_units()          # meters per stage unit
    up = UsdGeom.GetStageUpAxis(stage)
    m2u = 1.0 / units                  # meters -> stage units
    log(f"Stage loaded. meters_per_unit={units}  up_axis={up}")
    log_vram("after_scene_load", vram)

    # --- scene bounding box (informs trajectory sizing) ---
    try:
        cache = UsdGeom.BBoxCache(0, [UsdGeom.Tokens.default_])
        bb = cache.ComputeWorldBound(stage.GetPrimAtPath(C["env_prim"])).ComputeAlignedRange()
        mn, mx = bb.GetMin(), bb.GetMax()
        log(f"Warehouse world bbox (stage units): min=({mn[0]:.2f},{mn[1]:.2f},{mn[2]:.2f}) "
            f"max=({mx[0]:.2f},{mx[1]:.2f},{mx[2]:.2f})")
        log(f"Warehouse extent (meters): "
            f"{(mx[0]-mn[0])*units:.2f} x {(mx[1]-mn[1])*units:.2f} x {(mx[2]-mn[2])*units:.2f}")
    except Exception as e:
        log(f"bbox computation skipped: {e}")

    # --- stereo cameras ---
    def make_cam(path):
        cam = UsdGeom.Camera.Define(stage, path)
        cam.CreateFocalLengthAttr(C["focal_length"])
        cam.CreateHorizontalApertureAttr(C["h_aperture"])
        cam.CreateVerticalApertureAttr(C["v_aperture"])
        near, far = C["near_far_m"]
        cam.CreateClippingRangeAttr(Gf.Vec2f(near * m2u, far * m2u))
        xf = UsdGeom.Xformable(cam.GetPrim())
        xf.ClearXformOpOrder()
        op = xf.AddTransformOp()
        return cam.GetPrim(), op

    _, op_left = make_cam("/World/cam_left")
    _, op_right = make_cam("/World/cam_right")

    def set_cam(op, pos_m, R_opt):
        R_usd = R_opt @ np.diag([1.0, -1.0, -1.0])       # optical(RDF) -> USD cam(RUB)
        q = mat_to_quat_wxyz(R_usd)
        M = Gf.Matrix4d()
        M.SetRotate(Gf.Rotation(Gf.Quatd(float(q[0]), Gf.Vec3d(float(q[1]), float(q[2]), float(q[3])))))
        M.SetTranslateOnly(Gf.Vec3d(float(pos_m[0] * m2u), float(pos_m[1] * m2u), float(pos_m[2] * m2u)))
        op.Set(M)

    # --- render products + rgb annotators ---
    rp_left = rep.create.render_product("/World/cam_left", (C["width"], C["height"]))
    rp_right = rep.create.render_product("/World/cam_right", (C["width"], C["height"]))
    ann_left = rep.AnnotatorRegistry.get_annotator("rgb")
    ann_right = rep.AnnotatorRegistry.get_annotator("rgb")
    ann_left.attach(rp_left)
    ann_right.attach(rp_right)

    # image saver (PIL preferred, imageio fallback)
    try:
        from PIL import Image
        def save_png(path, arr):
            arr = np.asarray(arr)
            Image.fromarray(arr[:, :, :3].astype(np.uint8)).save(path)
        saver = "PIL"
    except Exception:
        import imageio.v2 as imageio
        def save_png(path, arr):
            arr = np.asarray(arr)
            imageio.imwrite(path, arr[:, :, :3].astype(np.uint8))
        saver = "imageio"
    log(f"Image saver: {saver}")

    # --- warm up renderer (load materials/textures before capture) ---
    p0, R0, _ = trajectory(0.0, C)
    set_cam(op_left, p0, R0)
    set_cam(op_right, p0 + C["baseline_m"] * trajectory(0.0, C)[2], R0)
    for _ in range(C["warmup_frames"]):
        sim.update()
    rep.orchestrator.step(rt_subframes=C["rt_subframes"], delta_time=0.0, pause_timeline=False)
    log_vram("after_warmup", vram)

    # optional top-down overview for scene inspection (smoke only)
    if args.smoke:
        ov = UsdGeom.Camera.Define(stage, "/World/cam_overview")
        ov.CreateFocalLengthAttr(8.0); ov.CreateHorizontalApertureAttr(30.0); ov.CreateVerticalApertureAttr(30.0)
        ov.CreateClippingRangeAttr(Gf.Vec2f(0.1 * m2u, 2000.0 * m2u))
        xf = UsdGeom.Xformable(ov.GetPrim()); xf.ClearXformOpOrder(); ov_op = xf.AddTransformOp()
        cx, cy = C["center_m"]
        # A USD camera looks down its own -Z. The stage is Z-up, so an identity
        # rotation already points it straight down; only translation is needed.
        #
        # Height matters: the Simple Warehouse is an INDOOR scene with a solid
        # roof at roughly 8 m. The previous value of 18 m put the camera outside
        # the building, so every capture was the unlit corrugated roof from above
        # -- a black frame of horizontal stripes. Stay under the ceiling.
        Md = Gf.Matrix4d()
        Md.SetTranslateOnly(Gf.Vec3d(cx * m2u, cy * m2u, C["overview_height_m"] * m2u))
        ov_op.Set(Md)
        rp_ov = rep.create.render_product("/World/cam_overview", (720, 720))
        ann_ov = rep.AnnotatorRegistry.get_annotator("rgb"); ann_ov.attach(rp_ov)
        rep.orchestrator.step(rt_subframes=C["rt_subframes"], delta_time=0.0, pause_timeline=False)
        save_png(os.path.join(fig_dir, "phase1_overview_topdown.png"), ann_ov.get_data())
        log("Saved top-down overview -> docs/figures/phase1_overview_topdown.png")
        ann_ov.detach()

    # --- main capture loop ---
    n_full = int(round(C["duration_s"] * C["fps"]))
    if args.smoke:
        idxs = np.linspace(0, n_full - 1, C["smoke_frames"]).astype(int)
    else:
        idxs = np.arange(n_full)
    log(f"Capturing {len(idxs)} stereo frames (full trajectory has {n_full} frames)")

    gt_lines, time_lines = [], []
    frame_times = []
    for k, i in enumerate(idxs):
        t = i / C["fps"]
        pos, R_opt, right = trajectory(t, C)
        set_cam(op_left, pos, R_opt)
        set_cam(op_right, pos + C["baseline_m"] * right, R_opt)
        f0 = time.time()
        rep.orchestrator.step(rt_subframes=C["rt_subframes"], delta_time=0.0, pause_timeline=False)
        L = ann_left.get_data(); Rr = ann_right.get_data()
        name = f"{i:06d}.png"
        save_png(os.path.join(out, "left", name), L)
        save_png(os.path.join(out, "right", name), Rr)
        frame_times.append(time.time() - f0)
        # GT = LEFT camera optical-frame pose in world, meters, TUM: t tx ty tz qx qy qz qw
        q = mat_to_quat_wxyz(R_opt)     # (w,x,y,z)
        gt_lines.append(f"{t:.6f} {pos[0]:.6f} {pos[1]:.6f} {pos[2]:.6f} "
                        f"{q[1]:.6f} {q[2]:.6f} {q[3]:.6f} {q[0]:.6f}")
        time_lines.append(f"{t:.6f}")
        if k % 25 == 0 or k == len(idxs) - 1:
            log(f"  frame {k+1}/{len(idxs)} (idx {i:06d}, t={t:.2f}s) "
                f"mean {np.mean(frame_times)*1000:.0f} ms/frame")
            if k % 100 == 0:
                root_free = subprocess.check_output(["df", "--output=avail", "-m", "/"], text=True).splitlines()[-1].strip()
                log(f"  root fs free: {root_free} MiB")

    log_vram("after_render", vram)

    # --- write ground truth + metadata ---
    with open(os.path.join(out, "groundtruth_tum.txt"), "w") as f:
        f.write("# ground truth trajectory (LEFT camera optical frame: x-right y-down z-forward)\n")
        f.write("# timestamp tx ty tz qx qy qz qw   (meters, TUM format)\n")
        f.write("\n".join(gt_lines) + "\n")
    with open(os.path.join(out, "times.txt"), "w") as f:
        f.write("\n".join(time_lines) + "\n")

    fx = C["focal_length"] * C["width"] / C["h_aperture"]
    fy = C["focal_length"] * C["height"] / C["v_aperture"]
    calib = {
        "model": "pinhole", "width": C["width"], "height": C["height"],
        "fx": fx, "fy": fy, "cx": C["width"] / 2.0, "cy": C["height"] / 2.0,
        "distortion": [0, 0, 0, 0, 0], "baseline_m": C["baseline_m"], "fps": C["fps"],
        "hfov_deg": float(np.degrees(2 * np.arctan(C["h_aperture"] / (2 * C["focal_length"])))),
        "camera_frame": "optical RDF (x-right, y-down, z-forward)",
        "left_is_reference": True,
    }
    with open(os.path.join(out, "calib.yaml"), "w") as f:
        for k, v in calib.items():
            f.write(f"{k}: {v}\n")

    meta = {
        "isaac_sim": open("/DataDrive/isaac-sim/VERSION").read().strip(),
        "smoke": args.smoke, "frames_captured": len(idxs), "frames_full": n_full,
        "resolution": [C["width"], C["height"]], "fps": C["fps"],
        "duration_s": C["duration_s"], "baseline_m": C["baseline_m"], "seed": C["seed"],
        "rt_subframes": C["rt_subframes"], "meters_per_unit": units, "up_axis": up,
        "vram_mib": vram, "mean_ms_per_frame": float(np.mean(frame_times) * 1000),
        "wall_clock_s": time.time() - t_start, "image_saver": saver,
        "intrinsics": calib,
    }
    tag = "smoke" if args.smoke else "full"
    with open(os.path.join(out, f"render_meta_{tag}.json"), "w") as f:
        json.dump(meta, f, indent=2)

    log(f"DONE ({tag}): {len(idxs)} stereo pairs, "
        f"{meta['mean_ms_per_frame']:.0f} ms/frame, wall {meta['wall_clock_s']:.1f}s")
    log(f"Peak VRAM during render: {vram.get('after_render','?')} MiB")
    sim.close()


if __name__ == "__main__":
    main()
