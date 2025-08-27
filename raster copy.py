
"""
Open3D-only: generate a raster spray-paint trajectory above a mesh.

Strategy (no trimesh):
- Use Open3D's RaycastingScene to intersect a grid of rays with the mesh.
- Build raster "lanes" by sweeping along one axis (travel axis) and stacking lanes along the chosen raster axis.
- For each hit, take surface normal from the intersected triangle; offset by --offset (e.g., 0.03 m).
- Orient TCP: z_tcp points toward the surface (=-normal), x_tcp follows lane tangent, y_tcp = z x x.
- Export CSV poses and optional preview PLY of offset path points.

Assumptions:
- Mesh is in the robot base frame (or any fixed frame you want to export in).
- Raycasting collects the nearest hit (outer surface). For double-sided/concave cases, ensure rays approach from outside;
  we automatically try both +ray_dir and -ray_dir and keep the nearest hit if only one direction works.
- Requires Open3D >= 0.16 (RaycastingScene).

Usage example:
python o3d_spray_path_from_mesh.py --mesh blade_mesh.ply --offset 0.03 --spacing 0.02 --axis x --sample-step 0.005 \
       --ray-dir z- --out-csv blade_spray.csv --preview-ply blade_spray_preview.ply
"""
import argparse, math
import numpy as np
import open3d as o3d

def log(msg): print(f"[o3d-spray] {msg}", flush=True)

def unit(v, eps=1e-12):
    n = np.linalg.norm(v)
    return v if n < eps else (v / n)

def quat_from_matrix(R: np.ndarray) -> np.ndarray:
    m = R
    t = np.trace(m)
    if t > 0:
        s = math.sqrt(t+1.0)*2
        w = 0.25*s
        x = (m[2,1] - m[1,2]) / s
        y = (m[0,2] - m[2,0]) / s
        z = (m[1,0] - m[0,1]) / s
    else:
        if (m[0,0] > m[1,1]) and (m[0,0] > m[2,2]):
            s = math.sqrt(1.0 + m[0,0] - m[1,1] - m[2,2]) * 2
            w = (m[2,1] - m[1,2]) / s
            x = 0.25 * s
            y = (m[0,1] + m[1,0]) / s
            z = (m[0,2] + m[2,0]) / s
        elif m[1,1] > m[2,2]:
            s = math.sqrt(1.0 + m[1,1] - m[0,0] - m[2,2]) * 2
            w = (m[0,2] - m[2,0]) / s
            x = (m[0,1] + m[1,0]) / s
            y = 0.25 * s
            z = (m[1,2] + m[2,1]) / s
        else:
            s = math.sqrt(1.0 + m[2,2] - m[0,0] - m[1,1]) * 2
            w = (m[1,0] - m[0,1]) / s
            x = (m[0,2] + m[2,0]) / s
            y = (m[1,2] + m[2,1]) / s
            z = 0.25 * s
    q = np.array([x, y, z, w], dtype=float)
    q /= np.linalg.norm(q)
    return q

def rotate_about_axis(v: np.ndarray, axis: np.ndarray, angle_rad: float) -> np.ndarray:
    a = unit(axis)
    return (v * math.cos(angle_rad) +
            np.cross(a, v) * math.sin(angle_rad) +
            a * (np.dot(a, v)) * (1 - math.cos(angle_rad)))

def build_frames(points: np.ndarray,
                 normals: np.ndarray,
                 lane_reverse: bool,
                 offset: float,
                 tilt_deg: float = 0.0,
                 flip_normals: bool = False) -> np.ndarray:
    """Given lane samples (points & normals ordered along travel), build TCP frames."""
    N = len(points)
    if N == 0:
        return np.zeros((0,7))
    tilt = math.radians(tilt_deg)
    frames = []
    for i in range(N):
        p = points[i]
        # tangent along lane
        if i == 0:
            t = unit(points[i+1] - points[i]) if N > 1 else np.array([1,0,0], dtype=float)
        elif i == N-1:
            t = unit(points[i] - points[i-1])
        else:
            t = unit(points[i+1] - points[i-1])

        n = unit(normals[i])
        if flip_normals:
            n = -n

        z_tcp = unit(-n)  # point sprayer to surface
        if abs(tilt) > 1e-6:
            z_tcp = unit(rotate_about_axis(z_tcp, t, tilt))

        x_tcp = unit(t)
        if abs(np.dot(x_tcp, z_tcp)) > 0.99:
            tmp = np.array([1.0,0.0,0.0]) if abs(z_tcp[0]) < 0.9 else np.array([0.0,1.0,0.0])
            x_tcp = unit(np.cross(tmp, z_tcp))
        y_tcp = unit(np.cross(z_tcp, x_tcp))
        x_tcp = unit(np.cross(y_tcp, z_tcp))

        R = np.column_stack([x_tcp, y_tcp, z_tcp])
        q = quat_from_matrix(R)
        p_off = p + n * offset
        frames.append([p_off[0], p_off[1], p_off[2], q[0], q[1], q[2], q[3]])
    if lane_reverse:
        frames = frames[::-1]
    return np.asarray(frames)

def parse_ray_dir(s: str):
    s = s.strip().lower()
    if s in ("x","x+","+x"): return np.array([1,0,0], dtype=float)
    if s in ("x-","-x"):    return np.array([-1,0,0], dtype=float)
    if s in ("y","y+","+y"): return np.array([0,1,0], dtype=float)
    if s in ("y-","-y"):    return np.array([0,-1,0], dtype=float)
    if s in ("z","z+","+z"): return np.array([0,0,1], dtype=float)
    if s in ("z-","-z"):    return np.array([0,0,-1], dtype=float)
    raise ValueError("--ray-dir must be one of x/x+/x-, y/y+/y-, z/z+/z-")

def cast_rays_batch(scene, origins: np.ndarray, dirs: np.ndarray):
    """Cast rays in Open3D RaycastingScene; returns (hit_mask, hit_points, face_ids)."""
    import open3d as o3d
    import numpy as np
    o = o3d.core.Tensor(origins.astype(np.float32))
    d = o3d.core.Tensor(unit(dirs).astype(np.float32))
    rays = o3d.core.Tensor.concatenate([o, d], axis=1)
    ans = scene.cast_rays(rays)
    # t_hit = inf (1e10) => no hit
    t = ans['t_hit'].numpy()
    hit_mask = np.isfinite(t) & (t > 0)
    pts = origins + (dirs * t[:,None])
    # primitive ids -> triangle ids
    prim = ans['primitive_ids'].numpy().astype(np.int64)
    return hit_mask, pts, prim

def generate_raster_with_raycast(mesh_legacy: o3d.geometry.TriangleMesh,
                                 spacing: float,
                                 sample_step: float,
                                 raster_axis: str,
                                 ray_dir_str: str,
                                 ray_alt: bool = True):
    """Return list of lanes; each lane dict has 'points' Nx3 and 'normals' Nx3 (ordered)."""
    # Build RaycastingScene
    tmesh = o3d.t.geometry.TriangleMesh.from_legacy(mesh_legacy)
    scene = o3d.t.geometry.RaycastingScene()
    _ = scene.add_triangles(tmesh)

    # triangle normals for face lookup
    mesh_legacy.compute_triangle_normals()
    tri_normals = np.asarray(mesh_legacy.triangle_normals)

    # axes & bbox
    raster_axis = raster_axis.lower()
    if raster_axis not in ('x','y','z'):
        raise ValueError("--axis must be x, y, or z")
    bounds = mesh_legacy.get_axis_aligned_bounding_box()
    mn = bounds.get_min_bound()
    mx = bounds.get_max_bound()
    extent = mx - mn

    # Choose travel axis:
    # If lanes are stacked along raster_axis, we'll travel along the next axis in order XYZ (wrap).
    axes = {'x':0,'y':1,'z':2}
    a = axes[raster_axis]
    travel = (a + 1) % 3  # simple heuristic; you can expose as an arg if needed
    depth_ax = (a + 2) % 3

    # ray directions
    ray_dir = parse_ray_dir(ray_dir_str)
    ray_dir = unit(ray_dir)

    # lane positions along raster_axis
    L = float(extent[a])
    num = max(1, int(math.floor(L / spacing)) + 1)
    lane_vals = np.linspace(mn[a], mx[a], num=num)

    lanes = []
    # For each lane value, make a set of equally spaced samples along "travel" axis
    # For each sample we build a ray origin at far end along depth_ax and cast into scene.
    for li, lv in enumerate(lane_vals):
        lo = mn.copy(); hi = mx.copy()
        # Build origins along travel axis
        n_samples = max(2, int(math.ceil(extent[travel] / sample_step)) + 1)
        s_vals = np.linspace(mn[travel], mx[travel], num=n_samples)
        origins = np.zeros((n_samples, 3), dtype=float)
        origins[:, a] = lv                          # lane coordinate
        origins[:, travel] = s_vals                 # along-lane coordinate
        # start from outside the bbox along depth axis
        dir_vec = np.zeros(3); dir_vec[depth_ax] = ray_dir[depth_ax] if ray_dir[depth_ax] != 0 else -1.0
        # if ray_dir chosen doesn't vary along depth_ax, fall back to -1 along depth axis
        if abs(dir_vec[depth_ax]) < 1e-9:
            dir_vec[depth_ax] = -1.0
        # position origins slightly beyond bbox to guarantee intersection if visible
        margin = 0.05 + 0.2 * extent[depth_ax]  # 5 cm + 20% margin
        origins[:, depth_ax] = (mx if dir_vec[depth_ax] < 0 else mn)[depth_ax] + margin * dir_vec[depth_ax]
        dirs = np.tile(unit(dir_vec), (n_samples,1))

        hit_mask, pts, prim = cast_rays_batch(scene, origins, dirs)

        # optional alternate direction for misses
        if np.any(~hit_mask) and ray_alt:
            origins2 = origins.copy()
            dirs2 = -dirs
            # move to the opposite side
            origins2[:, depth_ax] = (mn if dir_vec[depth_ax] < 0 else mx)[depth_ax] - margin * dir_vec[depth_ax]
            hit2, pts2, prim2 = cast_rays_batch(scene, origins2, dirs2)
            use2 = (~hit_mask) & (hit2)
            pts[use2] = pts2[use2]
            prim[use2] = prim2[use2]
            hit_mask = hit_mask | hit2

        # filter hits in-bounds and sort along travel axis
        P = pts[hit_mask]
        if len(P) == 0:
            continue
        # normals from triangle ids
        N = tri_normals[prim[hit_mask]]
        # order by travel coordinate
        order = np.argsort(P[:, travel])
        P = P[order]; N = N[order]

        # serpentine: reverse every other lane for continuous robot motion
        lane_rev = (li % 2 == 1)
        lanes.append({"points": P, "normals": N, "reverse": lane_rev})
    return lanes

def main():
    ap = argparse.ArgumentParser(description="Open3D-only spray path generator (raster via raycasting).")
    ap.add_argument("--mesh", required=True, help="Input mesh (PLY/OBJ/STL).")
    ap.add_argument("--offset", type=float, default=0.03, help="Standoff [m] above surface (e.g., 0.03=30mm).")
    ap.add_argument("--spacing", type=float, default=0.02, help="Lane spacing along raster axis [m].")
    ap.add_argument("--axis", choices=["x","y","z"], default="x", help="Raster stacking axis.")
    ap.add_argument("--sample-step", type=float, default=0.005, help="Sample spacing along lane (approx) [m].")
    ap.add_argument("--ray-dir", type=str, default="z-", help="Initial ray direction to hit the surface: one of x/x+/x-, y/y+/y-, z/z+/z-.")
    ap.add_argument("--no-alt-ray", action="store_true", help="Disable second-pass rays from the opposite side.")
    ap.add_argument("--tilt-deg", type=float, default=0.0, help="Tilt around lane tangent [deg].")
    ap.add_argument("--flip-normals", action="store_true", help="Flip surface normals (if your mesh normals point inward).")
    ap.add_argument("--out-csv", default="spray_path.csv", help="Output CSV of poses.")
    ap.add_argument("--preview-ply", default=None, help="Optional: save offset path points as PLY for quick preview.")
    args = ap.parse_args()

    # Load legacy mesh
    mesh = o3d.io.read_triangle_mesh(args.mesh)
    if not mesh.has_triangle_normals():
        mesh.compute_triangle_normals()

    bbox = mesh.get_axis_aligned_bounding_box()
    log(f"Mesh loaded: {len(mesh.vertices)} verts, {len(mesh.triangles)} tris; bbox min={bbox.min_bound}, max={bbox.max_bound}")

    lanes = generate_raster_with_raycast(mesh,
                                         spacing=args.spacing,
                                         sample_step=args.sample_step,
                                         raster_axis=args.axis,
                                         ray_dir_str=args.ray_dir,
                                         ray_alt=not args.no_alt_ray)
    if not lanes:
        raise RuntimeError("No lanes generated. Try different --axis, --ray-dir, or adjust spacing/sample-step.")

    import csv
    count_pts = 0
    with open(args.out_csv, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["lane","idx","x","y","z","qx","qy","qz","qw"])
        for li, lane in enumerate(lanes):
            frames = build_frames(lane["points"], lane["normals"], lane["reverse"],
                                  offset=args.offset, tilt_deg=args.tilt_deg,
                                  flip_normals=args.flip_normals)
            for j, row in enumerate(frames):
                w.writerow([li, j] + row.tolist())
            count_pts += len(frames)
    log(f"Wrote {count_pts} poses across {len(lanes)} lanes -> {args.out_csv}")

    if args.preview_ply:
        import numpy as np
        all_pts = []
        for lane in lanes:
            frames = build_frames(lane["points"], lane["normals"], lane["reverse"],
                                  offset=args.offset, tilt_deg=args.tilt_deg,
                                  flip_normals=args.flip_normals)
            if len(frames):
                all_pts.append(frames[:, :3])
        if all_pts:
            all_pts = np.vstack(all_pts)
            pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(all_pts))
            o3d.io.write_point_cloud(args.preview_ply, pcd)
            log(f"Saved preview PLY with {len(all_pts)} points -> {args.preview_ply}")

if __name__ == "__main__":
    main()
