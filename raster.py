# Write a ready-to-run script that generates a raster robot trajectory 30 mm above a mesh
# by slicing the mesh with parallel planes, offsetting along surface normals, and exporting poses.
# Dependencies: trimesh, numpy, (optional) scipy for smoothing


"""
Generate a raster spray-paint trajectory above a mesh.

Pipeline:
1) Load a triangle mesh (already in robot base frame).
2) Slice the mesh with a stack of parallel planes (raster stripes).
3) For each slice polyline, sample points along the curve.
4) For each sample:
   - find a local surface normal n
   - compute tool point p_off = p + n * offset
   - build orientation: z_tcp = -n (spray points to surface), x_tcp = path tangent,
     y_tcp = z x x; (optional) tilt around x_tcp by `--tilt-deg`
5) Export CSV: X Y Z Qx Qy Qz Qw lane idx

Notes:
- This is geometry-only. It doesn't check collisions or robot joint limits.
- For concave regions, ensure your normals are oriented outward (flip if needed with --flip-normals).
- Choose spacing based on spray fan width W and desired overlap o: spacing = W * (1 - o).

Requirements: trimesh, numpy
"""
import argparse
import numpy as np
import math
import trimesh
from pathlib import Path
import pyglet

def log(msg): print(f"[spray] {msg}", flush=True)

def unit(v, eps=1e-12):
    n = np.linalg.norm(v)
    if n < eps: return v
    return v / n

def quat_from_matrix(R: np.ndarray) -> np.ndarray:
    # Returns quaternion [x,y,z,w] from 3x3 rotation matrix
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
    q = np.array([x,y,z,w], dtype=float)
    # Normalize
    q /= np.linalg.norm(q)
    return q

def rotate_about_axis(v: np.ndarray, axis: np.ndarray, angle_rad: float) -> np.ndarray:
    # Rodrigues' rotation formula
    a = unit(axis)
    return (v * math.cos(angle_rad) +
            np.cross(a, v) * math.sin(angle_rad) +
            a * (np.dot(a, v)) * (1 - math.cos(angle_rad)))

def build_frames_along_polyline(poly: np.ndarray,
                                mesh: trimesh.Trimesh,
                                offset: float,
                                tilt_deg: float = 0.0,
                                flip_normals: bool = False) -> np.ndarray:
    """
    For a 3D polyline on the surface, sample normals from nearest faces and build TCP frames.
    Returns array of shape [N, 7] with [x,y,z,qx,qy,qz,qw]
    """
    if len(poly) < 2:
        return np.zeros((0,7))
    # precompute nearest face for all points
    nearest, face_idx = mesh.nearest.on_surface(poly)
    # normals: use smoothed vertex normals if available, else face normals
    vnormals = mesh.vertex_normals if (mesh.vertex_normals is not None and len(mesh.vertex_normals)) else None
    frames = []
    tilt = math.radians(tilt_deg)

    for i, p in enumerate(poly):
        # tangent
        if i == 0:
            t = unit(poly[i+1] - poly[i])
        elif i == len(poly)-1:
            t = unit(poly[i] - poly[i-1])
        else:
            t = unit(poly[i+1] - poly[i-1])

        fi = int(face_idx[i])
        if vnormals is not None:
            # interpolate by nearest face vertices average (simple and stable)
            fvs = mesh.faces[fi]
            n = unit(mesh.vertex_normals[fvs].mean(axis=0))
        else:
            n = unit(mesh.face_normals[fi])

        if flip_normals:
            n = -n

        # tool axis points to surface => z_tcp = -n (spray direction)
        z_tcp = unit(-n)

        # optional tilt: rotate z around tangent
        if abs(tilt) > 1e-6:
            z_tcp = unit(rotate_about_axis(z_tcp, t, tilt))

        # build orthonormal basis (x along tangent)
        x_tcp = unit(t)
        # fix degeneracy if x parallel to z
        if abs(np.dot(x_tcp, z_tcp)) > 0.99:
            # pick any vector not parallel to z to re-orthogonalize x
            tmp = unit(np.array([1.0, 0.0, 0.0])) if abs(z_tcp[0]) < 0.9 else unit(np.array([0.0, 1.0, 0.0]))
            x_tcp = unit(np.cross(tmp, z_tcp))
        y_tcp = unit(np.cross(z_tcp, x_tcp))
        # re-orthogonalize
        x_tcp = unit(np.cross(y_tcp, z_tcp))

        R = np.column_stack([x_tcp, y_tcp, z_tcp])  # columns are axes in world
        q = quat_from_matrix(R)

        p_off = p + n * offset  # offset along surface normal outward
        frames.append([p_off[0], p_off[1], p_off[2], q[0], q[1], q[2], q[3]])
    return np.array(frames)

def _resample_polyline_equal_step(P, step):
    """Resample 3D polyline P (N×3) na zhruba rovnoměrný krok 'step' (m)."""
    if len(P) < 2 or step <= 0:
        return P
    seg = np.linalg.norm(P[1:] - P[:-1], axis=1)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    L = s[-1]
    if L <= step:
        return P
    # cílové vzdálenosti po oblouku
    t = np.arange(0.0, L + 1e-9, step)
    Q = []
    j = 0
    for tt in t:
        while j < len(seg) and s[j+1] < tt:
            j += 1
        if j >= len(seg):
            Q.append(P[-1])
            break
        denom = seg[j] if seg[j] > 1e-12 else 1.0
        alpha = (tt - s[j]) / denom
        Q.append(P[j] * (1 - alpha) + P[j+1] * alpha)
    return np.vstack(Q)

def slice_mesh_raster(mesh, spacing: float, axis: str = 'x', sample_step: float = 0.005):
    """
    Řezej mesh rovnoběžnými rovinami kolmo na zvolenou osu.
    Vrací list 3D polylinek (Nx3). Serpentine (každý druhý pruh obráceně).
    """
    axis = axis.lower()
    if axis == 'x':
        n = np.array([1.0, 0.0, 0.0]); lo, hi = mesh.bounds[0,0], mesh.bounds[1,0]
    elif axis == 'y':
        n = np.array([0.0, 1.0, 0.0]); lo, hi = mesh.bounds[0,1], mesh.bounds[1,1]
    elif axis == 'z':
        n = np.array([0.0, 0.0, 1.0]); lo, hi = mesh.bounds[0,2], mesh.bounds[1,2]
    else:
        raise ValueError("--axis must be x, y, or z")

    length = float(hi - lo)
    num = max(1, int(math.floor(length / spacing)) + 1)
    heights = np.linspace(lo, hi, num=num)

    paths = mesh.section_multiplane(
        plane_origin=mesh.bounds.mean(axis=0),
        plane_normal=n,
        heights=heights
    )

    polylines = []
    for i, path in enumerate(paths):
        if path is None or getattr(path, "entities", None) is None or len(path.entities) == 0:
            continue

        # POZOR: path.discrete je LIST polylinek (bez parametrů)
        try:
            curves = path.discrete  # list of (N,3)
        except Exception:
            # starší verze trimesh: někdy je potřeba nejdřív převod na 2D a zpět
            planar, T = path.to_planar()
            curves2d = planar.discrete
            curves = [trimesh.transform_points(np.column_stack([c, np.zeros(len(c))]), np.linalg.inv(T)) for c in curves2d]

        for P in curves:
            P = np.asarray(P)
            if P.ndim != 2 or P.shape[1] != 3 or len(P) < 2:
                continue
            # rovnoměrný krok po křivce
            P = _resample_polyline_equal_step(P, sample_step)
            if i % 2 == 1:
                P = P[::-1]  # serpentine
            polylines.append(P)

    return polylines

def main():
    ap = argparse.ArgumentParser(description="Generate a spray trajectory above a mesh via raster slicing.")
    ap.add_argument("--mesh", required=True, help="Input mesh file (PLY/OBJ/STL).")
    ap.add_argument("--offset", type=float, default=0.03, help="Standoff distance above surface [m].")
    ap.add_argument("--spacing", type=float, default=0.02, help="Stripe spacing between slicing planes [m].")
    ap.add_argument("--axis", choices=["x","y","z"], default="x", help="Raster slicing axis (planes perpendicular to this axis).")
    ap.add_argument("--sample-step", type=float, default=0.005, help="Sampling step along each section curve [m].")
    ap.add_argument("--tilt-deg", type=float, default=0.0, help="Tilt angle (deg) around path tangent; 0 = aim along normal.")
    ap.add_argument("--flip-normals", action="store_true", help="Flip normals if your mesh normals point inward.")
    ap.add_argument("--out-csv", default="spray_path.csv", help="Output CSV with poses in base frame.")
    ap.add_argument("--preview-ply", default=None, help="Optional: export a PLY with the offset path as small line segments.")
    args = ap.parse_args()

    mesh = trimesh.load(args.mesh, process=True)
    if not isinstance(mesh, trimesh.Trimesh):
        # If it's a scene or multiple meshes, merge
        mesh = trimesh.util.concatenate(mesh.dump())

    log(f"Mesh loaded: {len(mesh.vertices)} verts, {len(mesh.faces)} faces")
    #show the mesh
    mesh.show()


    polylines = slice_mesh_raster(mesh, spacing=args.spacing, axis=args.axis, sample_step=args.sample_step)
    if not polylines:
        raise RuntimeError("No intersection polylines were generated. Try different axis or spacing.")

    import csv
    with open(args.out_csv, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["lane","idx","x","y","z","qx","qy","qz","qw"])
        for lane, P in enumerate(polylines):
            frames = build_frames_along_polyline(P, mesh, offset=args.offset,
                                                 tilt_deg=args.tilt_deg,
                                                 flip_normals=args.flip_normals)
            for j, row in enumerate(frames):
                w.writerow([lane, j] + row.tolist())
    log(f"Wrote poses to {args.out_csv} (base frame).")

    if args.preview_ply:
        # Build a small point cloud for visualization of the offset path
        pts = []
        for P in polylines:
            F = build_frames_along_polyline(P, mesh, offset=args.offset,
                                            tilt_deg=args.tilt_deg,
                                            flip_normals=args.flip_normals)
            if len(F):
                pts.append(F[:, :3])
        if pts:
            pts = np.vstack(pts)
            # Save as ASCII PLY for easy viewing
            with open(args.preview_ply, "w") as pf:
                pf.write("ply\nformat ascii 1.0\n")
                pf.write(f"element vertex {len(pts)}\n")
                pf.write("property float x\nproperty float y\nproperty float z\nend_header\n")
                for p in pts:
                    pf.write(f"{p[0]} {p[1]} {p[2]}\n")
            log(f"Saved preview point cloud: {args.preview_ply}")

if __name__ == "__main__":
    main()