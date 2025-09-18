
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ply2image_segment_backmap.py

Pipeline:
- Load Photoneo .ply (ordered grid) and extract width & height
- Create 2D images: RGB (HxWx3) and Texture (HxW), saved as PNGs
- Optionally run your own segmentation model (stub provided) or load a provided mask PNG
- Map segmentation (HxW) back onto the 3D point cloud (N = H*W), save:
    - labels.npy (int32, shape (N,))
    - segmented_points.ply (colorized by class label)  [simplified vertex-only PLY]

Requires: numpy, imageio (or pillow), plyfile, (optional) opencv-python for advanced usage.
"""
import re
import sys
import argparse
import numpy as np
from pathlib import Path
import os

try:
    import imageio.v3 as iio  # imageio v3 API
except Exception:
    from PIL import Image
    def iio_imwrite(path, arr):
        Image.fromarray(arr).save(path)
    def iio_imread(path):
        return np.array(Image.open(path))
else:
    def iio_imwrite(path, arr):
        iio.imwrite(path, arr)
    def iio_imread(path):
        return iio.imread(path)

from plyfile import PlyData, PlyElement

# -----------------------------
# Utilities
# -----------------------------

def parse_width_height_from_obj_info(plydata: PlyData):
    """
    Try to parse width/height from the 'obj_info' or comments line like:
    'Photoneo PLY PointCloud (Width = 2064; Height = 1544; Ordered)'
    """
    all_comments = []
    # plyfile stores comments in PlyData.comments (list of str)
    if hasattr(plydata, 'comments'):
        all_comments.extend(plydata.comments)
    # some tools may store them in obj_info as "obj_info" elements; plyfile exposes them as comments too
    pattern = re.compile(r'Width\s*=\s*(\d+).*Height\s*=\s*(\d+)', re.IGNORECASE)
    for c in all_comments:
        m = pattern.search(c)
        if m:
            w = int(m.group(1))
            h = int(m.group(2))
            return w, h
    return None

def parse_width_height_from_elements(plydata: PlyData):
    """
    Fallback: try to get width/height from elements in header (e.g., phoxi_frame_params or camera_resolution)
    """
    # camera_resolution: width, height (floats) - if present
    if 'camera_resolution' in plydata.elements:
        el = plydata['camera_resolution']
        try:
            w = int(round(float(el.data['width'][0])))
            h = int(round(float(el.data['height'][0])))
            if w > 0 and h > 0:
                return w, h
        except Exception:
            pass

    # phoxi_frame_params: frame_width, frame_height (uint32) - if present
    if 'phoxi_frame_params' in plydata.elements:
        el = plydata['phoxi_frame_params']
        try:
            w = int(el.data['frame_width'][0])
            h = int(el.data['frame_height'][0])
            if w > 0 and h > 0:
                return w, h
        except Exception:
            pass
    return None

def get_width_height(plydata: PlyData, n_vertices: int):
    wh = parse_width_height_from_obj_info(plydata)
    if wh is None:
        wh = parse_width_height_from_elements(plydata)
    if wh is None:
        raise RuntimeError("Nepodařilo se najít Width/Height v hlavičce .ply. "
                           "Zkuste použít --width/--height ručně.")
    w, h = wh
    if w * h != n_vertices:
        raise RuntimeError(f"Width*Height ({w}*{h}={w*h}) neodpovídá počtu vrcholů ({n_vertices}). "
                           "Zkontrolujte soubor nebo zadejte správné rozměry ručně.")
    return w, h

def to_uint8_safe(arr):
    """Convert float [0..1] or any range to uint8 by automatic normalization if needed."""
    arr = np.asarray(arr)
    if arr.dtype == np.uint8:
        return arr
    finite = np.isfinite(arr)
    if not finite.any():
        return np.zeros(arr.shape, dtype=np.uint8)
    amin = arr[finite].min()
    amax = arr[finite].max()
    if amax == amin:
        return np.zeros(arr.shape, dtype=np.uint8)
    norm = (arr - amin) / (amax - amin)
    return np.clip((norm * 255.0 + 0.5).astype(np.uint8), 0, 255)

def colorize_labels(labels, num_classes=None):
    """
    Map integer labels -> RGB colors for visualization.
    A simple fixed palette; extend if needed.
    """
    labels = labels.astype(np.int32)
    if num_classes is None:
        num_classes = int(labels.max()) + 1 if labels.size else 0
    # Simple palette (tab20-like subset); add more as needed
    palette = np.array([
        [  0,  0,  0],  # 0 - background (black)
        [255,  0,  0],  # 1 - red
        [  0,255,  0],  # 2 - green
        [  0,  0,255],  # 3 - blue
        [255,255,  0],  # 4 - yellow
        [255,  0,255],  # 5 - magenta
        [  0,255,255],  # 6 - cyan
        [255,127, 80],  # 7 - coral
        [127,255,  0],  # 8 - chartreuse
        [138, 43,226],  # 9 - blueviolet
        [255,165,  0],  # 10 - orange
        [ 46,139, 87],  # 11 - seagreen
        [ 70,130,180],  # 12 - steelblue
        [210,105, 30],  # 13 - chocolate
        [123,104,238],  # 14 - mediumpurple
        [255,192,203],  # 15 - pink
        [ 47, 79, 79],  # 16 - darkslategray
        [154,205, 50],  # 17 - yellowgreen
        [139, 69, 19],  # 18 - saddlebrown
        [  0,128,128],  # 19 - teal
    ], dtype=np.uint8)
    if num_classes > len(palette):
        # extend deterministically
        rng = np.random.default_rng(42)
        extra = rng.integers(0, 256, size=(num_classes - len(palette), 3), dtype=np.uint8)
        palette = np.vstack([palette, extra])
    colors = palette[np.clip(labels, 0, num_classes - 1)]
    return colors

def save_segmented_ply_xyzrgb(xyz, normals, labels, out_path):
    """
    Save a simplified vertex-only PLY with XYZ, Normals (if available), and colorized RGB by labels.
    This does NOT preserve extra elements from the original PLY.
    """
    colors = colorize_labels(labels)
    n = xyz.shape[0]
    has_normals = normals is not None and normals.shape == (n, 3)
    if has_normals:
        vertex_dtype = np.dtype([
            ('x', 'f4'), ('y', 'f4'), ('z', 'f4'),
            ('nx','f4'), ('ny','f4'), ('nz','f4'),
            ('red','u1'), ('green','u1'), ('blue','u1'),
            ('label','i4'),
        ])
        v = np.empty(n, dtype=vertex_dtype)
        v['x'], v['y'], v['z'] = xyz[:,0], xyz[:,1], xyz[:,2]
        v['nx'], v['ny'], v['nz'] = normals[:,0], normals[:,1], normals[:,2]
    else:
        vertex_dtype = np.dtype([
            ('x', 'f4'), ('y', 'f4'), ('z', 'f4'),
            ('red','u1'), ('green','u1'), ('blue','u1'),
            ('label','i4'),
        ])
        v = np.empty(n, dtype=vertex_dtype)
        v['x'], v['y'], v['z'] = xyz[:,0], xyz[:,1], xyz[:,2]

    v['red'], v['green'], v['blue'] = colors[:,0], colors[:,1], colors[:,2]
    v['label'] = labels.astype(np.int32)

    el = PlyElement.describe(v, 'vertex', comments=['colors = label colormap'])
    PlyData([el], text=False).write(out_path)

# -----------------------------
# Main pipeline
# -----------------------------

def main():

    ply_path = "/home/robolab2/master_project/frames"
    scan_name = "scan_28.ply"
    scan_path = os.path.join(ply_path, scan_name)
    print(os.getcwd())

    # Load PLY
    print(f"Loading PLY: {scan_path}")
    plydata = PlyData.read(str(scan_path))

    # Vertex data
    plydata._element_lookup
    if 'vertex' not in plydata.elements:
        raise RuntimeError("Element 'vertex' v .ply nenalezen.")
    vtx = plydata['vertex'].data
    n_vertices = vtx.shape[0]
    print(f"Vertices: {n_vertices}")

    # Get width/height
    if args.width is not None and args.height is not None:
        W, H = args.width, args.height
        if W * H != n_vertices:
            raise RuntimeError(f"--width*--height != number of vertices ({W}*{H} != {n_vertices})")
    else:
        W, H = get_width_height(plydata, n_vertices)
    print(f"Detected frame size: {W} x {H}")

    # Extract arrays (if fields exist)
    def field(name, default=None):
        return vtx[name] if name in vtx.dtype.names else default

    x = field('x'); y = field('y'); z = field('z')
    xyz = None
    if x is not None and y is not None and z is not None:
        xyz = np.column_stack([x, y, z]).astype(np.float32)
    nx = field('nx'); ny = field('ny'); nz = field('nz')
    normals = None
    if nx is not None and ny is not None and nz is not None:
        normals = np.column_stack([nx, ny, nz]).astype(np.float32)

    red = field('red'); green = field('green'); blue = field('blue')
    texture = field('Texture32')
    depth = field('Depth32')

    # Build images (row-major ordering assumed): idx = r*W + c
    if args.use_texture and texture is not None:
        tex = np.asarray(texture).reshape(H, W)
        tex_u8 = to_uint8_safe(tex)
        iio_imwrite(str(outdir / "texture.png"), tex_u8)
        print(f"Saved texture image: {outdir / 'texture.png'}")
    else:
        if red is None or green is None or blue is None:
            raise RuntimeError("RGB kanály v .ply chybí a --use_texture nebyl zvolen.")
        rgb = np.dstack([red.reshape(H, W), green.reshape(H, W), blue.reshape(H, W)]).astype(np.uint8)
        iio_imwrite(str(outdir / "rgb.png"), rgb)
        print(f"Saved RGB image: {outdir / 'rgb.png'}")

    if args.save_depth and depth is not None:
        depth_vis = to_uint8_safe(depth.reshape(H, W))  # visualization only
        iio_imwrite(str(outdir / "depth_vis.png"), depth_vis)
        print(f"Saved depth visualization: {outdir / 'depth_vis.png'}")

    # Segmentation: either load a mask or create a stub
    if args.mask_path is not None:
        mask_arr = iio_imread(args.mask_path)
        if mask_arr.ndim == 3:
            # If colored mask, convert to single-channel by taking the first channel or unique-color indexing
            # Simplest: assume class ids are in the first channel
            mask_arr = mask_arr[..., 0]
        mask = np.asarray(mask_arr).astype(np.int32)
        if mask.shape != (H, W):
            raise RuntimeError(f"Mask shape {mask.shape} != ({H}, {W}).")
        print(f"Loaded mask: {args.mask_path}")
    else:
        # Stub: mark valid (finite depth or finite z) as class 1, else 0
        valid = None
        if depth is not None:
            valid = np.isfinite(depth).reshape(H, W)
        elif z is not None:
            valid = np.isfinite(z).reshape(H, W)
        else:
            valid = np.ones((H, W), dtype=bool)
        mask = np.where(valid, 1, 0).astype(np.int32)
        print("No mask provided; using a trivial stub mask (class=1 for valid points).")

    # Map back to points
    labels = mask.reshape(-1).astype(np.int32)
    np.save(outdir / "labels.npy", labels)
    print(f"Saved labels.npy (aligned with vertex order): {outdir / 'labels.npy'}")

    # Save a colorized point cloud (simplified PLY)
    if xyz is None:
        print("WARNING: No XYZ in PLY. Skipping segmented_points.ply export.")
    else:
        save_segmented_ply_xyzrgb(xyz, normals, labels, str(outdir / "segmented_points.ply"))
        print(f"Saved colorized segmented point cloud: {outdir / 'segmented_points.ply'}")

    print("Done.")

if __name__ == "__main__":
    main()
