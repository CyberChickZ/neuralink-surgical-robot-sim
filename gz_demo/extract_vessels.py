"""Vessel ground truth from the D3MIA cortex texture (what the cameras see) -> 3D vessel centreline points.
Segment red vessel pixels around the opening, skeletonize, radius = distance transform, map each skeleton pixel
through UV -> (triangle, barycentric) on the cortex mesh -> 3D. Saved in WORLD frame (levelled brain pose).
Usage: python extract_vessels.py <gz_dir>      -> <gz_dir>/vessels.npz
"""
import sys
from pathlib import Path

import numpy as np
from PIL import Image
from scipy import ndimage
from skimage.morphology import skeletonize

DATA = Path("/nfs/hpc/share/zhanhaoc/neuro_demo/third_party/SOFA-NeuroSim-Recorder/data")
GZ = Path(sys.argv[1])
V, UV, F = [], [], []
for l in open(DATA / "surface_full_decimated.obj"):
    if l.startswith("v "): V.append(l.split()[1:4])
    elif l.startswith("vt "): UV.append(l.split()[1:3])
    elif l.startswith("f "): F.append([int(p.split("/")[0]) - 1 for p in l.split()[1:4]])
V, UV, F = np.array(V, float) * 1e-3, np.array(UV, float), np.array(F)   # metres, brain frame
OPEN = np.load(GZ / "opening.npz")                                         # find_opening.py: skull hole
probe = int(OPEN["center_idx"])
near = np.all(OPEN["exposed"][F], axis=1)                                  # faces inside the craniotomy (not under the skull)
Fn = F[near]

tex = np.asarray(Image.open(DATA / "texture_outpaint.png").convert("RGB")).astype(int)
H, W = tex.shape[:2]
R, G = tex[..., 0], tex[..., 1]
vessel = ((R - G) > 110) & (G < 120)                                       # saturated red = vessel; pink tissue / grey shadow excluded

# rasterize the near faces in UV space -> per-pixel (face, barycentric)
px = np.stack([UV[:, 0] * W, (1 - UV[:, 1]) * H], 1)
face_of = np.full((H, W), -1, int); bary = np.zeros((H, W, 3))
for fi, f in zip(np.where(near)[0], Fn):
    p = px[f]; x0, y0 = np.floor(p.min(0)).astype(int); x1, y1 = np.ceil(p.max(0)).astype(int)
    xs, ys = np.meshgrid(np.arange(x0, x1 + 1), np.arange(y0, y1 + 1))
    q = np.stack([xs.ravel() + 0.5, ys.ravel() + 0.5], 1)
    T = np.array([p[0] - p[2], p[1] - p[2]]).T
    if abs(np.linalg.det(T)) < 1e-12:
        continue
    l12 = np.linalg.solve(T, (q - p[2]).T).T
    b = np.column_stack([l12, 1 - l12.sum(1)])
    ok = np.all(b >= -1e-6, axis=1) & (q[:, 0] < W) & (q[:, 1] < H)
    yy, xx = ys.ravel()[ok], xs.ravel()[ok]
    face_of[yy, xx] = fi; bary[yy, xx] = b[ok]

region = face_of >= 0
vm = vessel & region
skel = skeletonize(vm)
dist_px = ndimage.distance_transform_edt(vm)
lab, nlab = ndimage.label(skel, structure=np.ones((3, 3)))
ys, xs = np.nonzero(skel)
fi = face_of[ys, xs]; b = bary[ys, xs]
p3 = np.einsum("nk,nkj->nj", b, V[F[fi]])                                 # brain frame, metres
# metres per texture pixel, per face: sqrt(3D area / UV-pixel area)
a3 = 0.5 * np.linalg.norm(np.cross(V[F[fi, 1]] - V[F[fi, 0]], V[F[fi, 2]] - V[F[fi, 0]]), axis=1)
pu = px[F[fi]]
a2 = 0.5 * np.abs((pu[:, 1, 0] - pu[:, 0, 0]) * (pu[:, 2, 1] - pu[:, 0, 1]) - (pu[:, 2, 0] - pu[:, 0, 0]) * (pu[:, 1, 1] - pu[:, 0, 1]))
m_per_px = np.sqrt(a3 / np.maximum(a2, 1e-12))
radius = dist_px[ys, xs] * m_per_px

ROT, OFF = np.load(GZ / "brain_rot.npy"), np.load(GZ / "brain_offset.npy")
pw = p3 @ ROT.T + OFF
Cw = V @ ROT.T + OFF
cortex = Cw[np.linalg.norm(Cw[:, :2], axis=1) < float(OPEN["radius_m"]) + 0.003]   # cortex points around the opening (world)
np.savez(GZ / "vessels.npz", cortex_world=cortex, xyz_world=pw, radius=radius, vessel_id=lab[ys, xs], face=F[fi], bary=b,
         xyz_brain=p3, probe=probe, uv_px=np.stack([xs, ys], 1))
print(f"vessel pixels {vm.sum()} skeleton points {len(pw)} vessels {nlab}  radius um p10/50/90 "
      f"{np.percentile(radius * 1e6, [10, 50, 90]).round(0)}  mm/px {np.median(m_per_px) * 1e3:.3f}")
