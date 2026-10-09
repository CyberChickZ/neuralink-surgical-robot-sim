"""Coarse tet mesh of the tissue block under the craniotomy, in WORLD frame, for real-time SOFA.
Cylinder (radius = hole radius + 2 mm, 10 mm thick), top conformed to the real (levelled) cortex height.
Writes <gz_dir>/block_mesh.npz: nodes, tets, side (fixed ids), bottom_tris (pressure), top (surface ids).
Usage: python make_block_mesh.py <gz_dir> [h=0.0025]      (sofa env: needs gmsh)
"""
import sys
from pathlib import Path

import gmsh
import numpy as np
from scipy.interpolate import griddata

GZ = Path(sys.argv[1]); H_EL = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0025
V = np.load(GZ / "vessels.npz"); C = V["cortex_world"]
R = float(np.load(GZ / "opening.npz")["radius_m"]) + 0.002
DEPTH = 0.010
z_top = float(np.median(C[np.linalg.norm(C[:, :2], axis=1) < 0.003, 2]))   # cortex height at the hole centre

gmsh.initialize(); gmsh.option.setNumber("General.Terminal", 0)
gmsh.model.occ.addCylinder(0, 0, -DEPTH, 0, 0, DEPTH, R); gmsh.model.occ.synchronize()
gmsh.option.setNumber("Mesh.MeshSizeMax", H_EL); gmsh.option.setNumber("Mesh.MeshSizeMin", H_EL * 0.5)
gmsh.model.mesh.generate(3)
tags, xyz, _ = gmsh.model.mesh.getNodes(); P = xyz.reshape(-1, 3)
idx = {t: i for i, t in enumerate(tags)}
_, en = gmsh.model.mesh.getElementsByType(4); T = np.vectorize(idx.get)(en.reshape(-1, 4))
gmsh.finalize()

# conform the top to the cortex: shift each node by the cortex height above its xy, blended linearly to 0 at the bottom
zc = griddata(C[:, :2], C[:, 2], P[:, :2], method="linear")
zc = np.where(np.isnan(zc), griddata(C[:, :2], C[:, 2], P[:, :2], method="nearest"), zc)
w = (P[:, 2] + DEPTH) / DEPTH                       # 0 at bottom, 1 at top
P[:, 2] = z_top - DEPTH + (P[:, 2] + DEPTH) + w * (zc - z_top)

r = np.linalg.norm(P[:, :2], axis=1)
side = np.where(r > R - 1e-6)[0]
top = np.where(w > 1 - 1e-9)[0]
faces = np.concatenate([T[:, [0, 2, 1]], T[:, [0, 1, 3]], T[:, [0, 3, 2]], T[:, [1, 2, 3]]])
_, inv, cnt = np.unique(np.sort(faces, axis=1), axis=0, return_inverse=True, return_counts=True)
bnd = faces[cnt[inv] == 1]
bottom = bnd[np.all(w[bnd] < 1e-9, axis=1)]
np.savez(GZ / "block_mesh.npz", nodes=P, tets=T, side=side, top=top, bottom_tris=bottom, radius=R, depth=DEPTH)
print(f"block: nodes {len(P)} tets {len(T)} side {len(side)} top {len(top)} bottom tris {len(bottom)} R={R*1e3:.1f}mm h={H_EL*1e3}mm")
