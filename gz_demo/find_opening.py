"""Locate the craniotomy = cortex region NOT covered by the D3MIA skull mesh (skull = cortex surface with a hole).
Exposed cortex vertex: no skull vertex within 1 mm. Saves <gz_dir>/opening.npz:
  center_idx (cortex vertex nearest to the exposed-area centroid, used as the opening centre), radius_m, exposed (bool per vertex)
Usage: python find_opening.py <gz_dir>      (needs scipy: run with the sofa env)
"""
import sys
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree

DATA = Path("/nfs/hpc/share/zhanhaoc/neuro_demo/third_party/SOFA-NeuroSim-Recorder/data")
GZ = Path(sys.argv[1])
load = lambda f: np.array([l.split()[1:4] for l in open(DATA / f) if l.startswith("v ")], float) * 1e-3
C, S = load("surface_full_decimated.obj"), load("surface_skull.obj")
d, _ = cKDTree(S).query(C)
exposed = d > 1e-3
lab_pts = C[exposed]
# keep the largest connected blob of exposed vertices (5 mm linkage) in case of stray uncovered spots
tree = cKDTree(lab_pts); pairs = tree.query_pairs(0.005, output_type="ndarray")
parent = np.arange(len(lab_pts))
def find(i):
    while parent[i] != i:
        parent[i] = parent[parent[i]]; i = parent[i]
    return i
for a, b in pairs:
    ra, rb = find(a), find(b)
    if ra != rb: parent[ra] = rb
roots = np.array([find(i) for i in range(len(lab_pts))])
big = roots == np.bincount(roots).argmax()
idx_exp = np.where(exposed)[0][big]
cen = C[idx_exp].mean(0)
center_idx = int(idx_exp[np.argmin(np.linalg.norm(C[idx_exp] - cen, axis=1))])
radius = float(np.sqrt(np.mean(np.sum((C[idx_exp] - C[center_idx]) ** 2, axis=1)) * 2))  # disk: E[r^2] = R^2/2
ex = np.zeros(len(C), bool); ex[idx_exp] = True
np.savez(GZ / "opening.npz", center_idx=center_idx, radius_m=radius, exposed=ex)
print(f"exposed verts {ex.sum()} / {len(C)}; opening centre (brain frame, mm) {(C[center_idx] * 1e3).round(1)}; "
      f"radius {radius * 1e3:.1f} mm; old ring-based probe was at (0,25,~54) mm")
