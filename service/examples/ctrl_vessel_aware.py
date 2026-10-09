"""Vessel-aware controller: reads the live vessel map, keeps only sites with >= CLEAR m clearance to every vessel edge
and >= SPACING between sites, then inserts there."""
import numpy as np

from neuro_sdk import Robot

CLEAR, SPACING, N_SITES = 0.0004, 0.002, 9
rb = Robot()
V = rb.vessels()                                                  # x, y, z, radius, id
g = np.arange(-0.010, 0.0101, 0.0005)
cand = np.array([(x, y) for x in g for y in g])
d = np.hypot(cand[:, None, 0] - V[None, :, 0], cand[:, None, 1] - V[None, :, 1]) - V[None, :, 3]
clear = d.min(axis=1)
sites = []
for i in np.argsort(-clear):                                      # greedy: largest clearance first, keep spacing
    if clear[i] < CLEAR or len(sites) == N_SITES:
        break
    if all(np.hypot(*(cand[i] - s)) >= SPACING for s in sites):
        sites.append(cand[i])
rb.event(f"vessel-aware: {len(sites)} sites, min planned clearance {min(clear[np.argsort(-clear)][:len(sites)])*1e6:.0f} um")
for x, y in sites:
    rb.move_tip(float(x), float(y))
    r = rb.insert(depth=0.004)
    rb.event(f"site ({x*1e3:+.1f},{y*1e3:+.1f}) mm: punctured={r['punctured']} peak={r['peak_force_n']*1e3:.2f}mN hits={r['hits']}")
    rb.retract()
rb.event(f"vessel-aware: done, total vessel hits {len(rb.hits)}")
rb.close()
