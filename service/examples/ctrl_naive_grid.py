"""Naive controller: inserts on a fixed 3x3 grid (+-8 mm) and ignores the vasculature."""
from neuro_sdk import Robot

rb = Robot()
rb.event("naive grid: start")
for x in (-0.008, 0.0, 0.008):
    for y in (-0.008, 0.0, 0.008):
        rb.move_tip(x, y)
        r = rb.insert(depth=0.004, speed=0.01)
        rb.event(f"site ({x*1e3:+.0f},{y*1e3:+.0f}) mm: punctured={r['punctured']} peak={r['peak_force_n']*1e3:.2f}mN hits={r['hits']}")
        rb.retract()
rb.event(f"naive grid: done, total vessel hits {len(rb.hits)}")
rb.close()
