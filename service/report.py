"""Per-run report from a run's rosbag: one row per insertion episode (contact -> retract) + run summary.
Writes <run_dir>/metrics.json and <run_dir>/xsection_inserted.png (cross-section at the deepest insertion).
Usage (inside the ROS container): python3 report.py <run_dir>
"""
import json
import sys
from pathlib import Path

import cv2
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image
from std_msgs.msg import String

run = Path(sys.argv[1])
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=str(run / "bag"), storage_id="mcap"), rosbag2_py.ConverterOptions("", ""))
states, events, xs = [], [], []
while r.has_next():
    topic, data, ts = r.read_next()
    if topic == "/brain/needle_state":
        states.append(json.loads(deserialize_message(data, String).data))
    elif topic == "/brain/vessel_burst":
        events.append(json.loads(deserialize_message(data, String).data))
    elif topic == "/brain/xsection_xz":
        xs.append((ts, deserialize_message(data, Image)))

# episodes: maximal runs of consecutive samples in contact/inserted
eps, cur = [], None
for s in states:
    active = s["state"] in ("contact", "inserted")
    if active and cur is None:
        cur = {"t_start": s["t"], "samples": []}
    if active:
        cur["samples"].append(s)
    elif cur is not None:
        eps.append(cur); cur = None
if cur is not None:
    eps.append(cur)
rows = []
for i, e in enumerate(eps):
    S = e["samples"]
    ins = [s for s in S if s["state"] == "inserted"]
    hits = [ev for ev in events if ev.get("event", "vessel_hit") == "vessel_hit" and e["t_start"] <= ev["t"] <= S[-1]["t"] + 0.05]
    rows.append({"episode": i, "t_start": e["t_start"], "t_end": S[-1]["t"],
                 "entry_xy_mm": [round(v * 1e3, 3) for v in S[0]["tip_world"][:2]],
                 "peak_force_mN": round(max(s.get("contact_peak_force_n", s.get("needle_force_n", 0.0)) for s in S) * 1e3, 3),
                 "punctured": bool(ins), "max_depth_mm": round(max((s["depth_m"] for s in ins), default=0.0) * 1e3, 3),
                 "min_vessel_clearance_um": round(min(s["nearest_vessel_clearance_m"] for s in S) * 1e6, 1),
                 "vessel_hits": [h["vessel_id"] for h in hits]})
import yaml
crit = (yaml.safe_load(open(run / "scenario.yaml")) or {}).get("pass", {}) if (run / "scenario.yaml").exists() else {}
n_hits = sum(len(r_["vessel_hits"]) for r_ in rows); n_punct = sum(r_["punctured"] for r_ in rows)
ok = n_hits <= crit.get("max_vessel_hits", 0) and n_punct >= crit.get("min_punctured", 0)
summary = {"n_insertions": len(rows), "n_punctured": sum(r_["punctured"] for r_ in rows),
           "n_vessel_hits": sum(len(r_["vessel_hits"]) for r_ in rows),
           "criteria": crit, "pass": bool(ok), "episodes": rows}
(run / "metrics.json").write_text(json.dumps(summary, indent=1))
print(json.dumps({k: v for k, v in summary.items() if k != "episodes"}))
for r_ in rows:
    print(r_)
if xs and states:
    t_deep = max(states, key=lambda s: s["depth_m"] if s["state"] == "inserted" else -1)["t"]
    m = min(xs, key=lambda x: abs(x[1].header.stamp.sec + x[1].header.stamp.nanosec * 1e-9 - t_deep))[1]
    a = np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.width, 3)
    cv2.imwrite(str(run / "xsection_inserted.png"), cv2.cvtColor(a, cv2.COLOR_RGB2BGR))

frames = sorted((run / "dash").glob("*.jpg"))
if frames:  # H.264 (browser/phone playable); static ffmpeg shipped in the image, host fallback for non-image runs
    import os
    import subprocess
    ff = next(f for f in (os.environ.get("NEURO_FFMPEG", ""), "/opt/neuro/bin/ffmpeg",
              "/nfs/hpc/dgx2-2/zhanhaoc/envs/sofa/lib/python3.12/site-packages/imageio_ffmpeg/binaries/ffmpeg-linux-x86_64-v7.0.2") if f and os.path.exists(f))
    subprocess.run([ff, "-y", "-loglevel", "error", "-framerate", "10", "-i", str(run / "dash" / "%06d.jpg"), "-c:v", "libx264",
                    "-pix_fmt", "yuv420p", "-crf", "23", "-movflags", "+faststart", str(run / "dashboard.mp4")], check=True)
    print(f"dashboard.mp4: {len(frames)} frames (h264)")
