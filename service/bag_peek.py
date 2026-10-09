"""Quick check of a run's bag: save camera | camera+vessel-GT overlay for 2 cameras, and the vessel map, at message #N.
Usage (inside the ROS container): python3 bag_peek.py <run_dir>/bag <out_prefix> [N=40]"""
import sys

import cv2
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image

bag, out = sys.argv[1], sys.argv[2]; N = int(sys.argv[3]) if len(sys.argv) > 3 else 40
r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=bag, storage_id="mcap"), rosbag2_py.ConverterOptions("", ""))
keys = ["/cam/east/image", "/cam/east/vessel_mask", "/cam/south/image", "/cam/south/vessel_mask", "/brain/vessel_map"]
got, cnt = {}, {}
while r.has_next():
    t, data, _ = r.read_next()
    if t in keys:
        cnt[t] = cnt.get(t, 0) + 1
        if cnt[t] == N:
            got[t] = deserialize_message(data, Image)
a = {k: np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.width, -1) for k, m in got.items()}


def ov(img, m):
    o = img[..., :3].copy(); sel = m[..., 0] > 0; o[sel] = (0.5 * o[sel] + [0, 127, 0]).astype(np.uint8); return o


row = np.hstack([a[keys[0]][..., :3], ov(a[keys[0]], a[keys[1]]), a[keys[2]][..., :3], ov(a[keys[2]], a[keys[3]])])
cv2.imwrite(out + "_cams.png", cv2.cvtColor(row, cv2.COLOR_RGB2BGR))
cv2.imwrite(out + "_map.png", cv2.cvtColor(a[keys[4]], cv2.COLOR_RGB2BGR))
print("saved", out + "_cams.png", "mask px east/south", int((a[keys[1]] > 0).sum()), int((a[keys[3]] > 0).sum()))
