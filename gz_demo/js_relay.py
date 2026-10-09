"""ROS2 /joint_states -> UDP JSON on 127.0.0.1:47011 (for the SOFA brain camera view, which runs outside the ROS container)."""
import json
import socket
import time

import rclpy
from sensor_msgs.msg import JointState

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
last = [0.0]


def cb(m):
    if time.time() - last[0] > 1 / 30:
        sock.sendto(json.dumps(dict(zip(m.name, m.position))).encode(), ("127.0.0.1", 47011)); last[0] = time.time()


rclpy.init(); n = rclpy.create_node("js_relay"); n.create_subscription(JointState, "/joint_states", cb, 10); rclpy.spin(n)
