"""Check the ROS2<->Gazebo link: joint_states rate, camera image rate, and that a command moves arm_roll_z."""
import time
import rclpy
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Float64

rclpy.init(); n = rclpy.create_node("probe")
c = {"js": 0, "img": 0}; last = {}
n.create_subscription(JointState, "/joint_states", lambda m: (c.__setitem__("js", c["js"] + 1), last.update(zip(m.name, m.position))), 10)
n.create_subscription(Image, "/cam/loop/image", lambda m: c.__setitem__("img", c["img"] + 1), 10)
pub = n.create_publisher(Float64, "/cmd/arm_roll_z", 10)
t0 = time.time()
while time.time() - t0 < 4: rclpy.spin_once(n, timeout_sec=0.1)
print(f"joint_states msgs in 4s: {c['js']}, loop image msgs: {c['img']}, arm_roll_z={last.get('arm_roll_z')}")
for _ in range(10): pub.publish(Float64(data=0.3)); rclpy.spin_once(n, timeout_sec=0.1)
t0 = time.time()
while time.time() - t0 < 3: rclpy.spin_once(n, timeout_sec=0.1)
print(f"after cmd 0.3: arm_roll_z={last.get('arm_roll_z')}")
for _ in range(10): pub.publish(Float64(data=0.0)); rclpy.spin_once(n, timeout_sec=0.1)
