"""
Save the arm driver's model (URDF) as sheppy.urdf next to this file, for place_bowl.py.

    python3 save_urdf.py                   # /arm/robot_description (nightly from 2026-10-19)
    python3 save_urdf.py /robot_description

The driver computes /ee_state from this model, so ours must be the same one: save it again
whenever the driver (or the robot's mount.yaml) changes. The topic is latched, so it arrives
even though the driver published it long ago.
"""

import sys
from pathlib import Path

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

OUT = Path(__file__).resolve().parent / "sheppy.urdf"
TIMEOUT_S = 10.0


def main():
    topic = next((a for a in sys.argv[1:] if a.startswith("/")), "/arm/robot_description")
    rclpy.init()
    node = rclpy.create_node("save_urdf")
    got = []
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
    node.create_subscription(String, topic, lambda m: got.append(m.data), qos)
    t0 = node.get_clock().now()
    while not got and (node.get_clock().now() - t0).nanoseconds < TIMEOUT_S * 1e9:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_node()
    rclpy.shutdown()
    if not got:
        sys.exit(f"Nothing on {topic} after {TIMEOUT_S:.0f} s. Is the driver up (sheppy-arm_state_publisher "
                 "for /arm/robot_description)? Older drivers: python3 save_urdf.py /robot_description")
    urdf = got[0]
    if OUT.exists():
        OUT.with_suffix(".urdf.old").write_text(OUT.read_text())
    OUT.write_text(urdf)
    names = "arm_ prefix (nightly from 2026-10-19)" if 'name="arm_joint_1"' in urdf else "unprefixed joint names"
    print(f"saved {topic} -> {OUT} ({len(urdf)} chars, {names})"
          + (f"; the previous one is {OUT.with_suffix('.urdf.old').name}" if OUT.with_suffix(".urdf.old").exists() else ""))


if __name__ == "__main__":
    main()
