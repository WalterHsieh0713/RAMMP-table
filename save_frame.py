"""
Grab ONE depth (+ color) frame from the robot camera, save it to disk, and exit.
Lets you test table_detect.py on real robot data offline.

    python3 save_frame.py
    python3 save_frame.py --ros-args -p depth_topic:=/camera/wrist/depth/image_rect_raw -p info_topic:=/camera/wrist/depth/camera_info

Writes robot_depth.png (16-bit, millimeters), robot_color.png (if a color topic is given),
and prints the exact table_detect.py command to run on them.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

from table_detect import write_image
from table_detector_node import depth_to_meters, image_to_numpy


class SaveFrame(Node):
    def __init__(self):
        super().__init__("save_frame")
        self.declare_parameter("depth_topic", "/camera/wrist/aligned_depth_to_color/image_raw")
        self.declare_parameter("info_topic", "/camera/wrist/aligned_depth_to_color/camera_info")
        self.declare_parameter("color_topic", "/camera/wrist/color/image_raw")
        self.depth = self.info = self.color = None
        self.done = False
        self.create_subscription(Image, self.get_parameter("depth_topic").value,
                                 lambda m: setattr(self, "depth", m), 5)
        self.create_subscription(CameraInfo, self.get_parameter("info_topic").value,
                                 lambda m: setattr(self, "info", m), 5)
        if self.get_parameter("color_topic").value:
            self.create_subscription(Image, self.get_parameter("color_topic").value,
                                     lambda m: setattr(self, "color", m), 5)
        self.create_timer(0.2, self._try_save)
        self.get_logger().info("Waiting for depth + camera_info...")

    def _try_save(self):
        if self.done or self.depth is None or self.info is None:
            return
        depth_mm = (depth_to_meters(self.depth) * 1000.0).astype(np.uint16)
        write_image("robot_depth.png", depth_mm)
        if self.color is not None:
            color = np.ascontiguousarray(image_to_numpy(self.color))
            write_image("robot_color.png", color)
            color_arg = "robot_color.png"
        else:
            color_arg = "robot_depth.png"
        K = self.info.k
        print("\nSaved robot_depth.png" + (" and robot_color.png" if self.color is not None else ""))
        print(f"Depth frame id: {self.depth.header.frame_id}, size {self.depth.width}x{self.depth.height}")
        print("Run the detector on it with:\n")
        print(f"python3 table_detect.py --color {color_arg} --depth robot_depth.png "
              f"--fx {K[0]:.2f} --fy {K[4]:.2f} --cx {K[2]:.2f} --cy {K[5]:.2f} "
              f"--depth-scale 1000 --largest-plane --out robot_overlay.png\n")
        print("Note: if the color image size differs from depth, the red overlay won't line up -"
              " pass --color robot_depth.png instead.")
        self.done = True


def main():
    rclpy.init()
    node = SaveFrame()
    while rclpy.ok() and not node.done:
        rclpy.spin_once(node, timeout_sec=0.5)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
