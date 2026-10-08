"""
ROS 2 node: live table detection from the scene camera (Sheppy) or the wrist RealSense.

Which camera is picked by TABLE_CAMERA (default "scene"); see CAMERAS below for
the topics and floor-rejection values each one uses.

Scene camera on Sheppy (behind the joystick, fixed to the chair; its TF to
base_link is published, so no arm move is needed):
    python3 table_detector_node.py
Its depth isn't aligned to color, so the overlay is drawn on a grayscale depth picture.

Wrist camera on the old rig - start it first (from the RAMMP Demo-Software bringup):
    ros2 launch rammp_prototype_bringup camera.launch.py \
        params_file:=$(ros2 pkg prefix rammp_prototype_bringup --share)/config/camera_wrist.yaml
    TABLE_CAMERA=wrist python3 table_detector_node.py
It uses the depth image aligned to color (so red lands on the real photo).

Override topics if yours are named differently:
    python3 table_detector_node.py --ros-args \
        -p depth_topic:=/camera/wrist/depth/image_rect_raw \
        -p info_topic:=/camera/wrist/depth/camera_info -p color_topic:=""

It also looks for a clear spot on the table where a round bowl could be set down
(see the "free space" notes at the top of table_detect.py) and shows it as a green
circle on the overlay plus a green cylinder marker. The bowl size (5.5 cm radius)
and the safety gaps (2 cm from objects, 8 cm from the table edge) are set at the top
of table_detect.py - change them there and this node picks them up.

Watch the result:
    ros2 run rqt_image_view rqt_image_view /table_detector/overlay
    rviz2   (add a Marker display on /table_detector/marker)
"""

import os

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker

from table_detect import (MAX_PLANE_DIST, MIN_TABLE_HEIGHT, depth_to_color, detect_table,
                          overlay_image, plane_axes)

# Default topics and floor rejection per camera; TABLE_CAMERA picks one.
CAMERAS = {
    # Sheppy, measured at HERL 2026-10-08: the camera is 0.35 m above base_link, the
    # table ~0.37 m below the camera at z~-0.02 in base_link. Depth is not aligned to color.
    "scene": dict(depth_topic="/scene_camera/depth/image_raw",
                  info_topic="/scene_camera/depth/camera_info",
                  color_topic="/scene_camera/color/image_raw",
                  max_plane_dist=0.6, min_table_height=-0.2),
    # Old rig's wrist RealSense at the scan pose (see table_detect.py).
    "wrist": dict(depth_topic="/camera/wrist/aligned_depth_to_color/image_raw",
                  info_topic="/camera/wrist/aligned_depth_to_color/camera_info",
                  color_topic="/camera/wrist/color/image_raw",
                  max_plane_dist=MAX_PLANE_DIST,
                  min_table_height=float("nan") if MIN_TABLE_HEIGHT is None else MIN_TABLE_HEIGHT),
}
CAMERA_NAME = os.environ.get("TABLE_CAMERA", "scene")
if CAMERA_NAME not in CAMERAS:
    raise SystemExit(f"TABLE_CAMERA={CAMERA_NAME!r} is unknown (known: {', '.join(CAMERAS)}).")
CAMERA = CAMERAS[CAMERA_NAME]


def image_to_numpy(msg):
    """sensor_msgs/Image -> numpy array (no cv_bridge needed)."""
    dtypes = {"16UC1": np.uint16, "mono16": np.uint16, "32FC1": np.float32,
              "rgb8": np.uint8, "bgr8": np.uint8, "mono8": np.uint8}
    channels = 3 if msg.encoding in ("rgb8", "bgr8") else 1
    dtype = np.dtype(dtypes[msg.encoding])
    if msg.is_bigendian:
        dtype = dtype.newbyteorder(">")
    row = np.frombuffer(bytes(msg.data), dtype=dtype).reshape(msg.height, -1)
    img = row[:, : msg.width * channels].reshape(msg.height, msg.width, channels)
    img = img[..., 0] if channels == 1 else img
    if msg.encoding == "bgr8":
        img = img[..., ::-1]
    return img


def depth_to_meters(msg):
    """RealSense publishes depth as 16UC1 in millimeters; some drivers use 32FC1 meters."""
    depth = image_to_numpy(msg)
    if depth.dtype == np.uint16:
        return depth.astype(np.float32) / 1000.0
    return np.nan_to_num(depth.astype(np.float32), nan=0.0)


class TableDetector(Node):
    def __init__(self):
        super().__init__("table_detector")
        # Defaults come from CAMERAS[TABLE_CAMERA].
        self.declare_parameter("depth_topic", CAMERA["depth_topic"])
        self.declare_parameter("info_topic", CAMERA["info_topic"])
        # Color image to draw the red table on. If it doesn't match the depth size
        # (unaligned depth), the overlay falls back to a grayscale depth picture.
        self.declare_parameter("color_topic", CAMERA["color_topic"])
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("rate_hz", 2.0)
        # Floor rejection. max_plane_dist: planes farther than this from the camera
        # (m) are never the table. min_table_height: planes lower than this in
        # base_frame (m) are never the table; needs TF, NaN = off.
        self.declare_parameter("max_plane_dist", CAMERA["max_plane_dist"])
        self.declare_parameter("min_table_height", CAMERA["min_table_height"])
        # The free-space search (bowl size, safety gaps, grid resolution) is fixed
        # at the top of table_detect.py - edit it there, not here.

        self.depth_msg = None
        self.info_msg = None
        self.color_msg = None

        self.create_subscription(Image, self.get_parameter("depth_topic").value, self._on_depth, 5)
        self.create_subscription(CameraInfo, self.get_parameter("info_topic").value, self._on_info, 5)
        color_topic = self.get_parameter("color_topic").value
        if color_topic:
            self.create_subscription(Image, color_topic, self._on_color, 5)

        self.overlay_pub = self.create_publisher(Image, "/table_detector/overlay", 5)
        self.marker_pub = self.create_publisher(Marker, "/table_detector/marker", 5)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.warned_no_tf = False

        self.create_timer(1.0 / self.get_parameter("rate_hz").value, self._tick)
        self.create_timer(5.0, self._check_inputs)
        self.get_logger().info(f"Table detector started ({CAMERA_NAME} camera), waiting for depth images...")

    def _check_inputs(self):
        """Complain (every 5 s) if the camera topics are silent - usually a wrong topic name."""
        missing = [name for name, msg in (("depth_topic", self.depth_msg), ("info_topic", self.info_msg))
                   if msg is None]
        if missing:
            topics = ", ".join(self.get_parameter(m).value for m in missing)
            self.get_logger().warn(
                f"No messages yet on: {topics}. Is the camera running? "
                "Check names with: ros2 topic list | grep camera")

    def _on_depth(self, msg):
        self.depth_msg = msg

    def _on_info(self, msg):
        self.info_msg = msg

    def _on_color(self, msg):
        self.color_msg = msg

    def _camera_to_base(self, camera_frame):
        """Rotation + translation taking camera-frame points into base_frame, or None."""
        base = self.get_parameter("base_frame").value
        try:
            tf = self.tf_buffer.lookup_transform(base, camera_frame, rclpy.time.Time())
        except Exception as e:  # TF not published / frames not connected
            if not self.warned_no_tf:
                self.get_logger().warn(
                    f"No TF {camera_frame} -> {base} ({e}). Falling back to 'largest plane'.")
                self.warned_no_tf = True
            return None
        q = tf.transform.rotation
        t = tf.transform.translation
        R = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        return R, np.array([t.x, t.y, t.z])

    def _tick(self):
        if self.depth_msg is None or self.info_msg is None:
            return
        depth_msg, info = self.depth_msg, self.info_msg
        depth_m = depth_to_meters(depth_msg)
        K = info.k  # [fx 0 cx; 0 fy cy; 0 0 1]
        intr = dict(fx=K[0], fy=K[4], cx=K[2], cy=K[5])

        # "Up" in the camera frame = base_link's +z axis rotated into the camera frame.
        cam_to_base = self._camera_to_base(depth_msg.header.frame_id)
        up = None if cam_to_base is None else cam_to_base[0].T @ np.array([0.0, 0.0, 1.0])

        min_height = self.get_parameter("min_table_height").value
        result = detect_table(depth_m, intr, up=up, fast=True,
                              max_plane_dist=self.get_parameter("max_plane_dist").value,
                              cam_to_base=cam_to_base,
                              min_height=None if np.isnan(min_height) else min_height)
        if result is None:
            self.get_logger().info("No table in view", throttle_duration_sec=2.0)
            self._publish_overlay(depth_msg, depth_m, np.zeros(depth_m.shape, bool), intr)
            self._clear_placement_marker(depth_msg.header)
            return

        c, size = result["center"], result["size"]
        if cam_to_base is not None:
            R, t = cam_to_base
            c_base = R @ c + t
            self.get_logger().info(
                f"Table at x={c_base[0]:.2f} y={c_base[1]:.2f} m, "
                f"surface height z={c_base[2]:.2f} m in {self.get_parameter('base_frame').value}, "
                f"{result['mask'].mean() * 100:.0f}% of image",
                throttle_duration_sec=1.0)
        else:
            self.get_logger().info(
                f"Table {result['dist']:.2f} m from camera (camera frame center "
                f"{np.round(c, 2).tolist()}), {result['mask'].mean() * 100:.0f}% of image",
                throttle_duration_sec=1.0)

        self._log_placement(result, cam_to_base)
        self._publish_overlay(depth_msg, depth_m, result["mask"], intr,
                              result["obstacle_mask"], result["placement"])
        self._publish_marker(depth_msg.header, result)
        self._publish_placement_marker(depth_msg.header, result["placement"])

    def _log_placement(self, result, cam_to_base):
        """Same pattern as the table log line: base_link when TF is up, camera frame otherwise."""
        placement = result["placement"]
        if placement is None:
            self.get_logger().info(f"No room for the bowl: {result['placement_reason']}",
                                   throttle_duration_sec=2.0)
            return
        p, r = placement["point"], placement["radius"]
        clear = placement["clearance"]
        if cam_to_base is not None:
            R, t = cam_to_base
            p_base = R @ p + t
            self.get_logger().info(
                f"Bowl spot at x={p_base[0]:.2f} y={p_base[1]:.2f} z={p_base[2]:.2f} m in "
                f"{self.get_parameter('base_frame').value}, radius {r * 100:.1f} cm, "
                f"{clear * 100:.1f} cm clear",
                throttle_duration_sec=1.0)
        else:
            self.get_logger().info(
                f"Bowl spot (camera frame) {np.round(p, 3).tolist()} m, "
                f"radius {r * 100:.1f} cm, {clear * 100:.1f} cm clear",
                throttle_duration_sec=1.0)

    def _publish_overlay(self, depth_msg, depth_m, mask, intrinsics=None,
                         obstacle_mask=None, placement=None):
        base = None
        if self.color_msg is not None:
            color = image_to_numpy(self.color_msg)
            if color.shape[:2] == depth_m.shape:
                base = color
        if base is None:
            base = depth_to_color(depth_m)
        img = overlay_image(base, mask, obstacle_mask=obstacle_mask,
                            placement=placement, intrinsics=intrinsics)
        msg = Image()
        msg.header = depth_msg.header
        msg.height, msg.width = img.shape[:2]
        msg.encoding = "rgb8"
        msg.step = msg.width * 3
        msg.data = img.tobytes()
        self.overlay_pub.publish(msg)

    def _publish_marker(self, header, result):
        """Thin red box lying on the table, in the camera frame."""
        n = result["normal"]
        # Rotation whose z-axis is the table normal.
        x_axis, y_axis = plane_axes(n)
        quat = Rotation.from_matrix(np.column_stack([x_axis, y_axis, n])).as_quat()

        # Size of the table measured along the in-plane axes.
        rel = result["points"] - result["center"]
        span_x = np.ptp(np.percentile(rel @ x_axis, [2, 98]))
        span_y = np.ptp(np.percentile(rel @ y_axis, [2, 98]))

        m = Marker()
        m.header = header
        m.ns, m.id, m.type, m.action = "table", 0, Marker.CUBE, Marker.ADD
        m.pose.position = Point(x=float(result["center"][0]), y=float(result["center"][1]),
                                z=float(result["center"][2]))
        m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = (
            float(v) for v in quat)
        m.scale.x, m.scale.y, m.scale.z = float(span_x), float(span_y), 0.01
        m.color.r, m.color.a = 1.0, 0.6
        self.marker_pub.publish(m)

    def _publish_placement_marker(self, header, placement):
        """Flat green cylinder (id 1) sitting where the bowl would go."""
        if placement is None:
            self._clear_placement_marker(header)
            return
        n = placement["normal"]
        x_axis, y_axis = plane_axes(n)
        quat = Rotation.from_matrix(np.column_stack([x_axis, y_axis, n])).as_quat()
        p, r = placement["point"], placement["radius"]

        m = Marker()
        m.header = header
        m.ns, m.id, m.type, m.action = "table", 1, Marker.CYLINDER, Marker.ADD
        m.pose.position = Point(x=float(p[0]), y=float(p[1]), z=float(p[2]))
        m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = (
            float(v) for v in quat)
        m.scale.x = m.scale.y = float(2.0 * r)  # cylinder scale x/y is the diameter
        m.scale.z = 0.02
        m.color.g, m.color.a = 1.0, 0.6
        self.marker_pub.publish(m)

    def _clear_placement_marker(self, header):
        """Remove a stale green circle when there is nowhere to put the bowl."""
        m = Marker()
        m.header = header
        m.ns, m.id, m.action = "table", 1, Marker.DELETE
        self.marker_pub.publish(m)


def main():
    rclpy.init()
    node = TableDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():  # Ctrl-C may have shut the context down already
            rclpy.shutdown()


if __name__ == "__main__":
    main()
