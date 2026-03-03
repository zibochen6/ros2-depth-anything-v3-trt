#!/usr/bin/env python3
"""Compose camera/depth/point cloud panels and stream as RTSP (H264)."""

import math
import threading
from typing import Optional, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField

GI_IMPORT_ERROR = None
try:
    import gi

    gi.require_version("Gst", "1.0")
    gi.require_version("GstRtspServer", "1.0")
    from gi.repository import GLib
    from gi.repository import Gst
    from gi.repository import GstRtspServer
except Exception as exc:  # pragma: no cover - runtime dependency check
    GI_IMPORT_ERROR = exc
    GLib = None
    Gst = None
    GstRtspServer = None


def stamp_to_tuple(msg) -> Tuple[int, int]:
    """Convert ROS header stamp to a hashable tuple."""
    return (int(msg.header.stamp.sec), int(msg.header.stamp.nanosec))


if GstRtspServer is not None:

    class MosaicRtspFactory(GstRtspServer.RTSPMediaFactory):
        """GStreamer RTSP media factory fed by latest mosaic frame."""

        def __init__(self, owner: "DepthMosaicRtspNode", width: int, height: int, fps: int, launch: str):
            super().__init__()
            self._owner = owner
            self._width = int(width)
            self._height = int(height)
            self._fps = max(1, int(fps))
            self._launch = launch
            self._frame_count = 0
            self._frame_duration = Gst.util_uint64_scale_int(1, Gst.SECOND, self._fps)

        def do_create_element(self, _url):
            return Gst.parse_launch(self._launch)

        def do_configure(self, media):
            self._frame_count = 0
            element = media.get_element()
            appsrc = element.get_child_by_name("src")
            if appsrc is None:
                appsrc = element.get_by_name("src")
            if appsrc is not None:
                appsrc.connect("need-data", self._on_need_data)
            else:
                self._owner.get_logger().error("Failed to locate appsrc element in RTSP pipeline")

        def _on_need_data(self, src, _length):
            frame = self._owner.get_stream_frame(self._width, self._height)
            if frame is None:
                return
            payload = frame.tobytes()
            gst_buffer = Gst.Buffer.new_allocate(None, len(payload), None)
            gst_buffer.fill(0, payload)
            gst_buffer.duration = self._frame_duration
            timestamp = self._frame_count * self._frame_duration
            gst_buffer.pts = timestamp
            gst_buffer.dts = timestamp
            gst_buffer.offset = timestamp
            self._frame_count += 1

            result = src.emit("push-buffer", gst_buffer)
            if result != Gst.FlowReturn.OK:
                self._owner.get_logger().debug(f"push-buffer returned {result}")

else:

    class MosaicRtspFactory:  # pragma: no cover - only used when GI missing
        def __init__(self, *_args, **_kwargs):
            raise RuntimeError("GStreamer RTSP bindings are unavailable")


class DepthMosaicRtspNode(Node):
    """ROS2 node: subscribe RGB/depth/point cloud, compose mosaic, stream via RTSP."""

    def __init__(self):
        super().__init__("depth_mosaic_rtsp_node")

        if GI_IMPORT_ERROR is not None:
            raise RuntimeError(
                "Missing GStreamer RTSP Python bindings. Install with:\n"
                "  sudo apt install -y python3-gi python3-gst-1.0 "
                "gir1.2-gst-rtsp-server-1.0 gstreamer1.0-tools "
                "gstreamer1.0-plugins-base gstreamer1.0-plugins-good "
                "gstreamer1.0-plugins-bad gstreamer1.0-libav\n"
                f"Import error: {GI_IMPORT_ERROR}"
            )

        self._declare_parameters()
        self._load_parameters()

        self._bridge = CvBridge()
        self._lock = threading.Lock()

        self._latest_image_msg: Optional[Image] = None
        self._latest_depth_msg: Optional[Image] = None
        self._latest_cloud_msg: Optional[PointCloud2] = None

        self._cached_image_stamp: Optional[Tuple[int, int]] = None
        self._cached_depth_stamp: Optional[Tuple[int, int]] = None
        self._cached_cloud_stamp: Optional[Tuple[int, int]] = None

        self._cached_image_frame: Optional[np.ndarray] = None
        self._cached_depth_frame: Optional[np.ndarray] = None
        self._cached_cloud_points: Optional[np.ndarray] = None

        self._latest_mosaic: Optional[np.ndarray] = None
        self._panel_width_runtime: Optional[int] = self.panel_width if self.panel_width > 0 else None
        self._panel_height_runtime: Optional[int] = self.panel_height if self.panel_height > 0 else None
        self._panel_wait_ticks = 0
        self._panel_wait_limit = max(1, int(self.stream_fps * 2))

        self._rtsp_started = False
        self._rtsp_failed = False
        self._rtsp_factory = None
        self._rtsp_server = None
        self._rtsp_loop = None
        self._rtsp_thread = None
        self._warned_cloud_dtype = False

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.create_subscription(Image, self.input_image_topic, self._image_callback, qos)
        self.create_subscription(Image, self.input_depth_topic, self._depth_callback, qos)
        self.create_subscription(PointCloud2, self.input_pointcloud_topic, self._pointcloud_callback, qos)
        self._mosaic_pub = self.create_publisher(Image, self.output_mosaic_topic, 10)

        period_sec = 1.0 / float(max(1, self.stream_fps))
        self._timer = self.create_timer(period_sec, self._timer_callback)

        Gst.init(None)
        self.get_logger().info("depth_mosaic_rtsp_node initialized")
        self.get_logger().info(f"Input topics: {self.input_image_topic}, {self.input_depth_topic}, {self.input_pointcloud_topic}")
        self.get_logger().info(f"Output mosaic topic: {self.output_mosaic_topic}")

    def _declare_parameters(self):
        self.declare_parameter("input_image_topic", "/camera/image")
        self.declare_parameter("input_depth_topic", "/camera/depth_colored")
        self.declare_parameter("input_pointcloud_topic", "/camera/point_cloud")
        self.declare_parameter("output_mosaic_topic", "/camera/depth_mosaic")
        self.declare_parameter("stream_fps", 15)
        self.declare_parameter("panel_height", -1)
        self.declare_parameter("panel_width", -1)
        self.declare_parameter("max_points", 30000)
        self.declare_parameter("view_yaw_deg", 35.0)
        self.declare_parameter("view_pitch_deg", 20.0)
        self.declare_parameter("point_radius", 1)
        self.declare_parameter("rtsp_port", 8554)
        self.declare_parameter("rtsp_mount", "/depth")
        self.declare_parameter("encoder", "auto")
        self.declare_parameter("bitrate_kbps", 4000)

    def _load_parameters(self):
        self.input_image_topic = self.get_parameter("input_image_topic").value
        self.input_depth_topic = self.get_parameter("input_depth_topic").value
        self.input_pointcloud_topic = self.get_parameter("input_pointcloud_topic").value
        self.output_mosaic_topic = self.get_parameter("output_mosaic_topic").value
        self.stream_fps = int(self.get_parameter("stream_fps").value)
        self.panel_height = int(self.get_parameter("panel_height").value)
        self.panel_width = int(self.get_parameter("panel_width").value)
        self.max_points = int(self.get_parameter("max_points").value)
        self.view_yaw_deg = float(self.get_parameter("view_yaw_deg").value)
        self.view_pitch_deg = float(self.get_parameter("view_pitch_deg").value)
        self.point_radius = int(self.get_parameter("point_radius").value)
        self.rtsp_port = int(self.get_parameter("rtsp_port").value)
        self.rtsp_mount = str(self.get_parameter("rtsp_mount").value)
        self.encoder = str(self.get_parameter("encoder").value).lower()
        self.bitrate_kbps = int(self.get_parameter("bitrate_kbps").value)

        self.stream_fps = max(1, self.stream_fps)
        self.max_points = max(1, self.max_points)
        self.point_radius = max(1, self.point_radius)
        self.bitrate_kbps = max(500, self.bitrate_kbps)
        if not self.rtsp_mount.startswith("/"):
            self.rtsp_mount = f"/{self.rtsp_mount}"

    def _image_callback(self, msg: Image):
        with self._lock:
            self._latest_image_msg = msg

    def _depth_callback(self, msg: Image):
        with self._lock:
            self._latest_depth_msg = msg

    def _pointcloud_callback(self, msg: PointCloud2):
        with self._lock:
            self._latest_cloud_msg = msg

    def _timer_callback(self):
        with self._lock:
            image_msg = self._latest_image_msg
            depth_msg = self._latest_depth_msg
            cloud_msg = self._latest_cloud_msg

        self._refresh_cached_frames(image_msg, depth_msg, cloud_msg)

        panel_size = self._resolve_panel_size()
        if panel_size is None:
            return
        panel_w, panel_h = panel_size

        if not self._rtsp_started and not self._rtsp_failed:
            try:
                self._start_rtsp_server(panel_w * 2, panel_h * 2)
            except Exception as exc:
                self._rtsp_failed = True
                self.get_logger().error(
                    f"Failed to start RTSP server: {exc}. "
                    "Check GStreamer plugins and encoder setting."
                )

        image_panel = self._prepare_image_panel(self._cached_image_frame, panel_w, panel_h, "Camera", "waiting image")
        depth_panel = self._prepare_image_panel(self._cached_depth_frame, panel_w, panel_h, "Depth", "waiting depth")
        point_panel = self._render_pointcloud_panel(self._cached_cloud_points, panel_w * 2, panel_h)
        point_panel = self._annotate_panel(point_panel, "Point Cloud", self._status_text(self._cached_cloud_points, "waiting cloud"))

        mosaic = np.zeros((panel_h * 2, panel_w * 2, 3), dtype=np.uint8)
        mosaic[0:panel_h, 0 : panel_w * 2] = point_panel
        mosaic[panel_h : panel_h * 2, 0:panel_w] = image_panel
        mosaic[panel_h : panel_h * 2, panel_w : panel_w * 2] = depth_panel

        stamp = self.get_clock().now().to_msg()
        mosaic_msg = self._bridge.cv2_to_imgmsg(mosaic, encoding="bgr8")
        mosaic_msg.header.stamp = stamp
        mosaic_msg.header.frame_id = "camera_link"
        self._mosaic_pub.publish(mosaic_msg)

        with self._lock:
            self._latest_mosaic = mosaic

    def _resolve_panel_size(self) -> Optional[Tuple[int, int]]:
        if self._panel_width_runtime is not None and self._panel_height_runtime is not None:
            return self._panel_width_runtime, self._panel_height_runtime

        candidate_w = None
        candidate_h = None
        if self._cached_image_frame is not None:
            candidate_h, candidate_w = self._cached_image_frame.shape[:2]
        elif self._cached_depth_frame is not None:
            candidate_h, candidate_w = self._cached_depth_frame.shape[:2]

        if candidate_w is None or candidate_h is None:
            if self._panel_wait_ticks < self._panel_wait_limit:
                self._panel_wait_ticks += 1
                return None
            candidate_w, candidate_h = 640, 480

        if self._panel_width_runtime is None:
            self._panel_width_runtime = int(max(64, candidate_w))
        if self._panel_height_runtime is None:
            self._panel_height_runtime = int(max(64, candidate_h))

        return self._panel_width_runtime, self._panel_height_runtime

    def _refresh_cached_frames(self, image_msg, depth_msg, cloud_msg):
        if image_msg is not None:
            stamp = stamp_to_tuple(image_msg)
            if stamp != self._cached_image_stamp:
                self._cached_image_frame = self._decode_to_bgr(image_msg)
                self._cached_image_stamp = stamp

        if depth_msg is not None:
            stamp = stamp_to_tuple(depth_msg)
            if stamp != self._cached_depth_stamp:
                self._cached_depth_frame = self._decode_to_bgr(depth_msg)
                self._cached_depth_stamp = stamp

        if cloud_msg is not None:
            stamp = stamp_to_tuple(cloud_msg)
            if stamp != self._cached_cloud_stamp:
                self._cached_cloud_points = self._extract_points(cloud_msg)
                self._cached_cloud_stamp = stamp

    def _decode_to_bgr(self, msg: Image) -> Optional[np.ndarray]:
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            return frame
        except Exception:
            pass

        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as exc:
            self.get_logger().warning(f"Failed to decode image '{msg.encoding}': {exc}")
            return None

        if frame is None:
            return None

        if frame.ndim == 2:
            normalized = cv2.normalize(frame, None, 0, 255, cv2.NORM_MINMAX)
            u8 = normalized.astype(np.uint8)
            return cv2.cvtColor(u8, cv2.COLOR_GRAY2BGR)
        if frame.ndim == 3 and frame.shape[2] == 4:
            return cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
        if frame.ndim == 3 and frame.shape[2] == 3:
            if msg.encoding.lower() == "rgb8":
                return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            return frame
        return None

    def _extract_points(self, cloud_msg: PointCloud2) -> Optional[np.ndarray]:
        fields = {field.name: field for field in cloud_msg.fields}
        if "x" not in fields or "y" not in fields or "z" not in fields:
            return None

        x_field = fields["x"]
        y_field = fields["y"]
        z_field = fields["z"]
        if (
            x_field.datatype != PointField.FLOAT32
            or y_field.datatype != PointField.FLOAT32
            or z_field.datatype != PointField.FLOAT32
        ):
            if not self._warned_cloud_dtype:
                self.get_logger().warning("PointCloud2 x/y/z are not FLOAT32; skipping cloud rendering")
                self._warned_cloud_dtype = True
            return None

        if cloud_msg.point_step <= 0:
            return None

        count = int(cloud_msg.width) * int(cloud_msg.height)
        max_count = len(cloud_msg.data) // int(cloud_msg.point_step)
        count = min(count, max_count)
        if count <= 0:
            return None

        byte_order = ">" if cloud_msg.is_bigendian else "<"
        dtype = np.dtype(
            {
                "names": ["x", "y", "z"],
                "formats": [f"{byte_order}f4", f"{byte_order}f4", f"{byte_order}f4"],
                "offsets": [x_field.offset, y_field.offset, z_field.offset],
                "itemsize": cloud_msg.point_step,
            }
        )
        points_raw = np.frombuffer(cloud_msg.data, dtype=dtype, count=count)
        points = np.stack([points_raw["x"], points_raw["y"], points_raw["z"]], axis=1).astype(np.float32, copy=False)

        valid = np.isfinite(points).all(axis=1)
        valid &= points[:, 2] > 0.01
        valid &= points[:, 2] < 300.0
        points = points[valid]
        if points.size == 0:
            return None

        if points.shape[0] > self.max_points:
            indices = np.linspace(0, points.shape[0] - 1, self.max_points, dtype=np.int64)
            points = points[indices]

        return points

    def _prepare_image_panel(
        self, frame: Optional[np.ndarray], width: int, height: int, title: str, wait_text: str
    ) -> np.ndarray:
        if frame is None:
            panel = self._make_placeholder(width, height, title, wait_text)
            return panel
        panel = cv2.resize(frame, (width, height), interpolation=cv2.INTER_LINEAR)
        return self._annotate_panel(panel, title, "ok")

    def _render_pointcloud_panel(self, points: Optional[np.ndarray], width: int, height: int) -> np.ndarray:
        if points is None or points.shape[0] == 0:
            return self._make_placeholder(width, height, "Point Cloud", "waiting cloud")

        canvas = np.zeros((height, width, 3), dtype=np.uint8)
        canvas[:] = (15, 15, 15)

        pts = points.copy()
        center = np.median(pts, axis=0)
        pts_centered = pts - center

        yaw = math.radians(self.view_yaw_deg)
        pitch = math.radians(self.view_pitch_deg)

        rot_y = np.array(
            [
                [math.cos(yaw), 0.0, math.sin(yaw)],
                [0.0, 1.0, 0.0],
                [-math.sin(yaw), 0.0, math.cos(yaw)],
            ],
            dtype=np.float32,
        )
        rot_x = np.array(
            [
                [1.0, 0.0, 0.0],
                [0.0, math.cos(pitch), -math.sin(pitch)],
                [0.0, math.sin(pitch), math.cos(pitch)],
            ],
            dtype=np.float32,
        )
        rotated = (pts_centered @ rot_y.T) @ rot_x.T

        z_values = rotated[:, 2]
        z_shift = float(np.percentile(np.abs(z_values), 95)) + 3.0
        z_cam = z_values + z_shift
        valid = z_cam > 0.05
        if not np.any(valid):
            return self._make_placeholder(width, height, "Point Cloud", "no valid cloud")

        rotated = rotated[valid]
        z_cam = z_cam[valid]
        base_dist = np.linalg.norm(pts_centered[valid], axis=1)

        fx = width * 0.85
        fy = height * 0.85
        u = (fx * rotated[:, 0] / z_cam) + (width * 0.5)
        v = (fy * (-rotated[:, 1]) / z_cam) + (height * 0.5)

        u_i = u.astype(np.int32)
        v_i = v.astype(np.int32)
        in_view = (u_i >= 0) & (u_i < width) & (v_i >= 0) & (v_i < height)
        if not np.any(in_view):
            return self._make_placeholder(width, height, "Point Cloud", "no projected points")

        u_i = u_i[in_view]
        v_i = v_i[in_view]
        z_cam = z_cam[in_view]
        base_dist = base_dist[in_view]

        d_lo = float(np.percentile(base_dist, 5))
        d_hi = float(np.percentile(base_dist, 95))
        d_norm = np.clip((base_dist - d_lo) / max(1e-6, d_hi - d_lo), 0.0, 1.0)
        d_u8 = (d_norm * 255.0).astype(np.uint8)
        colors = cv2.applyColorMap(d_u8.reshape(-1, 1), cv2.COLORMAP_TURBO).reshape(-1, 3)

        order = np.argsort(z_cam)[::-1]  # draw far first, near last
        radius = max(1, self.point_radius)
        for idx in order:
            color = tuple(int(c) for c in colors[idx])
            px = int(u_i[idx])
            py = int(v_i[idx])
            if radius <= 1:
                canvas[py, px] = color
            else:
                cv2.circle(canvas, (px, py), radius, color, thickness=-1, lineType=cv2.LINE_AA)

        return canvas

    def _make_placeholder(self, width: int, height: int, title: str, status: str) -> np.ndarray:
        panel = np.zeros((height, width, 3), dtype=np.uint8)
        panel[:] = (25, 25, 25)
        return self._annotate_panel(panel, title, status)

    def _annotate_panel(self, panel: np.ndarray, title: str, status: str) -> np.ndarray:
        out = panel.copy()
        cv2.rectangle(out, (0, 0), (out.shape[1], 34), (0, 0, 0), thickness=-1)
        cv2.putText(
            out,
            title,
            (12, 22),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            out,
            status,
            (12, out.shape[0] - 14),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (180, 180, 180),
            1,
            cv2.LINE_AA,
        )
        return out

    def _status_text(self, value, waiting_text: str) -> str:
        return "ok" if value is not None else waiting_text

    def _resolve_encoder(self) -> str:
        requested = self.encoder
        if requested == "auto":
            if Gst.ElementFactory.find("nvv4l2h264enc") is not None:
                return "nvv4l2h264enc"
            if Gst.ElementFactory.find("x264enc") is not None:
                return "x264enc"
            raise RuntimeError("No H264 encoder found. Install nvv4l2h264enc or x264enc plugins.")

        if requested not in ("nvv4l2h264enc", "x264enc"):
            raise RuntimeError("Invalid encoder parameter. Use: auto, nvv4l2h264enc, x264enc")
        if Gst.ElementFactory.find(requested) is None:
            raise RuntimeError(f"Requested encoder '{requested}' is not available in GStreamer")
        return requested

    def _build_rtsp_launch(self, width: int, height: int, fps: int, encoder_name: str) -> str:
        common = (
            f"appsrc name=src is-live=true block=true format=time do-timestamp=true "
            f"caps=video/x-raw,format=BGR,width={width},height={height},framerate={fps}/1 "
            f"! queue leaky=downstream max-size-buffers=2 "
            "! videoconvert "
        )
        if encoder_name == "nvv4l2h264enc":
            bitrate_bps = self.bitrate_kbps * 1000
            encode = (
                "! video/x-raw,format=NV12 "
                f"! nvv4l2h264enc bitrate={bitrate_bps} control-rate=1 insert-sps-pps=true "
                f"iframeinterval={fps * 2} idrinterval={fps * 2} "
                "! h264parse config-interval=1 "
            )
        else:
            encode = (
                f"! x264enc tune=zerolatency speed-preset=ultrafast bitrate={self.bitrate_kbps} "
                f"key-int-max={fps * 2} "
                "! h264parse config-interval=1 "
            )
        pay = "! rtph264pay name=pay0 pt=96 config-interval=1"
        return common + encode + pay

    def _start_rtsp_server(self, mosaic_width: int, mosaic_height: int):
        encoder_name = self._resolve_encoder()
        launch_str = self._build_rtsp_launch(mosaic_width, mosaic_height, self.stream_fps, encoder_name)

        self._rtsp_factory = MosaicRtspFactory(self, mosaic_width, mosaic_height, self.stream_fps, launch_str)
        self._rtsp_factory.set_shared(True)

        self._rtsp_server = GstRtspServer.RTSPServer.new()
        self._rtsp_server.set_address("0.0.0.0")
        self._rtsp_server.set_service(str(self.rtsp_port))
        mounts = self._rtsp_server.get_mount_points()
        mounts.add_factory(self.rtsp_mount, self._rtsp_factory)
        self._rtsp_server.attach(None)

        self._rtsp_loop = GLib.MainLoop()
        self._rtsp_thread = threading.Thread(target=self._rtsp_loop.run, daemon=True)
        self._rtsp_thread.start()

        self._rtsp_started = True
        self.get_logger().info(f"RTSP encoder: {encoder_name}")
        self.get_logger().info(f"RTSP stream ready: rtsp://127.0.0.1:{self.rtsp_port}{self.rtsp_mount}")
        self.get_logger().info(f"LAN stream ready:  rtsp://<jetson-ip>:{self.rtsp_port}{self.rtsp_mount}")

    def get_stream_frame(self, width: int, height: int) -> Optional[np.ndarray]:
        with self._lock:
            if self._latest_mosaic is None:
                frame = self._make_placeholder(width, height, "Depth Mosaic", "waiting data")
            else:
                frame = self._latest_mosaic.copy()

        if frame.shape[1] != width or frame.shape[0] != height:
            frame = cv2.resize(frame, (width, height), interpolation=cv2.INTER_LINEAR)
        return frame

    def destroy_node(self):
        if self._rtsp_loop is not None and self._rtsp_loop.is_running():
            self._rtsp_loop.quit()
        if self._rtsp_thread is not None and self._rtsp_thread.is_alive():
            self._rtsp_thread.join(timeout=1.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = DepthMosaicRtspNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError as exc:
        print(f"[depth_mosaic_rtsp_node] {exc}")
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
