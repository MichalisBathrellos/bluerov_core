#!/usr/bin/env python3
"""
BlueROV2 AprilTag Camera Localization ROS 2 Node (Jazzy)
Node: rov_localization_node
Topics Published:
  - /rov/pose (geometry_msgs/msg/PoseStamped)
  - /rov/pose_human_readable (std_msgs/msg/String)
"""

import time
import cv2
import gi
import numpy as np
import pupil_apriltags as apriltag

gi.require_version("Gst", "1.0")
from gi.repository import Gst

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

# ==============================================================================
# 1. GLOBAL CONFIGURATION & TAG MAP
# ==============================================================================
REFERENCE_TAG_ID = 0  # The ID that defines the origin of the coordinate system (0,0,0)
TAG_SIZE_METERS = 0.06  # Length of the black square side in meters
TAG_FAMILY = "tagStandard41h12"

# Χάρτης γνωστών Tags ως προς το Reference Tag Frame:
TAG_MAP = {
    0: {
        "pos": [0.0, 0.0, 0.0],
        "rpy": [0.0, 0.0, 0.0],
    },
    1: {
        "pos": [0.21, 0.0, 0.0],
        "rpy": [0.0, 0.0, 0.0],
    },
    2: {
        "pos": [0.0, 0.40, 0.0],
        "rpy": [0.0, 0.0, 0.0],
    },
}

# ==============================================================================
# 2. BLUEROV2 CAMERA INTRINSICS (CALIBRATION)
# ==============================================================================
FRAME_WIDTH = 1920
FRAME_HEIGHT = 1080

FX = 1200.0
FY = 1200.0
CX = FRAME_WIDTH / 2.0
CY = FRAME_HEIGHT / 2.0

K = np.array([[FX, 0.0, CX], [0.0, FY, CY], [0.0, 0.0, 1.0]], dtype=np.float64)
D = np.zeros(5, dtype=np.float64)

CAMERA_TO_BODY_OFFSET = np.array([[0.0], [0.0], [-0.00]])


# ==============================================================================
# 3. BLUEROV2 VIDEO CAPTURE CLASS (GSTREAMER)
# ==============================================================================
class Video:
    """BlueROV video capture class using GStreamer appsink."""

    def __init__(self, port=5600):
        Gst.init(None)

        self.port = port
        self.latest_frame = None
        self._new_frame = None

        self.video_source = f"udpsrc port={self.port}"
        self.video_codec = (
            "! application/x-rtp, payload=96 ! rtph264depay ! h264parse !"
            " openh264dec"
        )
        self.video_decode = (
            "! decodebin ! videoconvert ! video/x-raw,format=(string)BGR !"
            " videoconvert"
        )
        self.video_sink_conf = (
            "! appsink emit-signals=true sync=false max-buffers=2 drop=true"
        )

        self.video_pipe = None
        self.video_sink = None

        self.run()

    def start_gst(self, config=None):
        if not config:
            config = [
                "videotestsrc ! decodebin",
                "! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert",
                "! appsink",
            ]

        command = " ".join(config)
        self.video_pipe = Gst.parse_launch(command)
        self.video_pipe.set_state(Gst.State.PLAYING)
        self.video_sink = self.video_pipe.get_by_name("appsink0")

    @staticmethod
    def gst_to_opencv(sample):
        buf = sample.get_buffer()
        caps_structure = sample.get_caps().get_structure(0)
        array = np.ndarray(
            (
                caps_structure.get_value("height"),
                caps_structure.get_value("width"),
                3,
            ),
            buffer=buf.extract_dup(0, buf.get_size()),
            dtype=np.uint8,
        )
        return array

    def frame_available(self):
        return self._new_frame is not None

    def frame(self):
        if self.frame_available():
            self.latest_frame = self._new_frame
            self._new_frame = None
        return self.latest_frame

    def run(self):
        self.start_gst([
            self.video_source,
            self.video_codec,
            self.video_decode,
            self.video_sink_conf,
        ])
        self.video_sink.connect("new-sample", self.callback)

    def callback(self, sink):
        sample = sink.emit("pull-sample")
        self._new_frame = self.gst_to_opencv(sample)
        return Gst.FlowReturn.OK

    def stop(self):
        if self.video_pipe:
            self.video_pipe.set_state(Gst.State.NULL)


# ==============================================================================
# 4. HELPER FUNCTIONS: ROTATIONS & 3D CORNER GENERATION
# ==============================================================================
def rpy_to_rotation_matrix(rpy_deg):
    r_rad, p_rad, y_rad = np.radians(rpy_deg)

    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(r_rad), -np.sin(r_rad)],
        [0, np.sin(r_rad), np.cos(r_rad)],
    ])
    Ry = np.array([
        [np.cos(p_rad), 0, np.sin(p_rad)],
        [0, 1, 0],
        [-np.sin(p_rad), 0, np.cos(p_rad)],
    ])
    Rz = np.array([
        [np.cos(y_rad), -np.sin(y_rad), 0],
        [np.sin(y_rad), np.cos(y_rad), 0],
        [0, 0, 1],
    ])
    return Rz @ Ry @ Rx


def rotation_matrix_to_rpy(R):
    sy = np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    singular = sy < 1e-6
    if not singular:
        roll = np.arctan2(R[2, 1], R[2, 2])
        pitch = np.arctan2(-R[2, 0], sy)
        yaw = np.arctan2(R[1, 0], R[0, 0])
    else:
        roll = np.arctan2(-R[1, 2], R[1, 1])
        pitch = np.arctan2(-R[2, 0], sy)
        yaw = 0.0
    return np.degrees([roll, pitch, yaw])


def rotation_matrix_to_quaternion(R):
    """Μετατροπή 3x3 Rotation Matrix σε Quaternion (qx, qy, qz, qw)."""
    tr = np.trace(R)
    if tr > 0:
        S = np.sqrt(tr + 1.0) * 2.0
        qw = 0.25 * S
        qx = (R[2, 1] - R[1, 2]) / S
        qy = (R[0, 2] - R[2, 0]) / S
        qz = (R[1, 0] - R[0, 1]) / S
    elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
        S = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        qw = (R[2, 1] - R[1, 2]) / S
        qx = 0.25 * S
        qy = (R[0, 1] + R[1, 0]) / S
        qz = (R[0, 2] + R[2, 0]) / S
    elif R[1, 1] > R[2, 2]:
        S = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        qw = (R[0, 2] - R[2, 0]) / S
        qx = (R[0, 1] + R[1, 0]) / S
        qy = 0.25 * S
        qz = (R[1, 2] + R[2, 1]) / S
    else:
        S = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        qw = (R[1, 0] - R[0, 1]) / S
        qx = (R[0, 2] + R[2, 0]) / S
        qy = (R[1, 2] + R[2, 1]) / S
        qz = 0.25 * S

    return qx, qy, qz, qw


def build_global_tag_corners(tag_map, tag_size):
    s = tag_size / 2.0
    local_corners = np.array(
        [
            [-s, s, 0.0],
            [s, s, 0.0],
            [s, -s, 0.0],
            [-s, -s, 0.0],
        ],
        dtype=np.float32,
    )

    global_corners_map = {}
    for tag_id, transform in tag_map.items():
        R = rpy_to_rotation_matrix(transform["rpy"])
        t = np.array(transform["pos"], dtype=np.float32).reshape(3, 1)

        corners_global = (R @ local_corners.T + t).T
        global_corners_map[tag_id] = corners_global

    return global_corners_map


GLOBAL_TAG_CORNERS = build_global_tag_corners(TAG_MAP, TAG_SIZE_METERS)


# ==============================================================================
# 5. ROS 2 NODE CLASS
# ==============================================================================
class BlueROVLocalizationNode(Node):

    def __init__(self):
        super().__init__("localization_node")

        # Δημοσίευση ROS Parameters
        self.declare_parameter("udp_port", 5600)
        self.declare_parameter("world_frame_id", "world")
        self.port = self.get_parameter("udp_port").get_parameter_value().integer_value
        self.world_frame = (
            self.get_parameter("world_frame_id").get_parameter_value().string_value
        )

        # Publishers
        self.pose_pub = self.create_publisher(PoseStamped, "rov/pose", 10)
        self.human_pub = self.create_publisher(String, "rov/pose_human_readable", 10)

        # Video stream & AprilTag detector
        self.video = Video(port=self.port)
        self.detector = apriltag.Detector(
            families=TAG_FAMILY,
            nthreads=4,
            quad_decimate=1.0,
            quad_sigma=0.0,
            refine_edges=1,
        )

        self.get_logger().info(f"Connecting to UDP stream on port {self.port}...")
        waited = 0
        while not self.video.frame_available() and rclpy.ok():
            waited += 1
            time.sleep(0.05)
        self.get_logger().info("Video stream connected successfully!")

        # Timer loop (30 Hz)
        self.timer = self.create_timer(1.0 / 30.0, self.process_frame)

    def process_frame(self):
        if not self.video.frame_available():
            cv2.waitKey(1)
            return

        frame = self.video.frame()
        if frame is None:
            return
        frame = frame.copy()

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        results = self.detector.detect(gray)

        all_obj_pts = []
        all_img_pts = []
        detected_known_ids = []

        for r in results:
            tag_id = r.tag_id
            corners = r.corners.astype(int)

            for i in range(4):
                cv2.line(
                    frame,
                    tuple(corners[i]),
                    tuple(corners[(i + 1) % 4]),
                    (0, 255, 0),
                    2,
                )
            cv2.putText(
                frame,
                f"ID:{tag_id}",
                (corners[0][0], corners[0][1] - 8),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2,
            )

            if tag_id in GLOBAL_TAG_CORNERS:
                all_obj_pts.append(GLOBAL_TAG_CORNERS[tag_id])
                all_img_pts.append(r.corners)
                detected_known_ids.append(tag_id)

        if len(detected_known_ids) > 0:
            obj_pts = np.vstack(all_obj_pts).astype(np.float64)
            img_pts = np.vstack(all_img_pts).astype(np.float64)

            if len(detected_known_ids) >= 2:
                success, rvec, tvec, inliers = cv2.solvePnPRansac(
                    obj_pts,
                    img_pts,
                    K,
                    D,
                    reprojectionError=8.0,
                    flags=cv2.SOLVEPNP_SQPNP,
                )
            else:
                success, rvec, tvec = cv2.solvePnP(
                    obj_pts, img_pts, K, D, flags=cv2.SOLVEPNP_IPPE_SQUARE
                )

            if success:
                # 1. World -> Camera (C_T_W)
                R_cw, _ = cv2.Rodrigues(rvec)
                t_cw = tvec.reshape(3, 1)

                # 2. Camera -> World (W_T_C)
                R_wc = R_cw.T
                t_wc = -R_wc @ t_cw

                # 3. Μετατόπιση στο Body Frame (World Frame)
                t_wb = t_wc + R_wc @ CAMERA_TO_BODY_OFFSET

                rov_x, rov_y, rov_z = t_wb.flatten()
                rov_roll, rov_pitch, rov_yaw = rotation_matrix_to_rpy(R_wc)
                qx, qy, qz, qw = rotation_matrix_to_quaternion(R_wc)

                stamp = self.get_clock().now().to_msg()

                # Publisher 1: PoseStamped (Quaternions)
                pose_msg = PoseStamped()
                pose_msg.header.stamp = stamp
                pose_msg.header.frame_id = self.world_frame
                pose_msg.pose.position.x = float(rov_x)
                pose_msg.pose.position.y = float(rov_y)
                pose_msg.pose.position.z = float(rov_z)
                pose_msg.pose.orientation.x = float(qx)
                pose_msg.pose.orientation.y = float(qy)
                pose_msg.pose.orientation.z = float(qz)
                pose_msg.pose.orientation.w = float(qw)
                self.pose_pub.publish(pose_msg)

                # Publisher 2: Human-readable String format
                human_msg = String()
                human_msg.data = (
                    f"Position [m]: X={rov_x:+.3f}, Y={rov_y:+.3f}, Z={rov_z:+.3f} | "
                    f"Orientation [deg]: Roll={rov_roll:+.1f}, Pitch={rov_pitch:+.1f}, Yaw={rov_yaw:+.1f}"
                )
                self.human_pub.publish(human_msg)

                # OSD Text Rendering
                status_text = (
                    f"Tags used ({len(detected_known_ids)}): {detected_known_ids}"
                )
                pos_text = (
                    f"ROV Pos (m)  : X={rov_x:+.2f}, Y={rov_y:+.2f}, Z={rov_z:+.2f}"
                )
                rot_text = (
                    f"ROV Ori (deg): R={rov_roll:+.0f}, P={rov_pitch:+.0f},"
                    f" Y={rov_yaw:+.0f}"
                )

                cv2.putText(
                    frame,
                    status_text,
                    (30, 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 255),
                    2,
                )
                cv2.putText(
                    frame,
                    pos_text,
                    (30, 75),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 0),
                    2,
                )
                cv2.putText(
                    frame,
                    rot_text,
                    (30, 110),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 0),
                    2,
                )
            else:
                cv2.putText(
                    frame,
                    "PnP Solver Failed",
                    (30, 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 0, 255),
                    2,
                )
        else:
            cv2.putText(
                frame,
                "No known tags in view",
                (30, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 255),
                2,
            )

        cv2.imshow("BlueROV2 Localization (Global Multi-Tag PnP)", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            self.get_logger().info("Exit key pressed.")
            rclpy.shutdown()

    def destroy_node(self):
        self.video.stop()
        cv2.destroyAllWindows()
        super().destroy_node()


# ==============================================================================
# 6. MAIN EXECUTION
# ==============================================================================
def main(args=None):
    rclpy.init(args=args)
    node = BlueROVLocalizationNode()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()