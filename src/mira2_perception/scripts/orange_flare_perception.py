#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import cv2
import numpy as np
from collections import deque

from geometry_msgs.msg import Point, PoseStamped
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class FlarePerceptionNode(Node):

    def __init__(self):
        super().__init__("flare_perception_node")
        self.get_logger().info("Flare Perception Node Started")

        self.bridge = CvBridge()
        self.frame  = None

        # ── Camera Matrix (640x480, FOV=75°) ──────────────────────────────
        fx = fy = 415.69
        cx, cy  = 320.0, 240.0
        self.K  = np.array([[fx,  0, cx],
                             [ 0, fy, cy],
                             [ 0,  0,  1]], dtype=np.float64)
        self.dist = np.zeros((4, 1), dtype=np.float64)

        # ── Pole 3D corners (from Pole.obj) ───────────────────────────────
        self.obj_pts = np.array([
            [-0.0473,  0.7956, 0.0],   # top-left
            [ 0.0473,  0.7956, 0.0],   # top-right
            [-0.0473, -0.7956, 0.0],   # bottom-left
            [ 0.0473, -0.7956, 0.0],   # bottom-right
        ], dtype=np.float32)

        # ── HSV range for orange flare ─────────────────────────────────────
        self.LOWER_ORANGE = np.array([60,  40,  40])
        self.UPPER_ORANGE = np.array([80, 255, 255])

        # ── Morphology kernels ─────────────────────────────────────────────
        self.kernel       = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 75))
        self.kernel_erode = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))

        # ── Smoothing history ──────────────────────────────────────────────
        self.SMOOTH_N     = 5
        self.pose_history = deque(maxlen=self.SMOOTH_N)

        # ── Publishers ────────────────────────────────────────────────────
        self.pose_pub  = self.create_publisher(PoseStamped, '/flare/pose',  10)
        self.debug_pub = self.create_publisher(String,      '/flare/debug', 10)

        # ── Subscriber ────────────────────────────────────────────────────
        self.frame_sub = self.create_subscription(
            Image, '/bluerov2/left/image_color', self.frame_callback, 10
        )

    def frame_callback(self, msg):
        self.frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        if self.frame is None:
            self.get_logger().warn("Frame conversion failed")
            return
        self.detect_flare()

    def color_correction(self, frame):
        b, g, r = cv2.split(frame)
        b = cv2.normalize(b, None, 0, 255, cv2.NORM_MINMAX)
        g = cv2.normalize(g, None, 0, 255, cv2.NORM_MINMAX)
        r = cv2.normalize(r, None, 0, 255, cv2.NORM_MINMAX)
        return cv2.merge((b, g, r))

    def apply_clahe(self, frame):
        lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)
        l = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8)).apply(l)
        return cv2.cvtColor(cv2.merge((l, a, b)), cv2.COLOR_LAB2BGR)

    def detect_flare(self):
        vis   = self.frame.copy()
        img_h, img_w = self.frame.shape[:2]

        # Preprocess
        frame_proc = self.color_correction(self.frame)
        frame_proc = self.apply_clahe(frame_proc)
        frame_proc = cv2.medianBlur(frame_proc, 3)

        # HSV mask
        hsv  = cv2.cvtColor(frame_proc, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.LOWER_ORANGE, self.UPPER_ORANGE)
        # Erode first to kill thin noise lines
        mask = cv2.erode(mask, self.kernel_erode, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)

        # Contours → pole candidates
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        pole_candidates = []
        for c in contours:
            if cv2.contourArea(c) < 50:
                continue
            x, y, w, h = cv2.boundingRect(c)
            if w > 50:                            # pole is thin — reject wide noise blobs
                continue
            if (h / w if w != 0 else 0) < 5:     # stricter aspect ratio
                continue
            pole_candidates.append((x, y, w, h))

        detected = False

        if pole_candidates:
            x, y, w, h = max(pole_candidates, key=lambda item: item[3])  # tallest

            img_pts = np.array([
                [x,     y    ],
                [x + w, y    ],
                [x,     y + h],
                [x + w, y + h],
            ], dtype=np.float32)

            success, rvec, tvec = cv2.solvePnP(
                self.obj_pts, img_pts, self.K, self.dist,
                flags=cv2.SOLVEPNP_IPPE
            )

            if success:
                tx = float(tvec[0])
                ty = float(tvec[1])
                tz = float(tvec[2])

                self.pose_history.append({'x': tx, 'y': ty, 'z': tz})
                if len(self.pose_history) == self.SMOOTH_N:
                    tx = float(np.mean([p['x'] for p in self.pose_history]))
                    ty = float(np.mean([p['y'] for p in self.pose_history]))
                    tz = float(np.mean([p['z'] for p in self.pose_history]))

                pose_msg = PoseStamped()
                pose_msg.header.stamp    = self.get_clock().now().to_msg()
                pose_msg.header.frame_id = "camera_left"
                pose_msg.pose.position.x = tx
                pose_msg.pose.position.y = ty
                pose_msg.pose.position.z = tz
                self.pose_pub.publish(pose_msg)

                detected = True

                self.get_logger().info(
                    f"FLARE | tx:{tx:.2f} ty:{ty:.2f} tz:{tz:.2f}",
                    throttle_duration_sec=0.5
                )

                cx_px    = x + w // 2
                cy_px    = y + h // 2
                frame_cx = img_w // 2
                frame_cy = img_h // 2

                cv2.rectangle(vis, (x, y), (x + w, y + h), (255, 0, 0), 2)
                cv2.circle(vis, (cx_px, cy_px), 8, (0, 255, 0), -1)
                cv2.drawMarker(vis, (frame_cx, frame_cy),
                               (255, 255, 255), cv2.MARKER_CROSS, 20, 2)
                cv2.line(vis, (frame_cx, frame_cy), (cx_px, cy_px), (0, 255, 255), 2)
                cv2.putText(vis, "DETECTED", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(vis, f"Pose norm: ({tx:+.2f},{ty:+.2f}) tz:{tz:.2f}m",
                            (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        if not detected:
            pose_msg = PoseStamped()
            pose_msg.header.stamp    = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = "none"
            pose_msg.pose.position.x = 0.0
            pose_msg.pose.position.y = 0.0
            pose_msg.pose.position.z = -1.0
            self.pose_pub.publish(pose_msg)

            frame_cx = img_w // 2
            frame_cy = img_h // 2
            cv2.drawMarker(vis, (frame_cx, frame_cy),
                           (255, 255, 255), cv2.MARKER_CROSS, 20, 2)
            cv2.putText(vis, "NO DETECTION", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)
            self.get_logger().warn("No flare detected", throttle_duration_sec=1.0)

        self.debug_pub.publish(String(data=f"[FLARE] {'DETECTED' if detected else 'LOST'}"))

        cv2.imshow("Flare Detection", vis)
        cv2.imshow("Orange Mask", mask)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = FlarePerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()