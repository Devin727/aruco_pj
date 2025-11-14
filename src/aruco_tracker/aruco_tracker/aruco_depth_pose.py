#!/usr/bin/env python3
# aruco_depth_pose.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import numpy as np
import cv2
from cv2 import aruco
import pyrealsense2 as rs

def make_aruco_params():
    # OpenCV 4.6 이하: DetectorParameters_create(), 4.7 이상: DetectorParameters()
    if hasattr(aruco, "DetectorParameters_create"):
        p = aruco.DetectorParameters_create()
    else:
        p = aruco.DetectorParameters()
    # 탐지 관대하게 (인쇄/조명 편차 대응)
    if hasattr(p, "detectInvertedMarker"):
        p.detectInvertedMarker = True      # 흑/백 반전 허용
    p.adaptiveThreshWinSizeMin = 3
    p.adaptiveThreshWinSizeMax = 35
    p.adaptiveThreshWinSizeStep = 5
    p.minMarkerPerimeterRate   = 0.02
    p.maxMarkerPerimeterRate   = 4.0
    if hasattr(aruco, "CORNER_REFINE_SUBPIX"):
        p.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
    return p

class ArucoDepthNode(Node):
    def __init__(self):
        super().__init__('aruco_depth_node')

        # ---- Parameters ----
        self.declare_parameter('target_id', -1)         # -1: 아무 마커든 우선
        self.declare_parameter('marker_size_m', 0.05)   # (현재 Z만 쓰므로 영향 적음)
        self.target_id   = int(self.get_parameter('target_id').value)
        self.marker_size = float(self.get_parameter('marker_size_m').value)

        # ---- Publishers ----
        self.pub_pose  = self.create_publisher(PoseStamped, '/aruco/pose', 10)
        self.pub_color = self.create_publisher(Image, '/camera/color/image_raw', 10)  # 디버그/뷰어용
        self.pub_depth = self.create_publisher(Image, '/camera/aligned_depth_to_color/image_raw', 10)
        self.pub_dbg   = self.create_publisher(Image, '/camera/color/debug', 10)

        self.br = CvBridge()

        # ---- RealSense pipeline ----
        self.pipeline = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.profile = self.pipeline.start(cfg)
        self.align = rs.align(rs.stream.color)  # depth -> color 정렬

        # ---- ArUco 설정 ----
        self.dict_candidates = [
            aruco.DICT_4X4_50,
            aruco.DICT_5X5_50,
            aruco.DICT_6X6_50,
        ]
        self.aruco_dict   = aruco.getPredefinedDictionary(self.dict_candidates[0])
        self.aruco_params = make_aruco_params()

        # ---- Loop ----
        self.timer = self.create_timer(1.0/30.0, self.loop)

        self.get_logger().info('aruco_depth_node up (any-marker mode, nearest-by-depth)')

    def loop(self):
        # 프레임 획득
        try:
            frames = self.pipeline.wait_for_frames(2000)
        except Exception as e:
            self.get_logger().warn(f'wait_for_frames timeout/err: {e}')
            return

        aligned = self.align.process(frames)
        depth_f = aligned.get_depth_frame()
        color_f = aligned.get_color_frame()
        if not depth_f or not color_f:
            return

        depth_img = np.asanyarray(depth_f.get_data())
        color_img = np.asanyarray(color_f.get_data())
        now = self.get_clock().now().to_msg()

        # 퍼블리시(뷰어용)
        depth_msg = self.br.cv2_to_imgmsg(depth_img, encoding='16UC1'); depth_msg.header.stamp = now
        color_msg = self.br.cv2_to_imgmsg(color_img, encoding='bgr8');  color_msg.header.stamp = now
        depth_msg.header.frame_id = color_msg.header.frame_id = 'camera_color_optical_frame'
        self.pub_depth.publish(depth_msg)
        self.pub_color.publish(color_msg)

        debug = color_img.copy()

        # ---- ArUco 탐지 (우선 4x4_50, 실패 시 후보 사전 순회) ----
        gray = cv2.cvtColor(color_img, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)
        if ids is None or len(ids) == 0:
            for d in self.dict_candidates[1:]:
                dct = aruco.getPredefinedDictionary(d)
                c, i, _ = aruco.detectMarkers(gray, dct, parameters=self.aruco_params)
                if i is not None and len(i) > 0:
                    self.aruco_dict = dct
                    corners, ids = c, i
                    break

        if ids is None or len(ids) == 0:
            # 감지 실패
            self.pub_dbg.publish(self.br.cv2_to_imgmsg(debug, encoding='bgr8'))
            return

        aruco.drawDetectedMarkers(debug, corners, ids)
        ids = ids.flatten()

        # 후보 인덱스
        indices = list(range(len(ids))) if self.target_id < 0 else [i for i, cid in enumerate(ids) if cid == self.target_id]
        if not indices:
            self.pub_dbg.publish(self.br.cv2_to_imgmsg(debug, encoding='bgr8'))
            return

        # 가장 가까운(z) 마커 선택 (중심 픽셀로 깊이 샘플)
        best = None
        for i in indices:
            pts = corners[i].reshape(-1, 2).astype(int)
            cx, cy = int(np.mean(pts[:, 0])), int(np.mean(pts[:, 1]))

            Z = depth_f.get_distance(cx, cy)  # meters
            if Z == 0.0:
                patch = depth_img[max(cy-2, 0):cy+3, max(cx-2, 0):cx+3]
                vals = patch[(patch > 0) & (patch < 10000)]
                if vals.size:
                    Z = float(np.median(vals)) / 1000.0

            if Z == 0.0:
                continue

            if (best is None) or (Z < best[0]):
                best = (Z, i, cx, cy)

        if best is not None:
            Z, i, cx, cy = best
            cid = int(ids[i])

            # Z만 퍼블리시 (optical frame: z forward)
            ps = PoseStamped()
            ps.header.stamp = now
            ps.header.frame_id = 'camera_color_optical_frame'
            ps.pose.position.z = Z
            ps.pose.orientation.w = 1.0
            self.pub_pose.publish(ps)

            cv2.putText(debug, f'id {cid}: {Z:.3f} m',
                        (cx+6, cy-6), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2, cv2.LINE_AA)

        # 디버그 오버레이 퍼블리시
        dbg_msg = self.br.cv2_to_imgmsg(debug, encoding='bgr8')
        dbg_msg.header.stamp = now
        dbg_msg.header.frame_id = 'camera_color_optical_frame'
        self.pub_dbg.publish(dbg_msg)

    def destroy_node(self):
        try:
            self.pipeline.stop()
        except Exception:
            pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDepthNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
