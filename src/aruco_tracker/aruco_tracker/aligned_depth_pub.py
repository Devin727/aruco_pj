#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import pyrealsense2 as rs
import numpy as np

class AlignedDepthPublisher(Node):
    def __init__(self):
        super().__init__('aligned_depth_publisher')

        # Publishers
        self.pub_color = self.create_publisher(Image, '/camera/color/image_raw', 10)
        self.pub_depth = self.create_publisher(Image, '/camera/aligned_depth_to_color/image_raw', 10)

        self.br = CvBridge()

        # RealSense pipeline
        self.pipeline = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.profile = self.pipeline.start(cfg)

        # Align depth to color
        self.align = rs.align(rs.stream.color)

        # 30 Hz timer
        self.timer = self.create_timer(1.0/30.0, self.loop)

        self.get_logger().info('aligned_depth_publisher up (publishing color & aligned depth)')

    def loop(self):
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

        # To ROS
        depth_msg = self.br.cv2_to_imgmsg(depth_img, encoding='16UC1')
        color_msg = self.br.cv2_to_imgmsg(color_img, encoding='bgr8')
        depth_msg.header.stamp = color_msg.header.stamp = now
        depth_msg.header.frame_id = color_msg.header.frame_id = 'camera_color_optical_frame'

        self.pub_depth.publish(depth_msg)
        self.pub_color.publish(color_msg)

    def destroy_node(self):
        try:
            self.pipeline.stop()
        except Exception:
            pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = AlignedDepthPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
