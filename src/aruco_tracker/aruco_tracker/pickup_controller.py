#!/usr/bin/env python3
import math
from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float64  # TODO: 실제 gripper 메시지 타입에 맞게 바꿔야 할 수도 있음


class PickupState(Enum):
    IDLE = 0
    APPROACH = 1
    PICK = 2
    RETREAT = 3
    DROP = 4


class PickupController(Node):
    def __init__(self):
        super().__init__('pickup_controller')

        # ========= Parameters =========
        # 토픽 이름: 너가 올려준 리스트에 맞게 기본값 설정함 (*수정됨*)
        self.declare_parameter('marker_topic', '/aruco/pose')                  # ★ 수정됨
        self.declare_parameter('odom_topic', '/odom')                          # ★ 수정됨
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')                    # ★ 유지
        self.declare_parameter('joint_traj_topic', '/arm_controller/joint_trajectory')  # ★ 수정됨

        # gripper는 토픽 리스트에 command 토픽이 안 보이는데,
        # 일단 placeholder로 parameter만 만들어 둠. 나중에 실제 토픽 이름/타입에 맞게 수정 필요.
        self.declare_parameter('gripper_topic', '/gripper_controller/command')

        # 속도 & 오프셋 파라미터
        self.declare_parameter('forward_speed', 0.05)   # m/s
        self.declare_parameter('backward_speed', 0.05)  # m/s
        self.declare_parameter('distance_offset', 0.15) # m (카메라-그리퍼 보정)

        # 매니퓰레이터 조인트 이름 / 포즈 (예시 값)
        # ★ 실제 joint_names / 각도는 꼭 너 로봇에 맞게 바꿔야 함
        self.arm_joint_names = ['joint1', 'joint2', 'joint3', 'joint4']
        self.arm_pick_positions = [0.0, -0.8, 0.8, 0.0]  # 앞으로 쭉 뻗는 자세 예시
        self.arm_home_positions = [0.0, 0.0, 0.0, 0.0]   # 초기자세 예시

        # ========= Parameter 값 읽기 =========
        marker_topic = self.get_parameter('marker_topic').get_parameter_value().string_value
        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        joint_traj_topic = self.get_parameter('joint_traj_topic').get_parameter_value().string_value
        gripper_topic = self.get_parameter('gripper_topic').get_parameter_value().string_value

        self.forward_speed = self.get_parameter('forward_speed').get_parameter_value().double_value
        self.backward_speed = self.get_parameter('backward_speed').get_parameter_value().double_value
        self.distance_offset = self.get_parameter('distance_offset').get_parameter_value().double_value

        # ========= Publishers =========
        self.cmd_vel_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.joint_traj_pub = self.create_publisher(JointTrajectory, joint_traj_topic, 10)
        self.gripper_pub = self.create_publisher(Float64, gripper_topic, 10)  # ★ gripper 토픽/타입은 나중에 확인 필요

        # ========= Subscribers =========
        self.marker_sub = self.create_subscription(
            PoseStamped,          # ★ /aruco/pose 타입이 PoseStamped라고 가정
            marker_topic,
            self.marker_callback,
            10
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            10
        )

        # ========= 상태 변수 =========
        self.state = PickupState.IDLE
        self.flag_active = False           # 한 사이클 동작 중인지 여부
        self.target_distance = 0.0         # offset 적용 후 이동할 거리
        self.raw_distance = 0.0            # 카메라에서 읽은 z 거리

        # 오돔 / 이동거리 계산용
        self.odom_available = False
        self.current_x = 0.0
        self.current_y = 0.0
        self.move_start_x = None
        self.move_start_y = None

        # 마커 인식 시간
        self.last_marker_time = None
        self.marker_timeout_sec = 1.0

        # PICK / DROP 단계 안에서의 서브 스텝
        self.sub_state_step = 0
        self.state_enter_time = self.get_clock().now()

        # 메인 루프 타이머 (20Hz)
        self.timer = self.create_timer(0.05, self.main_loop)

        self.get_logger().info('PickupController node started.')

    # ========= 콜백들 =========
    def marker_callback(self, msg: PoseStamped):
        """아루코 마커 포즈 콜백"""
        self.last_marker_time = self.get_clock().now()
        z = msg.pose.position.z
        self.raw_distance = z

        # IDLE + 아직 작업 중이 아닐 때만 타겟 설정
        if self.state == PickupState.IDLE and not self.flag_active:
            # 너무 가까운/먼 값 필터링 (필요에 따라 조정)
            if 0.1 < z < 2.0:
                self.target_distance = max(0.0, z - self.distance_offset)
                self.flag_active = True
                self.state = PickupState.APPROACH
                self.sub_state_step = 0
                self.state_enter_time = self.get_clock().now()
                self.move_start_x = None
                self.move_start_y = None
                self.get_logger().info(
                    f'Marker detected. raw_distance={z:.3f}, target_distance={self.target_distance:.3f}'
                )

    def odom_callback(self, msg: Odometry):
        """오돔 콜백 - 현재 위치 저장"""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.odom_available = True

    # ========= 유틸 함수들 =========
    def reset_motion_start(self):
        """이동 시작 위치 초기화"""
        if self.odom_available:
            self.move_start_x = self.current_x
            self.move_start_y = self.current_y
        else:
            self.move_start_x = None
            self.move_start_y = None

    def moved_distance(self) -> float:
        """오돔 기준 현재까지 이동한 거리 계산"""
        if self.move_start_x is None or self.move_start_y is None:
            return 0.0
        dx = self.current_x - self.move_start_x
        dy = self.current_y - self.move_start_y
        return math.hypot(dx, dy)

    def stop_robot(self):
        """로봇 정지 (cmd_vel = 0)"""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)

    def send_forward_cmd(self):
        twist = Twist()
        twist.linear.x = self.forward_speed
        self.cmd_vel_pub.publish(twist)

    def send_backward_cmd(self):
        twist = Twist()
        twist.linear.x = -self.backward_speed
        self.cmd_vel_pub.publish(twist)

    def send_arm_trajectory(self, positions):
        """JointTrajectory 한 번 publish"""
        traj = JointTrajectory()
        traj.joint_names = self.arm_joint_names

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(seconds=3.0).to_msg()

        traj.points.append(point)
        self.joint_traj_pub.publish(traj)

    def send_gripper(self, value: float):
        """그리퍼 제어 (Float64 예시)"""
        msg = Float64()
        msg.data = value
        self.gripper_pub.publish(msg)

    # ========= 메인 루프 =========
    def main_loop(self):
        now = self.get_clock().now()

        # (원하면) 마커 타임아웃 체크 가능
        if self.last_marker_time is not None:
            if (now - self.last_marker_time).nanoseconds * 1e-9 > self.marker_timeout_sec:
                # 여기서 마커 안 보일 때 별도 처리 가능 (지금은 패스)
                pass

        if self.state == PickupState.IDLE:
            self.handle_idle()

        elif self.state == PickupState.APPROACH:
            self.handle_approach()

        elif self.state == PickupState.PICK:
            self.handle_pick(now)

        elif self.state == PickupState.RETREAT:
            self.handle_retreat()

        elif self.state == PickupState.DROP:
            self.handle_drop(now)

    # ========= 상태 핸들러 =========
    def handle_idle(self):
        """IDLE: 마커 기다리는 상태"""
        # marker_callback에서 상태 전환함
        pass

    def handle_approach(self):
        """APPROACH: target_distance만큼 전진"""
        if not self.odom_available:
            self.get_logger().warn('No odom available. Cannot move forward.')
            return

        if self.move_start_x is None or self.move_start_y is None:
            self.reset_motion_start()
            self.get_logger().info('Start forward motion.')
            self.send_forward_cmd()
            return

        dist = self.moved_distance()
        if dist < self.target_distance:
            self.send_forward_cmd()
        else:
            self.stop_robot()
            self.get_logger().info(
                f'Approach done. moved_distance={dist:.3f}, target={self.target_distance:.3f}'
            )
            self.move_start_x = None
            self.move_start_y = None

            self.state = PickupState.PICK
            self.sub_state_step = 0
            self.state_enter_time = self.get_clock().now()

    def handle_pick(self, now):
        """PICK: 팔 뻗고 그리퍼로 집는 단계"""
        # sub_state_step:
        # 0: 팔을 pick pose로
        # 1: 대기
        # 2: 그리퍼 닫기
        # 3: 대기 후 RETREAT로

        if self.sub_state_step == 0:
            self.get_logger().info('Moving arm to pick pose.')
            self.send_arm_trajectory(self.arm_pick_positions)
            self.sub_state_step = 1
            self.state_enter_time = now

        elif self.sub_state_step == 1:
            if (now - self.state_enter_time).nanoseconds * 1e-9 > 3.0:
                self.get_logger().info('Closing gripper.')
                # 예: 1.0 = close, 0.0 = open (환경에 맞게 수정)
                self.send_gripper(1.0)
                self.sub_state_step = 2
                self.state_enter_time = now

        elif self.sub_state_step == 2:
            if (now - self.state_enter_time).nanoseconds * 1e-9 > 1.5:
                self.get_logger().info('Pick done. Go to RETREAT.')
                self.state = PickupState.RETREAT
                self.sub_state_step = 0
                self.state_enter_time = now
                self.move_start_x = None
                self.move_start_y = None

    def handle_retreat(self):
        """RETREAT: 전진했던 거리만큼 후진"""
        if not self.odom_available:
            self.get_logger().warn('No odom available. Cannot move backward.')
            return

        if self.move_start_x is None or self.move_start_y is None:
            self.reset_motion_start()
            self.get_logger().info('Start backward motion.')
            self.send_backward_cmd()
            return

        dist = self.moved_distance()
        if dist < self.target_distance:
            self.send_backward_cmd()
        else:
            self.stop_robot()
            self.get_logger().info(
                f'Retreat done. moved_distance={dist:.3f}, target={self.target_distance:.3f}'
            )
            self.move_start_x = None
            self.move_start_y = None

            self.state = PickupState.DROP
            self.sub_state_step = 0
            self.state_enter_time = self.get_clock().now()

    def handle_drop(self, now):
        """DROP: 물건 내려놓고 팔을 홈으로, flag off"""
        # sub_state_step:
        # 0: 그리퍼 open
        # 1: 대기
        # 2: 팔 홈 포즈로
        # 3: 대기 후 IDLE로

        if self.sub_state_step == 0:
            self.get_logger().info('Opening gripper to drop object.')
            self.send_gripper(0.0)  # 예: 0.0 = open
            self.sub_state_step = 1
            self.state_enter_time = now

        elif self.sub_state_step == 1:
            if (now - self.state_enter_time).nanoseconds * 1e-9 > 1.0:
                self.get_logger().info('Moving arm to home pose.')
                self.send_arm_trajectory(self.arm_home_positions)
                self.sub_state_step = 2
                self.state_enter_time = now

        elif self.sub_state_step == 2:
            if (now - self.state_enter_time).nanoseconds * 1e-9 > 3.0:
                self.get_logger().info('Drop done. Back to IDLE.')
                self.flag_active = False
                self.state = PickupState.IDLE
                self.sub_state_step = 0
                self.state_enter_time = now
                self.target_distance = 0.0
                self.raw_distance = 0.0


def main(args=None):
    rclpy.init(args=args)
    node = PickupController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
