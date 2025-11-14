# launch/aruco.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 공통 인자
    ns_arg     = DeclareLaunchArgument('ns', default_value='')
    use_rs_arg = DeclareLaunchArgument('use_rs_pipeline', default_value='true')  # true: aruco_depth_pose.py

    # 토픽 구독 방식(arcuo_node.py)용 인자들
    img_arg    = DeclareLaunchArgument('image_topic', default_value='/camera/camera/color/image_raw')
    info_arg   = DeclareLaunchArgument('camera_info_topic', default_value='/camera/camera/color/camera_info')
    dict_arg   = DeclareLaunchArgument('aruco_dict_name', default_value='DICT_4X4_50')
    size_arg   = DeclareLaunchArgument('marker_size', default_value='0.03')  # meters
    tid_arg    = DeclareLaunchArgument('target_id', default_value='-1')      # -1: any
    frame_arg  = DeclareLaunchArgument('camera_frame', default_value='camera_link')
    dbg_arg    = DeclareLaunchArgument('publish_debug_image', default_value='true')

    # pyrealsense2 직접 사용(aruco_depth_pose.py)용 인자들
    rs_tid_arg   = DeclareLaunchArgument('rs_target_id', default_value='-1')
    rs_size_arg  = DeclareLaunchArgument('marker_size_m', default_value='0.03')  # 현재 Z만 쓰니 영향 적음

    # 노드 정의
    node_subscribe = Node(
        package='aruco_tracker',
        executable='aruco_node.py',      # CMakeLists.txt에서 PROGRAMS로 설치한 파일명
        name='aruco_node',
        namespace=LaunchConfiguration('ns'),
        output='screen',
        emulate_tty=True,
        parameters=[{
            'image_topic':        LaunchConfiguration('image_topic'),
            'camera_info_topic':  LaunchConfiguration('camera_info_topic'),
            'aruco_dict_name':    LaunchConfiguration('aruco_dict_name'),
            'marker_size':        LaunchConfiguration('marker_size'),
            'target_id':          LaunchConfiguration('target_id'),
            'camera_frame':       LaunchConfiguration('camera_frame'),
            'publish_debug_image':LaunchConfiguration('publish_debug_image'),
        }],
        condition=UnlessCondition(LaunchConfiguration('use_rs_pipeline'))
    )

    node_rs_pipeline = Node(
        package='aruco_tracker',
        executable='aruco_depth_pose.py',  # CMakeLists.txt에서 PROGRAMS로 설치 필요
        name='aruco_depth_node',
        namespace=LaunchConfiguration('ns'),
        output='screen',
        emulate_tty=True,
        parameters=[{
            'target_id':      LaunchConfiguration('rs_target_id'),
            'marker_size_m':  LaunchConfiguration('marker_size_m'),
        }],
        condition=IfCondition(LaunchConfiguration('use_rs_pipeline'))
    )

    return LaunchDescription([
        ns_arg, use_rs_arg,
        img_arg, info_arg, dict_arg, size_arg, tid_arg, frame_arg, dbg_arg,
        rs_tid_arg, rs_size_arg,
        node_subscribe,
        node_rs_pipeline,
    ])
