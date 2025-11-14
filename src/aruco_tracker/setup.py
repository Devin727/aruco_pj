from setuptools import setup
from glob import glob

package_name = 'aruco_tracker'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],  # src/aruco_tracker/aruco_tracker 패키지
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # launch 파일은 패키지 루트의 launch/ 경로를 사용해야 함
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dabin',
    maintainer_email='dabin@example.com',
    description='Aruco pose publisher (ROS 2)',
    license='Apache-2.0',  # package.xml과 일치시킴
    entry_points={
        'console_scripts': [
            # 모듈 내부 main() 에 매핑
            'aruco_node = aruco_tracker.aruco_node:main',
            'aruco_depth_pose = aruco_tracker.aruco_depth_pose:main',
            'aligned_depth_pub = aruco_tracker.aligned_depth_pub:main',
        ],
    },
)
