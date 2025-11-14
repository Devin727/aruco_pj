cmake_minimum_required(VERSION 3.5)
project(aruco_tracker)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Python package install
ament_python_install_package(${PROJECT_NAME})

# Install run scripts that actually exist
install(PROGRAMS
  ${PROJECT_NAME}/aruco_depth_pose.py
  ${PROJECT_NAME}/aligned_depth_pub.py
  ${PROJECT_NAME}/pickup_controller.py      
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Make package discoverable by ROS
install(FILES resource/${PROJECT_NAME}
  DESTINATION share/ament_index/resource_index/packages
)
install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()   
