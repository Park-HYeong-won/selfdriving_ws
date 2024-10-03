cmake_minimum_required(VERSION 3.0.2)
project(my_pid_controller)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
)


catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# 실행 파일 추가
add_executable(linear_controller src/linear_controller.cpp)
add_executable(speed_calculator src/speed_calculator.cpp)
add_executable(goal_velocity_publisher src/goal_velocity_publisher.cpp)
add_executable(angular_control src/angular_control.cpp)
add_executable(joy_push src/joy_push.cpp)
add_executable(cmd_vel_output src/cmd_vel_output.cpp)
add_executable(waypoint_follower src/waypoint_follower.cpp)

# 타겟 링크 설정
target_link_libraries(linear_controller
  ${catkin_LIBRARIES}
)

target_link_libraries(speed_calculator ${catkin_LIBRARIES})
target_link_libraries(goal_velocity_publisher ${catkin_LIBRARIES})
target_link_libraries(angular_control ${catkin_LIBRARIES})
target_link_libraries(joy_push ${catkin_LIBRARIES})
target_link_libraries(cmd_vel_output ${catkin_LIBRARIES})
target_link_libraries(waypoint_follower
  ${catkin_LIBRARIES} 
)
