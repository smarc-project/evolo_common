cmake_minimum_required(VERSION 3.8)
project(evolo_gimbal_remote_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(evolo_msgs REQUIRED)



# To install python nodes.
install(PROGRAMS 
  scripts/gimbal_action_client.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
