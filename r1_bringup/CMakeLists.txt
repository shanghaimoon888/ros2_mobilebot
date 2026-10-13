cmake_minimum_required(VERSION 3.8)
project(r1_bringup)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)

# 安装 config 目录
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# 安装 launch 目录
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()