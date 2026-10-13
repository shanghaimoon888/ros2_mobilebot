cmake_minimum_required(VERSION 3.8)
project(r1_description)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)

# 安装 urdf 目录（包含所有 .xacro 和 .urdf 文件）
install(
  DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)



ament_package()