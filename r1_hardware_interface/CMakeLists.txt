cmake_minimum_required(VERSION 3.8)
project(r1_hardware_interface)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# 链接 pigpio 库（需要先安装 pigpio 开发包：sudo apt install libpigpio-dev）
find_library(PIGPIO_LIB pigpio)
find_library(PIGPIOD_IF2_LIB pigpiod_if2)

add_library(r1_hardware_interface SHARED
  src/r1_system_hardware.cpp
)

target_include_directories(r1_hardware_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(r1_hardware_interface
  ${PIGPIO_LIB}
  ${PIGPIOD_IF2_LIB}
)

ament_target_dependencies(r1_hardware_interface
  rclcpp
  hardware_interface
  pluginlib
  rclcpp_lifecycle
)

# 关键：导出插件描述文件
pluginlib_export_plugin_description_file(
  hardware_interface
  r1_hardware_interface.xml
)

install(TARGETS r1_hardware_interface
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(r1_hardware_interface)
ament_export_dependencies(ament_cmake rclcpp hardware_interface pluginlib rclcpp_lifecycle)

ament_package()