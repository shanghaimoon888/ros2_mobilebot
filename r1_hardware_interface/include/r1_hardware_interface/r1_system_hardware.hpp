#ifndef R1_HARDWARE_INTERFACE__R1_SYSTEM_HARDWARE_HPP_
#define R1_HARDWARE_INTERFACE__R1_SYSTEM_HARDWARE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <atomic>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace r1_hardware_interface
{

class R1SystemHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(R1SystemHardware)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // --- 串口相关 ---
  std::string serial_port_;
  int baud_rate_;
  int serial_fd_ = -1;
  std::thread serial_thread_;
  std::atomic<bool> running_{false};
  std::mutex data_mutex_;

  // --- 参数 ---
  double wheel_radius_ = 0.024;
  double wheel_separation_ = 0.16;
  
  // ======================== IMU 数据缓存 ========================
  // 欧拉角 (从串口解析)
  double hw_states_imu_yaw_ = 0.0;
  double hw_states_imu_pitch_ = 0.0;
  double hw_states_imu_roll_ = 0.0;

  // 姿态 (四元数)
  double hw_states_imu_q_x_ = 0.0;
  double hw_states_imu_q_y_ = 0.0;
  double hw_states_imu_q_z_ = 0.0;
  double hw_states_imu_q_w_ = 1.0;

  // 角速度 (rad/s) - 占位
  double hw_states_imu_angular_x_ = 0.0;
  double hw_states_imu_angular_y_ = 0.0;
  double hw_states_imu_angular_z_ = 0.0;

  // 线加速度 (m/s^2) - 占位
  double hw_states_imu_linear_x_ = 0.0;
  double hw_states_imu_linear_y_ = 0.0;
  double hw_states_imu_linear_z_ = 9.81; // 默认受重力
  
  // --- 关节状态与命令缓存 ---
  std::vector<double> hw_commands_velocity_; // 树莓派发给STM32的指令 (rad/s)
  std::vector<double> hw_states_velocity_;   // STM32反馈给树莓派的实际速度 (rad/s)
  std::vector<double> hw_states_position_;
  
  // ========== 测距传感器 ==========
  double hw_states_range_{0.0}; // 单位 m，ros2 Range消息标准单位
  

  // --- 串口通信线程函数 ---
  void serial_receive_loop();

  // --- 辅助函数 ---
  void send_speed_command(int16_t left_mm_s, int16_t right_mm_s);
};

}  // namespace r1_hardware_interface

#endif  // R1_HARDWARE_INTERFACE__R1_SYSTEM_HARDWARE_HPP_