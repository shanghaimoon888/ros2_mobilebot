#ifndef R1_HARDWARE_INTERFACE__R1_SYSTEM_HARDWARE_HPP_
#define R1_HARDWARE_INTERFACE__R1_SYSTEM_HARDWARE_HPP_

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

// pigpio 头文件
#include <pigpiod_if2.h>

namespace r1_hardware_interface
{

class R1SystemHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(R1SystemHardware)

  // ======================== 生命周期函数 ========================
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  // ======================== 接口导出 ========================
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  // ======================== 实时循环 ========================
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // ======================== 编码器回调（静态，供 pigpio C 接口调用） ========================
  static void left_encoder_callback(int pi, unsigned gpio, unsigned level, uint32_t tick);
  static void right_encoder_callback(int pi, unsigned gpio, unsigned level, uint32_t tick);

  // 静态实例指针
  static R1SystemHardware * instance_;

  // ======================== 硬件参数 ========================
  // TB6612 控制引脚
  int left_pwm_pin_ = -1;
  int left_ain1_pin_ = -1;
  int left_ain2_pin_ = -1;
  int right_pwm_pin_ = -1;
  int right_bin1_pin_ = -1;
  int right_bin2_pin_ = -1;

  // 编码器引脚
  int left_encoder_a_pin_ = -1;
  int left_encoder_b_pin_ = -1;
  int right_encoder_a_pin_ = -1;
  int right_encoder_b_pin_ = -1;

  // 编码器/电机参数
  double encoder_resolution_ = 11.0;
  double gear_ratio_ = 30.0;
  double wheel_radius_ = 0.033;

  // ======================== PID 参数 ========================
  double kp_ = 1.0;
  double ki_ = 0.0;
  double kd_ = 0.0;
  double integral_clamp_ = 10.0;
  double max_velocity_ = 1.0;
  double velocity_filter_alpha_ = 0.05;

  // ======================== 加速度限制 ========================
  double max_accel_ = 3.0;
  double left_prev_output_ = 0.0;
  double right_prev_output_ = 0.0;

  // ======================== 目标方向变化检测 ========================
  double left_prev_target_ = 0.0;
  double right_prev_target_ = 0.0;

  // ======================== PID 内部状态 ========================
  double left_integral_ = 0.0;
  double right_integral_ = 0.0;
  double left_prev_error_ = 0.0;
  double right_prev_error_ = 0.0;

  // 滤波后的速度
  double left_filtered_velocity_ = 0.0;
  double right_filtered_velocity_ = 0.0;

  // ======================== pigpio 句柄与回调 ID ========================
  int pi_handle_ = -1;
  int left_encoder_cb_a_ = -1;
  int left_encoder_cb_b_ = -1;
  int right_encoder_cb_a_ = -1;
  int right_encoder_cb_b_ = -1;

  // ======================== 关节状态与命令缓存 ========================
  std::vector<double> hw_commands_velocity_;
  std::vector<double> hw_states_velocity_;
  std::vector<double> hw_states_position_;

  // ======================== 编码器计数 ========================
  std::atomic<int32_t> left_encoder_ticks_{0};
  std::atomic<int32_t> right_encoder_ticks_{0};
  int32_t prev_left_encoder_ticks_ = 0;
  int32_t prev_right_encoder_ticks_ = 0;

  // 上一次 A/B 相状态
  volatile int left_last_state_ = 0;
  volatile int right_last_state_ = 0;

  // ======================== PID 辅助函数 ========================
  double compute_pid(
    double target, double actual, double dt,
    double & integral, double & prev_error);
};

}  // namespace r1_hardware_interface

#endif  // R1_HARDWARE_INTERFACE__R1_SYSTEM_HARDWARE_HPP_