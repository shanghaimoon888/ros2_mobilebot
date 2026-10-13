#include "r1_hardware_interface/r1_system_hardware.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace r1_hardware_interface
{

// 静态实例指针
R1SystemHardware * R1SystemHardware::instance_ = nullptr;

// ==================== 方向查表（x4 正交解码） ====================
static const int8_t QUAD_TABLE[16] = {
     0, +1, -1,  0,
    -1,  0,  0, +1,
    +1,  0,  0, -1,
     0, -1, +1,  0
};

// ==================== 编码器回调（查表法） ====================
void R1SystemHardware::left_encoder_callback(
  int pi, unsigned /*gpio*/, unsigned /*level*/, uint32_t /*tick*/)
{
  if (instance_ == nullptr) return;

  int a = gpio_read(pi, instance_->left_encoder_a_pin_);
  int b = gpio_read(pi, instance_->left_encoder_b_pin_);
  int current = (a << 1) | b;
  int index = (instance_->left_last_state_ << 2) | current;
  instance_->left_last_state_ = current;

  int8_t delta = QUAD_TABLE[index & 0x0F];
  if (delta != 0) {
    instance_->left_encoder_ticks_ += delta;
  }
}

void R1SystemHardware::right_encoder_callback(
  int pi, unsigned /*gpio*/, unsigned /*level*/, uint32_t /*tick*/)
{
  if (instance_ == nullptr) return;

  int a = gpio_read(pi, instance_->right_encoder_a_pin_);
  int b = gpio_read(pi, instance_->right_encoder_b_pin_);
  int current = (a << 1) | b;
  int index = (instance_->right_last_state_ << 2) | current;
  instance_->right_last_state_ = current;

  int8_t delta = QUAD_TABLE[index & 0x0F];
  if (delta != 0) {
    // 右轮镜像安装，方向已通过测试确认为 +=
    instance_->right_encoder_ticks_ += delta;
  }
}

// ==================== 参数读取辅助 ====================
static std::string get_required_param(
  const hardware_interface::HardwareInfo & info, const std::string & key)
{
  auto it = info.hardware_parameters.find(key);
  if (it == info.hardware_parameters.end()) {
    throw std::runtime_error("Missing required hardware parameter: '" + key + "'");
  }
  return it->second;
}

static double get_optional_param(
  const hardware_interface::HardwareInfo & info,
  const std::string & key, double default_val)
{
  auto it = info.hardware_parameters.find(key);
  return (it != info.hardware_parameters.end()) ? std::stod(it->second) : default_val;
}

// ==================== on_init ====================
hardware_interface::CallbackReturn R1SystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  try {
    left_pwm_pin_ = std::stoi(get_required_param(info, "left_wheel_pwm_pin"));
    left_ain1_pin_ = std::stoi(get_required_param(info, "left_wheel_ain1_pin"));
    left_ain2_pin_ = std::stoi(get_required_param(info, "left_wheel_ain2_pin"));
    right_pwm_pin_ = std::stoi(get_required_param(info, "right_wheel_pwm_pin"));
    right_bin1_pin_ = std::stoi(get_required_param(info, "right_wheel_bin1_pin"));
    right_bin2_pin_ = std::stoi(get_required_param(info, "right_wheel_bin2_pin"));

    left_encoder_a_pin_ = std::stoi(get_required_param(info, "left_encoder_a_pin"));
    left_encoder_b_pin_ = std::stoi(get_required_param(info, "left_encoder_b_pin"));
    right_encoder_a_pin_ = std::stoi(get_required_param(info, "right_encoder_a_pin"));
    right_encoder_b_pin_ = std::stoi(get_required_param(info, "right_encoder_b_pin"));

    encoder_resolution_ = std::stod(get_required_param(info, "encoder_resolution"));
    gear_ratio_ = std::stod(get_required_param(info, "gear_ratio"));
    wheel_radius_ = std::stod(get_required_param(info, "wheel_radius"));

    kp_ = get_optional_param(info, "pid_kp", 1.0);
    ki_ = get_optional_param(info, "pid_ki", 0.0);
    kd_ = get_optional_param(info, "pid_kd", 0.0);
    integral_clamp_ = get_optional_param(info, "pid_integral_clamp", 10.0);
    max_velocity_ = get_optional_param(info, "max_velocity", 1.0);
    velocity_filter_alpha_ = get_optional_param(info, "velocity_filter_alpha", 0.05);
    max_accel_ = get_optional_param(info, "max_accel", 3.0);

  } catch (const std::exception & e) {
    RCLCPP_FATAL(
      rclcpp::get_logger("R1SystemHardware"),
      "Failed to read hardware parameters: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  const size_t n = info_.joints.size();
  hw_commands_velocity_.assign(n, 0.0);
  hw_states_velocity_.assign(n, 0.0);
  hw_states_position_.assign(n, 0.0);

  RCLCPP_INFO(
    rclcpp::get_logger("R1SystemHardware"),
    "Hardware initialized. PID: kp=%.3f ki=%.3f kd=%.3f max_vel=%.3f max_accel=%.3f filter=%.3f",
    kp_, ki_, kd_, max_velocity_, max_accel_, velocity_filter_alpha_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ==================== on_configure ====================
hardware_interface::CallbackReturn R1SystemHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  pi_handle_ = pigpio_start(nullptr, nullptr);
  if (pi_handle_ < 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger("R1SystemHardware"),
      "Failed to connect to pigpio daemon! Make sure 'sudo pigpiod' is running.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // TB6612 控制引脚设为输出
  set_mode(pi_handle_, left_pwm_pin_, PI_OUTPUT);
  set_mode(pi_handle_, left_ain1_pin_, PI_OUTPUT);
  set_mode(pi_handle_, left_ain2_pin_, PI_OUTPUT);
  set_mode(pi_handle_, right_pwm_pin_, PI_OUTPUT);
  set_mode(pi_handle_, right_bin1_pin_, PI_OUTPUT);
  set_mode(pi_handle_, right_bin2_pin_, PI_OUTPUT);

  // 初始停止
  set_PWM_dutycycle(pi_handle_, left_pwm_pin_, 0);
  set_PWM_dutycycle(pi_handle_, right_pwm_pin_, 0);
  gpio_write(pi_handle_, left_ain1_pin_, PI_LOW);
  gpio_write(pi_handle_, left_ain2_pin_, PI_LOW);
  gpio_write(pi_handle_, right_bin1_pin_, PI_LOW);
  gpio_write(pi_handle_, right_bin2_pin_, PI_LOW);

  // 编码器引脚设为输入，启用上拉
  set_mode(pi_handle_, left_encoder_a_pin_, PI_INPUT);
  set_mode(pi_handle_, left_encoder_b_pin_, PI_INPUT);
  set_mode(pi_handle_, right_encoder_a_pin_, PI_INPUT);
  set_mode(pi_handle_, right_encoder_b_pin_, PI_INPUT);

  set_pull_up_down(pi_handle_, left_encoder_a_pin_, PI_PUD_UP);
  set_pull_up_down(pi_handle_, left_encoder_b_pin_, PI_PUD_UP);
  set_pull_up_down(pi_handle_, right_encoder_a_pin_, PI_PUD_UP);
  set_pull_up_down(pi_handle_, right_encoder_b_pin_, PI_PUD_UP);

  // 设置 PWM 频率
  set_PWM_frequency(pi_handle_, left_pwm_pin_, 1000);
  set_PWM_frequency(pi_handle_, right_pwm_pin_, 1000);

  // 初始化 last_state
  left_last_state_  = (gpio_read(pi_handle_, left_encoder_a_pin_) << 1)
                    | gpio_read(pi_handle_, left_encoder_b_pin_);
  right_last_state_ = (gpio_read(pi_handle_, right_encoder_a_pin_) << 1)
                    | gpio_read(pi_handle_, right_encoder_b_pin_);

  // 注册 A/B 双回调，双边沿触发（x4 模式）
  instance_ = this;
  left_encoder_cb_a_ = callback(pi_handle_, left_encoder_a_pin_,  EITHER_EDGE, left_encoder_callback);
  left_encoder_cb_b_ = callback(pi_handle_, left_encoder_b_pin_,  EITHER_EDGE, left_encoder_callback);
  right_encoder_cb_a_ = callback(pi_handle_, right_encoder_a_pin_, EITHER_EDGE, right_encoder_callback);
  right_encoder_cb_b_ = callback(pi_handle_, right_encoder_b_pin_, EITHER_EDGE, right_encoder_callback);

  if (left_encoder_cb_a_ < 0 || left_encoder_cb_b_ < 0 ||
      right_encoder_cb_a_ < 0 || right_encoder_cb_b_ < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("R1SystemHardware"), "Failed to register encoder callbacks!");
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("R1SystemHardware"), "Hardware configured (x4 encoder mode).");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ==================== on_activate ====================
hardware_interface::CallbackReturn R1SystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  left_encoder_ticks_ = 0;
  right_encoder_ticks_ = 0;
  prev_left_encoder_ticks_ = 0;
  prev_right_encoder_ticks_ = 0;

  left_integral_ = 0.0;
  right_integral_ = 0.0;
  left_prev_error_ = 0.0;
  right_prev_error_ = 0.0;

  left_filtered_velocity_ = 0.0;
  right_filtered_velocity_ = 0.0;

  left_prev_output_ = 0.0;
  right_prev_output_ = 0.0;
  left_prev_target_ = 0.0;
  right_prev_target_ = 0.0;

  set_PWM_dutycycle(pi_handle_, left_pwm_pin_, 0);
  set_PWM_dutycycle(pi_handle_, right_pwm_pin_, 0);

  RCLCPP_INFO(rclcpp::get_logger("R1SystemHardware"), "Hardware activated.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ==================== on_deactivate ====================
hardware_interface::CallbackReturn R1SystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (pi_handle_ >= 0) {
    set_PWM_dutycycle(pi_handle_, left_pwm_pin_, 0);
    set_PWM_dutycycle(pi_handle_, right_pwm_pin_, 0);
    gpio_write(pi_handle_, left_ain1_pin_, PI_LOW);
    gpio_write(pi_handle_, left_ain2_pin_, PI_LOW);
    gpio_write(pi_handle_, right_bin1_pin_, PI_LOW);
    gpio_write(pi_handle_, right_bin2_pin_, PI_LOW);
  }

  if (left_encoder_cb_a_ >= 0) callback_cancel(left_encoder_cb_a_);
  if (left_encoder_cb_b_ >= 0) callback_cancel(left_encoder_cb_b_);
  if (right_encoder_cb_a_ >= 0) callback_cancel(right_encoder_cb_a_);
  if (right_encoder_cb_b_ >= 0) callback_cancel(right_encoder_cb_b_);

  RCLCPP_INFO(rclcpp::get_logger("R1SystemHardware"), "Hardware deactivated.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ==================== 导出状态接口 ====================
std::vector<hardware_interface::StateInterface>
R1SystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocity_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_position_[i]));
  }
  return state_interfaces;
}

// ==================== 导出命令接口 ====================
std::vector<hardware_interface::CommandInterface>
R1SystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_velocity_[i]));
  }
  return command_interfaces;
}

// ==================== PID 计算 ====================
double R1SystemHardware::compute_pid(
  double target, double actual, double dt,
  double & integral, double & prev_error)
{
  if (dt <= 0.0) return 0.0;

  double error = target - actual;

  // 积分分离：误差大时不计积分
  const double integral_separation_threshold = 0.5;
  if (std::fabs(error) < integral_separation_threshold) {
    integral += error * dt;
    integral = std::max(-integral_clamp_, std::min(integral_clamp_, integral));
  }

  double derivative = (error - prev_error) / dt;
  prev_error = error;

  return kp_ * error + ki_ * integral + kd_ * derivative;
}

// ==================== read ====================
hardware_interface::return_type R1SystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  double dt = period.seconds();
  if (dt <= 0.0) return hardware_interface::return_type::OK;

  int32_t left_ticks = left_encoder_ticks_.load();
  int32_t right_ticks = right_encoder_ticks_.load();

  int32_t left_delta = left_ticks - prev_left_encoder_ticks_;
  int32_t right_delta = right_ticks - prev_right_encoder_ticks_;

  prev_left_encoder_ticks_ = left_ticks;
  prev_right_encoder_ticks_ = right_ticks;

  // 脉冲合理性检查：丢弃异常大的增量
  const int32_t max_plausible_delta = 300;  // x4 模式下 100Hz 周期的合理上限
  if (std::abs(left_delta) > max_plausible_delta) left_delta = 0;
  if (std::abs(right_delta) > max_plausible_delta) right_delta = 0;

  // x4 模式：每圈计数 = encoder_resolution × gear_ratio × 4
  const double ticks_per_rev = encoder_resolution_ * gear_ratio_ * 4.0;
  const double rad_per_tick = (2.0 * M_PI) / ticks_per_rev;

  double left_delta_rad = left_delta * rad_per_tick;
  double right_delta_rad = right_delta * rad_per_tick;

  double left_raw_vel = left_delta_rad / dt;
  double right_raw_vel = right_delta_rad / dt;

  // 一阶低通滤波
  left_filtered_velocity_ =
    velocity_filter_alpha_ * left_raw_vel +
    (1.0 - velocity_filter_alpha_) * left_filtered_velocity_;
  right_filtered_velocity_ =
    velocity_filter_alpha_ * right_raw_vel +
    (1.0 - velocity_filter_alpha_) * right_filtered_velocity_;

  // 零目标时冻结编码器读数，避免停止时的噪声
  const double zero_threshold = 0.05;
  if (std::fabs(hw_commands_velocity_[0]) < zero_threshold) {
    hw_states_velocity_[0] = 0.0;
    left_filtered_velocity_ = 0.0;
  } else {
    hw_states_velocity_[0] = left_filtered_velocity_;
  }

  if (std::fabs(hw_commands_velocity_[1]) < zero_threshold) {
    hw_states_velocity_[1] = 0.0;
    right_filtered_velocity_ = 0.0;
  } else {
    hw_states_velocity_[1] = right_filtered_velocity_;
  }

  hw_states_position_[0] += left_delta_rad;
  hw_states_position_[1] += right_delta_rad;

  return hardware_interface::return_type::OK;
}

// ==================== write ====================
hardware_interface::return_type R1SystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  double dt = period.seconds();
  if (dt <= 0.0) return hardware_interface::return_type::OK;

  double left_target = hw_commands_velocity_[0];
  double right_target = hw_commands_velocity_[1];

  const double zero_threshold = 0.05;

  // 后退速度缩放
  const double reverse_scale = 0.6;
  if (left_target < -zero_threshold) left_target *= reverse_scale;
  if (right_target < -zero_threshold) right_target *= reverse_scale;

  const double max_delta = max_accel_ * dt;

  auto set_motor = [this](int pwm_pin, int in1_pin, int in2_pin, double velocity) {
    double vel = std::max(-max_velocity_, std::min(max_velocity_, velocity));

    if (vel > 0.0) {
      gpio_write(pi_handle_, in1_pin, PI_HIGH);
      gpio_write(pi_handle_, in2_pin, PI_LOW);
    } else if (vel < 0.0) {
      gpio_write(pi_handle_, in1_pin, PI_LOW);
      gpio_write(pi_handle_, in2_pin, PI_HIGH);
    } else {
      gpio_write(pi_handle_, in1_pin, PI_LOW);
      gpio_write(pi_handle_, in2_pin, PI_LOW);
    }

    int duty = static_cast<int>(std::fabs(vel) / max_velocity_ * 255.0);
    duty = std::max(0, std::min(255, duty));
    set_PWM_dutycycle(pi_handle_, pwm_pin, duty);
  };

  // ---------- 左轮 ----------
  double left_final;
  double left_correction = 0.0;
  if (std::fabs(left_target) < zero_threshold) {
    left_final = 0.0;
    left_integral_ = 0.0;
    left_prev_error_ = 0.0;
    left_prev_output_ = 0.0;
  } else {
    if (left_target * left_prev_target_ < 0.0) {
      left_integral_ = 0.0;
      left_prev_error_ = 0.0;
    }
    double left_actual = hw_states_velocity_[0];
    left_correction = compute_pid(
      left_target, left_actual, dt, left_integral_, left_prev_error_);
    left_final = left_target + left_correction;

    if (left_final > left_prev_output_ + max_delta) {
      left_final = left_prev_output_ + max_delta;
    } else if (left_final < left_prev_output_ - max_delta) {
      left_final = left_prev_output_ - max_delta;
    }
    left_prev_output_ = left_final;
  }
  left_prev_target_ = left_target;

  // ---------- 右轮 ----------
  double right_final;
  double right_correction = 0.0;
  if (std::fabs(right_target) < zero_threshold) {
    right_final = 0.0;
    right_integral_ = 0.0;
    right_prev_error_ = 0.0;
    right_prev_output_ = 0.0;
  } else {
    if (right_target * right_prev_target_ < 0.0) {
      right_integral_ = 0.0;
      right_prev_error_ = 0.0;
    }
    double right_actual = hw_states_velocity_[1];
    right_correction = compute_pid(
      right_target, right_actual, dt, right_integral_, right_prev_error_);
    right_final = right_target + right_correction;

    if (right_final > right_prev_output_ + max_delta) {
      right_final = right_prev_output_ + max_delta;
    } else if (right_final < right_prev_output_ - max_delta) {
      right_final = right_prev_output_ - max_delta;
    }
    right_prev_output_ = right_final;
  }
  right_prev_target_ = right_target;

  // ---------- 左右轮速度耦合 ----------
  double coupling = 0.1;
  if (left_target < 0.0 || right_target < 0.0) {
    coupling = 0.3;
  }

  double avg = (left_final + right_final) / 2.0;
  double left_before_coupling = left_final;
  double right_before_coupling = right_final;
  left_final  = left_final  * (1.0 - coupling) + avg * coupling;
  right_final = right_final * (1.0 - coupling) + avg * coupling;

  // ---------- 输出到电机 ----------
  set_motor(left_pwm_pin_, left_ain1_pin_, left_ain2_pin_, left_final);
  set_motor(right_pwm_pin_, right_bin1_pin_, right_bin2_pin_, right_final);

  // ---------- 调试日志 ----------
  static rclcpp::Clock::SharedPtr log_clock = rclcpp::Clock::make_shared();
  RCLCPP_INFO_THROTTLE(
    rclcpp::get_logger("R1SystemHardware"),
    *log_clock, 500,
    "tgt[%.3f,%.3f] act[%.3f,%.3f] pid[%.3f,%.3f] "
    "pre_couple[%.3f,%.3f] final[%.3f,%.3f] coup=%.2f",
    left_target, right_target,
    hw_states_velocity_[0], hw_states_velocity_[1],
    left_correction, right_correction,
    left_before_coupling, right_before_coupling,
    left_final, right_final,
    coupling);

  return hardware_interface::return_type::OK;
}

}  // namespace r1_hardware_interface

// 导出插件
PLUGINLIB_EXPORT_CLASS(
  r1_hardware_interface::R1SystemHardware,
  hardware_interface::SystemInterface)