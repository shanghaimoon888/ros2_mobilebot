#include "r1_hardware_interface/r1_system_hardware.hpp"

#include <cmath>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace r1_hardware_interface
{

// ==================== 参数读取辅助 ====================
static std::string get_required_param(const hardware_interface::HardwareInfo & info, const std::string & key)
{
  auto it = info.hardware_parameters.find(key);
  if (it == info.hardware_parameters.end()) throw std::runtime_error("Missing parameter: " + key);
  return it->second;
}

static double get_optional_param(const hardware_interface::HardwareInfo & info, const std::string & key, double default_val)
{
  auto it = info.hardware_parameters.find(key);
  return (it != info.hardware_parameters.end()) ? std::stod(it->second) : default_val;
}



// ==================== 串口初始化辅助 ====================
/*int setup_serial_port(const std::string& port, int baudrate) {
    int fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) return -1;

    struct termios options;
    tcgetattr(fd, &options);
    cfsetispeed(&options, baudrate);
    cfsetospeed(&options, baudrate);
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_oflag &= ~OPOST;
    tcsetattr(fd, TCSANOW, &options);
	tcflush(fd, TCIOFLUSH);
    return fd;
}*/
/*int setup_serial_port(const std::string& port, int baudrate) {
    int fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) return -1;

    struct termios options;
    tcgetattr(fd, &options);

    // 1. 设置波特率
    cfsetispeed(&options, baudrate);
    cfsetospeed(&options, baudrate);

    // 2. 原始模式（关键）
    cfmakeraw(&options); 

    // 3. 硬件控制流、软件控制流关闭
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~CRTSCTS;
    options.c_iflag &= ~(IXON | IXOFF | IXANY);

    // 4. 【核心修复】设置阻塞读取超时
    // VMIN = 0, VTIME = 10 表示：如果没有数据，read 会阻塞等待最多 1 秒，期间一旦有数据就立刻返回
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 10; 

    tcsetattr(fd, TCSANOW, &options);
    tcflush(fd, TCIOFLUSH);
    return fd;
}*/
int setup_serial_port(const std::string& port, int baudrate) {
    // 注意：不要用 O_NDELAY，改用 O_NONBLOCK
    int fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd == -1) return -1;

    struct termios options;
    tcgetattr(fd, &options);

    // 1. 强制转换为“原始模式”（等同于 pyserial 的默认行为）
    cfmakeraw(&options);

    // 2. 设置波特率
    cfsetispeed(&options, baudrate);
    cfsetospeed(&options, baudrate);

    // 3. 关键：关闭硬件流控和软件流控！（你原来的代码漏了硬件流控）
    options.c_cflag &= ~CRTSCTS;
    options.c_iflag &= ~(IXON | IXOFF | IXANY);

    // 4. 8N1，启用接收
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;

    // 5. 非阻塞读取：只要有数据就立刻返回，没数据返回0
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 0;

    tcsetattr(fd, TCSANOW, &options);

    // 清空所有历史脏数据
    tcflush(fd, TCIOFLUSH); 

    return fd;
}

uint8_t calc_checksum(uint8_t *data, uint8_t len) {
    uint8_t sum = 0;
    for (int i = 0; i < len; i++) sum += data[i];
    return sum;
}

// ==================== 生命周期函数 ====================
hardware_interface::CallbackReturn R1SystemHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
    return hardware_interface::CallbackReturn::ERROR;

  try {
    // 从 Xacro 的 <param> 中读取
    serial_port_ = get_required_param(info, "serial_port");
    baud_rate_ = std::stoi(get_required_param(info, "baud_rate"));
    wheel_radius_ = std::stod(get_required_param(info, "wheel_radius"));
    
    // 这里也可以从 param 读，如果你串口发的是线速度(m/s)，这里就用来转换
    wheel_separation_ = get_optional_param(info, "wheel_separation", 0.16); 
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("R1SystemHardware"), "Param error: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  const size_t n = info_.joints.size();
  hw_commands_velocity_.assign(n, 0.0);
  hw_states_velocity_.assign(n, 0.0);
  hw_states_position_.assign(n, 0.0);

  RCLCPP_INFO(rclcpp::get_logger("R1SystemHardware"), "Initialized. Serial: %s, Baud: %d", serial_port_.c_str(), baud_rate_);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn R1SystemHardware::on_configure(const rclcpp_lifecycle::State &)
{
  serial_fd_ = setup_serial_port(serial_port_, baud_rate_);
  if (serial_fd_ < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("R1SystemHardware"), "Failed to open serial port: %s", serial_port_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }
   
  RCLCPP_INFO(rclcpp::get_logger("R1SystemHardware"), "Serial port configured.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn R1SystemHardware::on_activate(const rclcpp_lifecycle::State &)
{
  running_ = true;
  serial_thread_ = std::thread(&R1SystemHardware::serial_receive_loop, this);
  
  for(int i=0; i<5; i++) {
      send_speed_command(0, 0);
      usleep(10000); // 间隔 10ms
  }
  
  RCLCPP_INFO(rclcpp::get_logger("R1SystemHardware"), "Hardware activated. Serial thread started.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn R1SystemHardware::on_deactivate(const rclcpp_lifecycle::State &)
{
  running_ = false;
  if (serial_thread_.joinable()) serial_thread_.join();
  
  if (serial_fd_ >= 0) {
    // 发送 0 速度确保停车
    send_speed_command(0, 0);
    close(serial_fd_);
    serial_fd_ = -1;
  }
  
  RCLCPP_INFO(rclcpp::get_logger("R1SystemHardware"), "Hardware deactivated.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ==================== 接口导出 ====================
std::vector<hardware_interface::StateInterface> R1SystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocity_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_position_[i]));
  }
  
// ======================== 导出 IMU 接口 ========================
  // 姿态
  state_interfaces.emplace_back(hardware_interface::StateInterface("imu_sensor", "orientation.x", &hw_states_imu_q_x_));
  state_interfaces.emplace_back(hardware_interface::StateInterface("imu_sensor", "orientation.y", &hw_states_imu_q_y_));
  state_interfaces.emplace_back(hardware_interface::StateInterface("imu_sensor", "orientation.z", &hw_states_imu_q_z_));
  state_interfaces.emplace_back(hardware_interface::StateInterface("imu_sensor", "orientation.w", &hw_states_imu_q_w_));
  
  // 角速度
  state_interfaces.emplace_back(hardware_interface::StateInterface("imu_sensor", "angular_velocity.x", &hw_states_imu_angular_x_));
  state_interfaces.emplace_back(hardware_interface::StateInterface("imu_sensor", "angular_velocity.y", &hw_states_imu_angular_y_));
  state_interfaces.emplace_back(hardware_interface::StateInterface("imu_sensor", "angular_velocity.z", &hw_states_imu_angular_z_));
  
  // 线加速度
  state_interfaces.emplace_back(hardware_interface::StateInterface("imu_sensor", "linear_acceleration.x", &hw_states_imu_linear_x_));
  state_interfaces.emplace_back(hardware_interface::StateInterface("imu_sensor", "linear_acceleration.y", &hw_states_imu_linear_y_));
  state_interfaces.emplace_back(hardware_interface::StateInterface("imu_sensor", "linear_acceleration.z", &hw_states_imu_linear_z_)); 
  
  // 距离
  state_interfaces.emplace_back(hardware_interface::StateInterface("range_sensor", "range", &hw_states_range_));
	
	
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> R1SystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_velocity_[i]));
  }
  return command_interfaces;
}

// ==================== 核心：read 与 write ====================
hardware_interface::return_type R1SystemHardware::read(const rclcpp::Time &, const rclcpp::Duration & period)
{
  // 直接在 read() 里死等串口是大忌！所以真实速度更新全在后台线程里完成。
  // read() 只需要极速把最新的数据从缓存里交出去。
  
  std::lock_guard<std::mutex> lock(data_mutex_);
  
  // 位置积分 (使用后台线程更新的速度)
  if (period.seconds() > 0) {
    hw_states_position_[0] += hw_states_velocity_[0] * period.seconds();
    hw_states_position_[1] += hw_states_velocity_[1] * period.seconds();
  }
  
   // 2. 欧拉角转四元数
  double roll_rad = hw_states_imu_roll_ * M_PI / 180.0;
  double pitch_rad = hw_states_imu_pitch_ * M_PI / 180.0;
  double yaw_rad = hw_states_imu_yaw_ * M_PI / 180.0;

  double cy = cos(yaw_rad * 0.5);
  double sy = sin(yaw_rad * 0.5);
  double cp = cos(pitch_rad * 0.5);
  double sp = sin(pitch_rad * 0.5);
  double cr = cos(roll_rad * 0.5);
  double sr = sin(roll_rad * 0.5);

  hw_states_imu_q_w_ = cr * cp * cy + sr * sp * sy;
  hw_states_imu_q_x_ = sr * cp * cy - cr * sp * sy;
  hw_states_imu_q_y_ = cr * sp * cy + sr * cp * sy;
  hw_states_imu_q_z_ = cr * cp * sy - sr * sp * cy;
  
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type R1SystemHardware::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  // 1. 读取控制器发来的目标轮速 (rad/s)
  double left_target_rad = hw_commands_velocity_[0];
  double right_target_rad = hw_commands_velocity_[1];

  // 2. 将 rad/s 转换为线速度 mm/s，发给 STM32
  // v = omega * r
  int16_t left_mm_s = static_cast<int16_t>(left_target_rad * wheel_radius_ * 1000.0);
  int16_t right_mm_s = static_cast<int16_t>(right_target_rad * wheel_radius_ * 1000.0);

  // 3. 发送给 STM32
  send_speed_command(left_mm_s, right_mm_s);

  return hardware_interface::return_type::OK;
}

// ==================== 串口发送与接收线程 ====================
void R1SystemHardware::send_speed_command(int16_t left_mm_s, int16_t right_mm_s)
{
  if (serial_fd_ < 0) return;

  uint8_t buffer[9];
  buffer[0] = 0xAA;
  buffer[1] = 0x55;
  buffer[2] = 5;    // 长度 (Cmd + 数据 4字节)
  buffer[3] = 0x01; // 功能码：设置速度
  buffer[4] = left_mm_s & 0xFF;
  buffer[5] = (left_mm_s >> 8) & 0xFF;
  buffer[6] = right_mm_s & 0xFF;
  buffer[7] = (right_mm_s >> 8) & 0xFF;
  buffer[8] = calc_checksum(&buffer[2], 6);

  ::write(serial_fd_, buffer, 9);
}

#if 0
void R1SystemHardware::serial_receive_loop()
{
  uint8_t rx_buffer[128];
  int state = 0; 
  int rx_index = 0;
  
  // 用于一次性读取的临时缓冲
  uint8_t read_buf[32]; 
  
  RCLCPP_INFO(rclcpp::get_logger("R1SystemHardware"), "Serial receive thread started.");
  
  while (running_ && rclcpp::ok()) {
    // 【核心修复】：一次最多读 32 个字节，而不是每次只读 1 个字节
    int n = ::read(serial_fd_, read_buf, sizeof(read_buf));
    
    if (n <= 0) {
        continue; // 因为设了 VTIME，如果没数据会自动阻塞，不需要 usleep
    }

    // 把读到的数据逐个字节丢进状态机
    for (int i = 0; i < n; i++) {
        uint8_t byte = read_buf[i];

        switch (state) {
          case 0:
            if (byte == 0xAA) state = 1;
            break;
          case 1:
            if (byte == 0x55) state = 2;
            else state = 0;
            break;
          case 2:
            rx_buffer[2] = byte;
            rx_index = 0;
            state = 3;
            break;
          case 3:
            rx_buffer[3 + rx_index] = byte;
            rx_index++;
            if (rx_index >= rx_buffer[2]) state = 4;
            break;
          case 4:
            {
              // 校验（+1 是把功能码算上）
              uint8_t calc = calc_checksum(&rx_buffer[2], rx_buffer[2] + 1);
              if (calc == byte) {
                // 校验通过！
                if (rx_buffer[3] == 0x83) { 
                  uint16_t left_raw = (static_cast<uint16_t>(rx_buffer[5]) << 8) | static_cast<uint16_t>(rx_buffer[4]);
                  int16_t left_mm_s = static_cast<int16_t>(left_raw);
                  uint16_t right_raw = (static_cast<uint16_t>(rx_buffer[7]) << 8) | static_cast<uint16_t>(rx_buffer[6]);
                  int16_t right_mm_s = static_cast<int16_t>(right_raw);
                  
                  uint16_t yaw_raw = (static_cast<uint16_t>(rx_buffer[9]) << 8) | static_cast<uint16_t>(rx_buffer[8]);
                  int16_t yaw_100 = static_cast<int16_t>(yaw_raw);
                  uint16_t pitch_raw = (static_cast<uint16_t>(rx_buffer[11]) << 8) | static_cast<uint16_t>(rx_buffer[10]);
                  int16_t pitch_100 = static_cast<int16_t>(pitch_raw);
                  uint16_t roll_raw = (static_cast<uint16_t>(rx_buffer[13]) << 8) | static_cast<uint16_t>(rx_buffer[12]);
                  int16_t roll_100 = static_cast<int16_t>(roll_raw);

                  uint16_t gz_raw = (static_cast<uint16_t>(rx_buffer[15]) << 8) | static_cast<uint16_t>(rx_buffer[14]);
                  int16_t gz = static_cast<int16_t>(gz_raw);

                  uint16_t ax_raw = (static_cast<uint16_t>(rx_buffer[17]) << 8) | static_cast<uint16_t>(rx_buffer[16]);
                  int16_t ax = static_cast<int16_t>(ax_raw);
                  uint16_t ay_raw = (static_cast<uint16_t>(rx_buffer[19]) << 8) | static_cast<uint16_t>(rx_buffer[18]);
                  int16_t ay = static_cast<int16_t>(ay_raw);

                  std::lock_guard<std::mutex> lock(data_mutex_);
                  hw_states_velocity_[0] = (left_mm_s / 1000.0) / wheel_radius_;
                  hw_states_velocity_[1] = (right_mm_s / 1000.0) / wheel_radius_;
                  
                  hw_states_imu_yaw_ = yaw_100 / 100.0;
                  hw_states_imu_pitch_ = pitch_100 / 100.0;
                  hw_states_imu_roll_ = roll_100 / 100.0;

                  hw_states_imu_angular_z_ = (gz / 16.4) * M_PI / 180.0; 
                  
                  hw_states_imu_linear_x_ = (ax / 16384.0) * 9.81;
                  hw_states_imu_linear_y_ = (ay / 16384.0) * 9.81;
                  hw_states_imu_linear_z_ = 9.81;
				  
				  static rclcpp::Clock::SharedPtr log_clock = rclcpp::Clock::make_shared();
				  // 【日志5】：解析成功的数值
                  RCLCPP_INFO_THROTTLE(
                    rclcpp::get_logger("R1Serial"), *log_clock, 500,
                    "解析成功！Yaw: %.2f, Pitch: %.2f | 速度 L: %d, R: %d", 
                    hw_states_imu_yaw_, hw_states_imu_pitch_, left_mm_s, right_mm_s);
                }
              }
              state = 0; // 恢复等待帧头
            }
            break;
        }
    }
  }
}
#endif

void R1SystemHardware::serial_receive_loop()
{
  uint8_t rx_buffer[128];
  int rx_index = 0;
  int state = 0; // 0:找0xAA, 1:找0x55, 2:接收长度, 3:接收数据, 4:接收校验和
  
  RCLCPP_INFO(rclcpp::get_logger("R1SystemHardware"), "Serial receive thread started.");
  
  while (running_ && rclcpp::ok()) {
    uint8_t byte;
    int n = ::read(serial_fd_, &byte, 1);
    
    if (n <= 0) {
      usleep(1000); // 没有数据，休眠 1ms 防止 CPU 100%
      continue;
    }
	
	//RCLCPP_INFO(rclcpp::get_logger("SerialRaw"), "收到字节: 0x%02X", byte);

    switch (state) {
      case 0:
        if (byte == 0xAA) state = 1;
        break;
      case 1:
        if (byte == 0x55) state = 2;
        else state = 0;
        break;
      case 2:
        rx_buffer[2] = byte;
        rx_index = 0;
        state = 3;
        break;
      case 3:
        rx_buffer[3 + rx_index] = byte;
        rx_index++;
        if (rx_index >= rx_buffer[2]) state = 4;
        break;
      case 4:
        {
          // 【修复1】：长度要 +1，把功能码和4个数据字节都算进去（共6字节）
          uint8_t calc = calc_checksum(&rx_buffer[2], rx_buffer[2] + 1);
          
          if (calc == byte) {
#if 0			  
			  // 校验通过！判断功能码
            if (rx_buffer[3] == 0x83) { // 超级反馈帧
              
              // 1. 提取速度
              uint16_t left_raw = (static_cast<uint16_t>(rx_buffer[5]) << 8) | static_cast<uint16_t>(rx_buffer[4]);
              int16_t left_mm_s = static_cast<int16_t>(left_raw);
              uint16_t right_raw = (static_cast<uint16_t>(rx_buffer[7]) << 8) | static_cast<uint16_t>(rx_buffer[6]);
              int16_t right_mm_s = static_cast<int16_t>(right_raw);
              
              // 2. 提取姿态
              uint16_t yaw_raw = (static_cast<uint16_t>(rx_buffer[9]) << 8) | static_cast<uint16_t>(rx_buffer[8]);
              int16_t yaw_100 = static_cast<int16_t>(yaw_raw);
              uint16_t pitch_raw = (static_cast<uint16_t>(rx_buffer[11]) << 8) | static_cast<uint16_t>(rx_buffer[10]);
              int16_t pitch_100 = static_cast<int16_t>(pitch_raw);
              uint16_t roll_raw = (static_cast<uint16_t>(rx_buffer[13]) << 8) | static_cast<uint16_t>(rx_buffer[12]);
              int16_t roll_100 = static_cast<int16_t>(roll_raw);

              // 3. 提取角速度
              uint16_t gz_raw = (static_cast<uint16_t>(rx_buffer[15]) << 8) | static_cast<uint16_t>(rx_buffer[14]);
              int16_t gz = static_cast<int16_t>(gz_raw);

              // 4. 提取加速度
              uint16_t ax_raw = (static_cast<uint16_t>(rx_buffer[17]) << 8) | static_cast<uint16_t>(rx_buffer[16]);
              int16_t ax = static_cast<int16_t>(ax_raw);
              uint16_t ay_raw = (static_cast<uint16_t>(rx_buffer[19]) << 8) | static_cast<uint16_t>(rx_buffer[18]);
              int16_t ay = static_cast<int16_t>(ay_raw);

              std::lock_guard<std::mutex> lock(data_mutex_);
              // 存速度
              hw_states_velocity_[0] = (left_mm_s / 1000.0) / wheel_radius_;
              hw_states_velocity_[1] = (right_mm_s / 1000.0) / wheel_radius_;
              
              // 存姿态
              hw_states_imu_yaw_ = yaw_100 / 100.0;
              hw_states_imu_pitch_ = pitch_100 / 100.0;
              hw_states_imu_roll_ = roll_100 / 100.0;

              // 存角速度 (弧度/秒)
              hw_states_imu_angular_z_ = (gz / 16.4) * M_PI / 180.0; 
              
              // 存加速度 (米/秒^2)
              hw_states_imu_linear_x_ = (ax / 16384.0) * 9.81;
              hw_states_imu_linear_y_ = (ay / 16384.0) * 9.81;
              hw_states_imu_linear_z_ = 9.81;
            }
			
#endif			
			
			  
#if 1
            // 校验通过！解析数据
            if (rx_buffer[3] == 0x81) { // 底盘状态反馈
              
              // 【修复2】：先转成无符号的 uint16_t，再强转成有符号的 int16_t
              uint16_t left_raw = (static_cast<uint16_t>(rx_buffer[5]) << 8) | static_cast<uint16_t>(rx_buffer[4]);
              int16_t left_mm_s = static_cast<int16_t>(left_raw);
              
              uint16_t right_raw = (static_cast<uint16_t>(rx_buffer[7]) << 8) | static_cast<uint16_t>(rx_buffer[6]);
              int16_t right_mm_s = static_cast<int16_t>(right_raw);
              
              // 将 mm/s 转换回 rad/s，存给状态接口
              std::lock_guard<std::mutex> lock(data_mutex_);
              hw_states_velocity_[0] = (left_mm_s / 1000.0) / wheel_radius_;
              hw_states_velocity_[1] = (right_mm_s / 1000.0) / wheel_radius_;
			  
			  static rclcpp::Clock::SharedPtr log_clock = rclcpp::Clock::make_shared();
              RCLCPP_INFO_THROTTLE(
                rclcpp::get_logger("R1SystemHardware"),
                *log_clock, 500,
                "【RX from STM32】 L: %d mm/s, R: %d mm/s  -->  ROS L: %.2f rad/s, R: %.2f rad/s",
                left_mm_s, right_mm_s, 
                hw_states_velocity_[0], hw_states_velocity_[1]);
            }
			// ================= 2. 解析 IMU 姿态反馈 0x82 =================
            else if (rx_buffer[3] == 0x82) {
             // 提取 Yaw
              uint16_t yaw_raw = (static_cast<uint16_t>(rx_buffer[5]) << 8) | static_cast<uint16_t>(rx_buffer[4]);
              int16_t yaw_100 = static_cast<int16_t>(yaw_raw);

              // 提取 Pitch
              uint16_t pitch_raw = (static_cast<uint16_t>(rx_buffer[7]) << 8) | static_cast<uint16_t>(rx_buffer[6]);
              int16_t pitch_100 = static_cast<int16_t>(pitch_raw);

              // 提取 Roll (新增)
              uint16_t roll_raw = (static_cast<uint16_t>(rx_buffer[9]) << 8) | static_cast<uint16_t>(rx_buffer[8]);
              int16_t roll_100 = static_cast<int16_t>(roll_raw);

			  // 2. 提取角速度 Z (原始 LSB)
              uint16_t gz_raw = (static_cast<uint16_t>(rx_buffer[11]) << 8) | static_cast<uint16_t>(rx_buffer[10]);
              int16_t gz = static_cast<int16_t>(gz_raw);

              // 3. 提取加速度 X / Y (原始 LSB)
              uint16_t ax_raw = (static_cast<uint16_t>(rx_buffer[13]) << 8) | static_cast<uint16_t>(rx_buffer[12]);
              int16_t ax = static_cast<int16_t>(ax_raw);
              
              uint16_t ay_raw = (static_cast<uint16_t>(rx_buffer[15]) << 8) | static_cast<uint16_t>(rx_buffer[14]);
              int16_t ay = static_cast<int16_t>(ay_raw);

              // 4. 存入变量并转换单位
              std::lock_guard<std::mutex> lock(data_mutex_);
              hw_states_imu_yaw_ = yaw_100 / 100.0;
              hw_states_imu_pitch_ = pitch_100 / 100.0;
              hw_states_imu_roll_ = roll_100 / 100.0;

              // 角速度：+2000dps 量程下，16.4 LSB = 1度/秒。
              // 转成弧度/秒：(gz / 16.4) * (PI / 180)
              hw_states_imu_angular_z_ = (gz / 16.4) * M_PI / 180.0; 
              hw_states_imu_angular_x_ = 0.0; 
              hw_states_imu_angular_y_ = 0.0; 

              // 线加速度：+2g 量程下，16384 LSB = 1g。
              // 转成 m/s^2：(ax / 16384.0) * 9.81
              hw_states_imu_linear_x_ = (ax / 16384.0) * 9.81;
              hw_states_imu_linear_y_ = (ay / 16384.0) * 9.81;
              hw_states_imu_linear_z_ = 9.81; // 默认平放，Z 轴就是重力
            }
			else if(rx_buffer[3] == 0x84)
			{
			  // 假设: 0x84帧，4号字节低字节，5号高字节，单位 cm
			  uint16_t dist_cm_raw = (static_cast<uint16_t>(rx_buffer[5]) << 8) | static_cast<uint16_t>(rx_buffer[4]);
			  double dist_m = static_cast<double>(dist_cm_raw) / 100.0; // 转为米，ROS标准单位

			  std::lock_guard<std::mutex> lock(data_mutex_);
			  hw_states_range_ = dist_m;

			  static rclcpp::Clock::SharedPtr log_clock = rclcpp::Clock::make_shared();
			  RCLCPP_INFO_THROTTLE(
				rclcpp::get_logger("R1Range"),
				*log_clock, 500,
				"【测距0x84】 raw cm: %u, range: %.3f m",
				dist_cm_raw, hw_states_range_
			  );
			}
#endif

          }
          state = 0; // 重新找下一帧
        }
        break;
    }
  }
}

}  // namespace r1_hardware_interface

PLUGINLIB_EXPORT_CLASS(
  r1_hardware_interface::R1SystemHardware,
  hardware_interface::SystemInterface)