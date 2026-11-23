/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
#define PI 3.1415926f

// --- 小车物理参数配置区（以后换硬件只改这里） ---
#define ENCODER_PPR         13.0f   // 编码器线数 (电机转一圈的脉冲数)
#define ENCODER_MULTIPLIER  4.0f    // 四倍频计数 (A/B相上下沿都算)
#define GEAR_RATIO          20.0f   // 减速比
#define WHEEL_RADIUS_MM     24.0f   // 轮子半径 (毫米) (2.4cm = 24mm)
#define CONTROL_PERIOD_MS   10.0f   // PID控制周期 (毫秒)，对应TIM6的中断时间

// --- 自动推导的常量 ---

// 轮子转一圈总共的脉冲数： 13 * 4 * 20 = 1040
#define PULSES_PER_WHEEL_REV (ENCODER_PPR * ENCODER_MULTIPLIER * GEAR_RATIO)

// 轮子的周长 (毫米)： 2 * PI * 24 ≈ 150.8 mm
#define WHEEL_CIRCUMFERENCE  (2.0f * PI * WHEEL_RADIUS_MM)

// 最终的速度换算系数
// 公式推导： (1000ms / 控制周期) * (轮子周长 / 轮子一圈脉冲数)
// 现在代入： (1000 / 10) * (150.8 / 1040) = 100 * 0.145 = 14.5
#define SPEED_COEFFICIENT    ((1000.0f / CONTROL_PERIOD_MS) * (WHEEL_CIRCUMFERENCE / PULSES_PER_WHEEL_REV))

// --- 1. 电机和PID结构体 ---
typedef struct {
    float target_speed; // 目标速度 (mm/s)
    float actual_speed; // 实际速度 (mm/s)
    float Kp, Ki, Kd;
    float error, last_error, prev_error;
    float pid_out;
    TIM_HandleTypeDef* encoder_tim;
    int16_t last_count;
	  float total_pulses; 

} Wheel_t;

// 实例化左右轮（假设左轮用TIM3，右轮用TIM4，PID参数先给个基础值）
Wheel_t Wheel_Left  = {0, 0, 0.15f, 0.05f, 0.0, 0, 0, 0, 0, &htim3, 0, 0};
Wheel_t Wheel_Right = {0, 0, 0.15f, 0.05f, 0.0, 0, 0, 0, 0, &htim4, 0, 0};

// --- 2. 串口接收相关变量 ---
uint8_t rx_data;          // 接收单个字节
uint8_t rx_data2;          // 接收单个字节
char rx_buffer[50];       // 接收字符串缓冲区
uint8_t rx_index = 0;     // 缓冲区索引
uint8_t rx_flag = 0;      // 接收完成标志

// --- 3. 串口打印计时 ---
uint32_t print_time = 0;

uint8_t square_state = 0; // 0:直行, 1:停止+准备转向, 2:转向, 3:停止+准备直行, 4:完成

// 串口2协议接收状态机
typedef enum {
    WAIT_HEADER1,
    WAIT_HEADER2,
    WAIT_LEN,
    WAIT_PAYLOAD,
    WAIT_CHECKSUM
} ParseState;

ParseState uart2_state = WAIT_HEADER1;
uint8_t uart2_rx_buf[20]; // 接收缓冲区
uint8_t uart2_rx_index = 0;
uint32_t uart2_tx_time = 0; // 串口2发送反馈的定时器
uint32_t last_rx_tick = 0;

// 累加和校验函数
uint8_t calc_checksum(uint8_t *data, uint8_t len) {
    uint8_t sum = 0;
    for (uint8_t i = 0; i < len; i++) sum += data[i];
    return sum;
}

float target_yaw = 0.0;       // 目标航向 (起步时记录一次)
uint8_t first_yaw_flag = 0;   // 标志位：是否记录了初始朝向

// 超声波引脚定义（根据你的实际接线修改）
#define TRIG_PIN GPIO_PIN_7
#define TRIG_PORT GPIOC
#define ECHO_PIN GPIO_PIN_6
#define ECHO_PORT GPIOC

float current_distance = 0.0f; // 存放当前距离 (厘米)

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#include <stdio.h>
#include <stdlib.h> // 用于 atof/atoi
#include <string.h>
#include <math.h>
#include "MPU6050.h" 
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"


// 超声波测距函数
float Get_Distance(void)
{
    uint32_t timeout = 0;
    uint32_t start_time = 0;
    uint32_t end_time = 0;

    // 1. 给 Trig 一个 10us 的高电平脉冲
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);
    delay_us(10); 
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);

    // 2. 等待 Echo 变高 (带有超时防死锁)
    while (HAL_GPIO_ReadPin(ECHO_PORT, ECHO_PIN) == GPIO_PIN_RESET) {
        timeout++;
        if (timeout > 100000) return 0.0f; // 超时，返回 0
    }

    // 3. 记录高电平开始的时间
    __HAL_TIM_SET_COUNTER(&htim2, 0); // 清零定时器
    HAL_TIM_Base_Start(&htim2);       // 开启定时器

    // 4. 等待 Echo 变低 (带有超时防死锁)
    while (HAL_GPIO_ReadPin(ECHO_PORT, ECHO_PIN) == GPIO_PIN_SET) {
        // 如果计数器超过 60000（即 60ms），说明距离超过 10 米，或者模块坏了
        if (__HAL_TIM_GET_COUNTER(&htim2) > 60000) {
            HAL_TIM_Base_Stop(&htim2);
            return 0.0f; 
        }
    }

    // 5. 记录结束时间
    end_time = __HAL_TIM_GET_COUNTER(&htim2);
    HAL_TIM_Base_Stop(&htim2); // 关闭定时器

    // 6. 计算距离（单位：厘米）
    // 时间单位是微秒，声速 340m/s = 34cm/ms = 0.034cm/us
    // 距离 = (高电平时间 * 0.034) / 2
    float dist = (end_time * 0.034f) / 2.0f;

    // 7. 简单滤波，剔除异常值（比如超过 4 米或小于 2 厘米）
    if (dist > 400.0f || dist < 2.0f) {
        return 0.0f;
    }

    return dist;
}

// 计算单个轮子的PID
void Calc_Wheel_PID(Wheel_t* wheel) {
	

    // 1. 读取编码器并计算实际速度
    int16_t current_count = (int16_t)__HAL_TIM_GET_COUNTER(wheel->encoder_tim);
    int16_t speed_pulses = current_count - wheel->last_count;
    wheel->last_count = current_count;
	
		if (speed_pulses > 20 || speed_pulses < -20) {
			HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_9); // 如果卡顿时 LED 闪了，就100%是编码器干扰
	  }
	
		if (speed_pulses > 50) speed_pulses = 50;
    if (speed_pulses < -50) speed_pulses = -50;
	
		wheel->total_pulses += (float)speed_pulses; 
    wheel->actual_speed = (float)speed_pulses * 14.5; // 10ms周期的系数
	
		//wheel->pid_out = wheel->target_speed;
	  //return;

    // 2. 计算误差
    wheel->error = wheel->target_speed - wheel->actual_speed;

    // 3. 增量式 PID
    float p_term = wheel->Kp * (wheel->error - wheel->last_error);
    float i_term = wheel->Ki * wheel->error;
    float d_term = wheel->Kd * (wheel->error - 2 * wheel->last_error + wheel->prev_error);
    
    wheel->pid_out += (p_term + i_term + d_term); 

    // 4. 限幅 (-99 到 99)
    if (wheel->pid_out > 99.0)  wheel->pid_out = 99.0;
    if (wheel->pid_out < -99.0) wheel->pid_out = -99.0;

    // 5. 更新历史
    wheel->prev_error = wheel->last_error;
    wheel->last_error = wheel->error;
}

// 重定向 printf 到 USART1
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
    // 注意：这里默认你用 USART1 打印，如果你的串口是其他句柄，把 huart1 换掉
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM6_Init();
  MX_TIM10_Init();
  MX_TIM11_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_I2C2_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	// 1. 启动 PWM (左轮PF6->TIM10, 右轮PF7->TIM11)
  HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1);
  
  // 2. 启动编码器 (左轮TIM3, 右轮TIM4)
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  
  // 3. 启动 PID 心跳定时器 (TIM6, 10ms)
  HAL_TIM_Base_Start_IT(&htim6);
  
  // 4. 启动串口中断接收（接收树莓派指令）
  HAL_UART_Receive_IT(&huart1, &rx_data, 1);
	HAL_UART_Receive_IT(&huart2, &rx_data2, 1); 
	
	
	
	// ==================== MPU6050 通信测试 (I2C2) ====================
  /*uint8_t mpu_id = 0;
  uint8_t tx_data = 0x75; // 要读取的寄存器地址 (WHO_AM_I)
  
  // 注意：这里换成了 &hi2c2
  HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(&hi2c2, (0x68 << 1), &tx_data, 1, 100);
  
  if (status == HAL_OK) {
      // 注意：这里也换成了 &hi2c2
      status = HAL_I2C_Master_Receive(&hi2c2, (0x68 << 1), &mpu_id, 1, 100);
      if (status == HAL_OK) {
          char msg[50];
          sprintf(msg, "MPU6050 ID: 0x%02X\r\n", mpu_id);
          HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
      } else {
          HAL_UART_Transmit(&huart1, (uint8_t*)"I2C Read Failed\r\n", 17, 100);
      }
  } else {
      HAL_UART_Transmit(&huart1, (uint8_t*)"I2C Transmit Failed\r\n", 22, 100);
  }*/
  // ========================================================
	uint8_t tx_buf[100];
  uint8_t mpu_id = 0;	
	//IIC_Init(); 
  delay_init(); 
  HAL_UART_Transmit(&huart1, (uint8_t *)"Init MPU6050...\r\n", 17, 100);
  MPU6050_initialize();  
  HAL_UART_Transmit(&huart1, (uint8_t *)"Init DMP...\r\n", 13, 100);
  DMP_Init();
  HAL_UART_Transmit(&huart1, (uint8_t *)"DMP Init Success!\r\n", 19, 100);
	
	//int ret = i2cRead(0x68, 0x75, 1, &mpu_id);
	

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	float target_distance = 500.0f * 6.896f; // 直行 500mm 需要的脉冲数 (3448)
  float target_angle = 812.0f;             // 转 90 度需要的脉冲数 (假设轮距 150mm)
  uint8_t square_round = 0;                // 记录走了几条边了
  uint8_t state = 0;                       // 状态机
  uint32_t state_timer = 0;
	uint8_t raw_buf[14];
	
  while (1)
  {
		  //直接读取6轴数据
		  /*MPU6050_Get_Accelscope();
		  MPU6050_Get_Gyroscope();
      sprintf(tx_buf, "Accel:%d %d %d\r\n", accel[0], accel[1], accel[2]);
      HAL_UART_Transmit(&huart1, (uint8_t*)tx_buf, strlen(tx_buf), 100);      
      sprintf(tx_buf, "Gyro: %d %d %d\r\n", gyro[0], gyro[1], gyro[2]);
      HAL_UART_Transmit(&huart1, (uint8_t*)tx_buf, strlen(tx_buf), 100);     
      HAL_UART_Transmit(&huart1, (uint8_t*)"\r\n", 2, 100);
		  delay_ms(200);
			continue;*/
	
		  //读取dmp数据
		  Read_DMP();       
		  HAL_Delay(10); // 让出 CPU，防止读太频繁

      // 记录第一次静止时的 Yaw 作为初始基准 (0 度)
      if (!first_yaw_flag && Yaw != 0.0) {
          target_yaw = Yaw;
          first_yaw_flag = 1;
      }

#if 0
      // ==================== 新增：航向修正逻辑 ====================
      // 只有当树莓派发来“直行”指令时，才开启航向锁定
      // 如果左右轮目标速度相近（说明没有在转弯）
      if (fabs(Wheel_Left.target_speed) > 10 && fabs(Wheel_Left.target_speed - Wheel_Right.target_speed) < 10) 
      {
          // 计算当前航向与目标航向的偏差
          float error_yaw = target_yaw - Yaw; 
          
          // 处理过零度问题（比如从 179 度变到 -179 度）
          if (error_yaw > 180.0f)  error_yaw -= 360.0f;
          if (error_yaw < -180.0f) error_yaw += 360.0f;

          // 比例系数 (Kp)，如果转得太猛就调小，太迟钝就调大
          float kp_yaw = 1.5f; 
          
          // 计算修正量并叠加到左右轮的 PID 目标上
          float correction = error_yaw * kp_yaw;
          
          Wheel_Left.target_speed  += correction; 
          Wheel_Right.target_speed -= correction;
          
          // 防止修正过度导致速度爆表
          if (Wheel_Left.target_speed > 200) Wheel_Left.target_speed = 200;
          if (Wheel_Right.target_speed > 200) Wheel_Right.target_speed = 200;
      } else {
          // 如果正在原地转弯或倒车，就不断刷新基准航向，免得转完弯后往回疯狂修正
          target_yaw = Yaw; 
      }
      // ============================================================
#endif

#if 0 //走正方形测试代码
		// 每 1 秒打印一次里程，方便调试
      if (HAL_GetTick() - print_time >= 1000) {
          print_time = HAL_GetTick();
          char tx_buf[60];
          sprintf(tx_buf, "L_pulses:%.0f, R_pulses:%.0f, State:%d, Round:%d\r\n", 
                  Wheel_Left.total_pulses, Wheel_Right.total_pulses, state, square_round);
          HAL_UART_Transmit(&huart1, (uint8_t*)tx_buf, strlen(tx_buf), 100);
      }

      switch (state) {
          case 0: // 直行
              Wheel_Left.target_speed = 300.0f;  // 目标速度 300 mm/s
              Wheel_Right.target_speed = 300.0f;
              
              // 如果左轮或右轮的总脉冲达到了 500mm 的距离
              if (fabs(Wheel_Left.total_pulses) >= target_distance || 
                  fabs(Wheel_Right.total_pulses) >= target_distance) 
              {
                  // 重置脉冲计数，准备转向
                  Wheel_Left.total_pulses = 0; 
                  Wheel_Right.total_pulses = 0;
                  state = 1; 
                  state_timer = HAL_GetTick(); 
              }
              break;

          case 1: // 停车 0.5 秒（等惯性消失）
              Wheel_Left.target_speed = 0;
              Wheel_Right.target_speed = 0;
              if (HAL_GetTick() - state_timer >= 500) {
                  state = 2; // 进入转向
              }
              break;

          case 2: // 原地右转 90 度
              // 左轮前进，右轮后退
              Wheel_Left.target_speed = 200.0f;  
              Wheel_Right.target_speed = -200.0f; 
              
              // 只要有一个轮子转了足够的角度，就停止
              if (fabs(Wheel_Left.total_pulses) >= target_angle || 
                  fabs(Wheel_Right.total_pulses) >= target_angle) 
              {
                  Wheel_Left.total_pulses = 0; 
                  Wheel_Right.total_pulses = 0;
                  state = 3;
                  state_timer = HAL_GetTick(); 
              }
              break;

          case 3: // 停车 0.5 秒（等转向停稳）
              Wheel_Left.target_speed = 0;
              Wheel_Right.target_speed = 0;
              if (HAL_GetTick() - state_timer >= 500) {
                  square_round++; // 完成了一条边
                  if (square_round >= 4) {
                      state = 4; // 4条边走完，结束
                  } else {
                      state = 0; // 继续走下一条边
                  }
              }
              break;

          case 4: // 结束
              Wheel_Left.target_speed = 0;
              Wheel_Right.target_speed = 0;
              break;
      }
		
#endif
		
#if 1
		// 1. 解析串口指令
      /*if (rx_flag == 1)
      {
          rx_flag = 0;
          float target_L, target_R;
          // 使用 sscanf 解析 "L300,R300" 格式
          if (sscanf(rx_buffer, "L%f,R%f", &target_L, &target_R) == 2)
          {
              Wheel_Left.target_speed = target_L;
              Wheel_Right.target_speed = target_R;
          }
      }*/
			
			// ==================== 超声波避障检测 ====================
      static uint32_t last_sonar_time = 0;
      if (HAL_GetTick() - last_sonar_time >= 50) // 每 50ms 测一次
      {
          last_sonar_time = HAL_GetTick();
          current_distance = Get_Distance();
          
          // 如果前方 20cm 内有障碍物，且当前是在前进或原地不动
          /*if (current_distance > 0 && current_distance < 20.0f) 
          {
              // 强制刹车！覆盖树莓派发来的目标速度
              Wheel_Left.target_speed = 0;
              Wheel_Right.target_speed = 0;
              
              // 可选：如果被挡住，可以强制后退一下
              // Wheel_Left.target_speed = -100;
              // Wheel_Right.target_speed = -100;
          }*/
      }
      // ========================================================

      // 2. 每 200ms 往电脑/树莓派发送一次当前速度反馈 (方便你用串口助手调试)
      if (HAL_GetTick() - print_time >= 200)
      {

          print_time = HAL_GetTick();
          char tx_buf[100];
				
				  // 打印格式: 实际L, 目标L, 实际R, 目标R, 距离
					sprintf(tx_buf, "AL:%.1f, AR:%.1f, Dist:%.1f cm\r\n", 
                  Wheel_Left.actual_speed, 
                  Wheel_Right.actual_speed, 
                  current_distance); // <-- 新增了距离打印
                  
          HAL_UART_Transmit(&huart1, (uint8_t*)tx_buf, strlen(tx_buf), 100);
				                 
      }
			
			 // ==================== 新增：每 20ms 给树莓派回传状态 ====================
			if (HAL_GetTick() - uart2_tx_time >= 20) 
			{
#if 0
				  uart2_tx_time = HAL_GetTick();
					
					// 构造超级反馈帧 (总长 25 字节)
					uint8_t tx_frame[25];
					tx_frame[0] = 0xAA;
					tx_frame[1] = 0x55;
					tx_frame[2] = 21;   // 长度: Cmd(1) + 速度(4) + IMU(12) + 校验(1) = 21
					tx_frame[3] = 0x83; // 新的超级功能码
					
					// 1. 速度 (4字节)
					int16_t left_mm_s  = (int16_t)Wheel_Left.actual_speed;
					int16_t right_mm_s = (int16_t)Wheel_Right.actual_speed;
					tx_frame[4] = left_mm_s & 0xFF;   tx_frame[5] = (left_mm_s >> 8) & 0xFF;
					tx_frame[6] = right_mm_s & 0xFF;  tx_frame[7] = (right_mm_s >> 8) & 0xFF;
					
					// 2. 姿态 (6字节)
					int16_t yaw_100 = (int16_t)(Yaw * 100.0f);
					int16_t pitch_100 = (int16_t)(Pitch * 100.0f);
					int16_t roll_100 = (int16_t)(Roll * 100.0f); 
					tx_frame[8] = yaw_100 & 0xFF;   tx_frame[9] = (yaw_100 >> 8) & 0xFF;
					tx_frame[10] = pitch_100 & 0xFF; tx_frame[11] = (pitch_100 >> 8) & 0xFF;
					tx_frame[12] = roll_100 & 0xFF;  tx_frame[13] = (roll_100 >> 8) & 0xFF;
					
					// 3. 角速度 Z (2字节)
					int16_t gz = gyro[2];
					tx_frame[14] = gz & 0xFF;  tx_frame[15] = (gz >> 8) & 0xFF;

					// 4. 加速度 X, Y (4字节)
					int16_t ax = accel[0];
					int16_t ay = accel[1];
					tx_frame[16] = ax & 0xFF;  tx_frame[17] = (ax >> 8) & 0xFF;
					tx_frame[18] = ay & 0xFF;  tx_frame[19] = (ay >> 8) & 0xFF;
					
					// 5. 校验和 (长度+cmd+数据 = 21字节)
					uint8_t sum = 0;
					for(int i = 2; i < 24; i++) {
							sum += tx_frame[i];
					}
					tx_frame[24] = sum;
					
					// 一次性发出去！
					HAL_UART_Transmit(&huart2, tx_frame, 25, 20);
#endif

#if 1
					uart2_tx_time = HAL_GetTick();
					
					// 构造反馈帧
					uint8_t tx_frame[9];
					tx_frame[0] = 0xAA;
					tx_frame[1] = 0x55;
					tx_frame[2] = 5;    // 长度
					tx_frame[3] = 0x81; // 功能码：状态反馈					
					// 将实际速度 float 转为 int16_t (注意不要超出范围)
					int16_t left_mm_s  = (int16_t)Wheel_Left.actual_speed;
					int16_t right_mm_s = (int16_t)Wheel_Right.actual_speed;				
					tx_frame[4] = left_mm_s & 0xFF;
					tx_frame[5] = (left_mm_s >> 8) & 0xFF;
					tx_frame[6] = right_mm_s & 0xFF;
					tx_frame[7] = (right_mm_s >> 8) & 0xFF;				
					// 计算校验和
					tx_frame[8] = calc_checksum(&tx_frame[2], 6);				
					// 发送给树莓派
					HAL_UART_Transmit(&huart2, tx_frame, 9, 10);
				
				  // ==================== 第二帧：发送姿态 (功能码 0x82) ====================
          // 为了防止两帧连发导致树莓派解析错位，中间稍微延时 2ms
          //HAL_Delay(5); 
					
					
					uint8_t tx_imu[17];   
          tx_imu[0] = 0xAA;
          tx_imu[1] = 0x55;
          tx_imu[2] = 13;       
          tx_imu[3] = 0x82;     
          
          int16_t yaw_100 = (int16_t)(Yaw * 100.0f);
          int16_t pitch_100 = (int16_t)(Pitch * 100.0f);
          int16_t roll_100 = (int16_t)(Roll * 100.0f); 
          
          tx_imu[4] = yaw_100 & 0xFF;
          tx_imu[5] = (yaw_100 >> 8) & 0xFF;
          tx_imu[6] = pitch_100 & 0xFF;
          tx_imu[7] = (pitch_100 >> 8) & 0xFF;
          tx_imu[8] = roll_100 & 0xFF;                 
          tx_imu[9] = (roll_100 >> 8) & 0xFF;          

          int16_t gyro_z = gyro[2]; 
          tx_imu[10] = gyro_z & 0xFF;
          tx_imu[11] = (gyro_z >> 8) & 0xFF;

          int16_t accel_x = accel[0];
          int16_t accel_y = accel[1];
          tx_imu[12] = accel_x & 0xFF;
          tx_imu[13] = (accel_x >> 8) & 0xFF;
          tx_imu[14] = accel_y & 0xFF;
          tx_imu[15] = (accel_y >> 8) & 0xFF;
          
          tx_imu[16] = calc_checksum(&tx_imu[2], 13);   
          
          HAL_UART_Transmit(&huart2, tx_imu, 17, 100); 
					
					// ==================== 第三帧：发送超声波距离 (功能码 0x84) ====================
          //HAL_Delay(5); // 给树莓派留点处理时间
          
          uint8_t tx_sonar[9];
          tx_sonar[0] = 0xAA;
          tx_sonar[1] = 0x55;
          tx_sonar[2] = 5;    // 长度: Cmd(1) + 数据(2) + 保留(2)
          tx_sonar[3] = 0x84;  // 功能码
          
          uint16_t dist_cm = (uint16_t)current_distance; // 转成整数厘米
          tx_sonar[4] = dist_cm & 0xFF;
          tx_sonar[5] = (dist_cm >> 8) & 0xFF;
          tx_sonar[6] = 0x00;
          tx_sonar[7] = 0x00;
          
          tx_sonar[8] = calc_checksum(&tx_sonar[2], 6);
          
          HAL_UART_Transmit(&huart2, tx_sonar, 9, 100);
#endif
			}

#else
// 1. 接收串口指令
      if (rx_flag == 1)
      {
          rx_flag = 0;
          
          // 发送 '1' 正转 20% 速度
          if (rx_buffer[0] == '1') {
						
					
              // 左轮正转，固定给 20 的占空比
              HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, GPIO_PIN_SET);   
              HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_RESET); 
              __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, 20); // 固定 PWM=20
              
              // 右轮正转，固定给 20 的占空比
              HAL_GPIO_WritePin(GPIOF, GPIO_PIN_2, GPIO_PIN_SET);   
              HAL_GPIO_WritePin(GPIOF, GPIO_PIN_3, GPIO_PIN_RESET); 
              __HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, 20);
          }
          // 发送 '2' 反转 20% 速度
          else if (rx_buffer[0] == '2') {
              // 左轮反转
              HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, GPIO_PIN_RESET);   
              HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_SET); 
              __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, 20);
              
              // 右轮反转
              HAL_GPIO_WritePin(GPIOF, GPIO_PIN_2, GPIO_PIN_RESET);   
              HAL_GPIO_WritePin(GPIOF, GPIO_PIN_3, GPIO_PIN_SET); 
              __HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, 20); 
          }
          // 发送 '0' 停止
          else if (rx_buffer[0] == '0') {
              __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, 0);
              __HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, 0);
          }
      }

      // 2. 每 200ms 打印一次速度
      if (HAL_GetTick() - print_time >= 200)
      {
          print_time = HAL_GetTick();
          char tx_buf[60];
          sprintf(tx_buf, "AL:%.1f, AR:%.1f\r\n", Wheel_Left.actual_speed, Wheel_Right.actual_speed);
          HAL_UART_Transmit(&huart1, (uint8_t*)tx_buf, strlen(tx_buf), 100);
      }
#endif		

			
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
// --- 1. 定时器中断回调：处理 PID 和 电机输出 ---
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
#if 1
    if (htim->Instance == TIM6) // 10ms 心跳
    {
        // 计算左轮PID
        Calc_Wheel_PID(&Wheel_Left);
			
			  int32_t left_pwm = (int32_t)Wheel_Left.pid_out;
        /*if (Wheel_Left.target_speed > 0.1f || Wheel_Left.target_speed < -0.1f) {
            // 目标不是停止状态
            if (left_pwm > 0 && left_pwm < 15) left_pwm = 10;   // 正转死区补偿
            if (left_pwm < 0 && left_pwm > -15) left_pwm = -10; // 反转死区补偿
        }*/
				
        // 输出左轮
        if (left_pwm >= 0) {
            HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, GPIO_PIN_SET);   // AIN1
            HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_RESET); // AIN2
            __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, (uint32_t)left_pwm);
        } else {
            HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, GPIO_PIN_RESET); 
            HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_SET);   
            __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, (uint32_t)(-left_pwm));
        }

        // 计算右轮PID
        Calc_Wheel_PID(&Wheel_Right);
				
				int32_t right_pwm = (int32_t)Wheel_Right.pid_out;
        /*if (Wheel_Right.target_speed > 0.1f || Wheel_Right.target_speed < -0.1f) {
            if (right_pwm > 0 && right_pwm < 15) right_pwm = 10;
            if (right_pwm < 0 && right_pwm > -15) right_pwm = -10;
        }*/
        // 输出右轮
        if (right_pwm >= 0) {
            HAL_GPIO_WritePin(GPIOF, GPIO_PIN_2, GPIO_PIN_SET);   // BIN1
            HAL_GPIO_WritePin(GPIOF, GPIO_PIN_3, GPIO_PIN_RESET); // BIN2
            __HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, (uint32_t)right_pwm);
        } else {
            HAL_GPIO_WritePin(GPIOF, GPIO_PIN_2, GPIO_PIN_RESET); 
            HAL_GPIO_WritePin(GPIOF, GPIO_PIN_3, GPIO_PIN_SET);   
            __HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, (uint32_t)(-right_pwm));
        }
    }

#else
    if (htim->Instance == TIM6) // 10ms 心跳
    {
        // ====== 只测速，不计算 PID ======
        
        // 1. 读取左轮速度
        int16_t cur_L = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
        int16_t pulses_L = cur_L - Wheel_Left.last_count;
        Wheel_Left.last_count = cur_L;
        Wheel_Left.actual_speed = (float)pulses_L * SPEED_COEFFICIENT;

        // 2. 读取右轮速度
        int16_t cur_R = (int16_t)__HAL_TIM_GET_COUNTER(&htim4);
        int16_t pulses_R = cur_R - Wheel_Right.last_count;
        Wheel_Right.last_count = cur_R;
        Wheel_Right.actual_speed = (float)pulses_R * SPEED_COEFFICIENT;

        // 注意：这里把 Calc_Wheel_PID 和输出 PWM 的代码全部删掉了！
    }
#endif
}

// --- 2. 串口接收中断回调：解析树莓派指令 ---
// 假设树莓派发送格式为 "L300,R300\n" (L左轮速度, R右轮速度)
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) 
    {
        if (rx_data == '\n' || rx_data == '\r') // 收到换行符，说明一条指令结束
        {
            rx_buffer[rx_index] = '\0'; // 字符串结尾
            rx_index = 0;
            rx_flag = 1; // 标记接收完成，让主循环去解析
        } 
        else if (rx_index < 49) // 防止数组越界
        {
            rx_buffer[rx_index++] = rx_data;
        }
        
        // 重新开启接收中断，等待下一个字节
        HAL_UART_Receive_IT(&huart1, &rx_data, 1);
    }
		// 判断是不是 USART2 触发的
    if (huart->Instance == USART2)
    {
			  // 【防线1】：如果当前不是找帧头状态，且距离上一次收到数据超过 50ms，说明断包了，强行复位！
        if (uart2_state != WAIT_HEADER1 && (HAL_GetTick() - last_rx_tick > 50)) {
            uart2_state = WAIT_HEADER1;
        }
        last_rx_tick = HAL_GetTick(); // 记录最新接收时间
				
        // 1. 把收到的数据原样发回去 (Echo)
        //HAL_UART_Transmit(&huart2, &rx_data2, 1, 10);
			switch (uart2_state) 
        {
            case WAIT_HEADER1:
                if (rx_data2 == 0xAA) uart2_state = WAIT_HEADER2;
                break;
                
            case WAIT_HEADER2:
                if (rx_data2 == 0x55) uart2_state = WAIT_LEN;
                else uart2_state = WAIT_HEADER1;
                break;
                
            case WAIT_LEN:
                uart2_rx_buf[2] = rx_data2; // 记录长度
                uart2_rx_index = 0;
                uart2_state = WAIT_PAYLOAD;
                break;
                
            case WAIT_PAYLOAD:
                uart2_rx_buf[3 + uart2_rx_index] = rx_data2;
                uart2_rx_index++;
                if (uart2_rx_index >= uart2_rx_buf[2]) {
                    uart2_state = WAIT_CHECKSUM;
                }
                break;
                
            case WAIT_CHECKSUM:
                {
                    // 校验：长度 + 功能码 + 数据
                    uint8_t calc = calc_checksum(&uart2_rx_buf[2], uart2_rx_buf[2]+1);
                    if (calc == rx_data2) 
                    {
                        // 校验通过！判断功能码
                        if (uart2_rx_buf[3] == 0x01) // 速度设置指令
                        {
													  HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_9); 
                            // 提取 int16_t 速度 (小端模式)
                            int16_t left_mm_s  = (uart2_rx_buf[5] << 8) | uart2_rx_buf[4];
                            int16_t right_mm_s = (uart2_rx_buf[7] << 8) | uart2_rx_buf[6];
                            
                            // 更新 PID 目标值
                            Wheel_Left.target_speed = (float)left_mm_s;
                            Wheel_Right.target_speed = (float)right_mm_s;
                        }
                    }
                    uart2_state = WAIT_HEADER1; // 重新等待下一帧
                }
                break;
        }
        
        
        // 2. 重新开启中断，准备接收下一个字节
        HAL_UART_Receive_IT(&huart2, &rx_data2, 1);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        // 清除所有错误标志（特别是溢出错误 ORE）
        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        
        // 重置状态机
        uart2_state = WAIT_HEADER1;
        
        // 强行重新开启接收中断
        HAL_UART_Receive_IT(&huart2, &rx_data2, 1);
    }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
