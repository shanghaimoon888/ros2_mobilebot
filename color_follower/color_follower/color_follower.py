#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

class ColorFollowerNode(Node):
    def __init__(self):
        super().__init__('color_follower_node')
        
        # 1. 订阅图像
        self.subscription = self.create_subscription(
            Image, '/image_raw', self.image_callback, 10)
        # 2. 发布速度指令 (注意根据你的控制器订阅修改)
        self.publisher_ = self.create_publisher(
            Twist, '/diff_drive_controller/cmd_vel_unstamped', 10)
        
        self.bridge = CvBridge()
        
        # ==================== 核心参数（佛系极慢版） ====================
        self.screen_center_x = 320     # 画面中心点 (640/2)
        
        # 速度限制
        self.max_linear = 0.02         # 最大线速度 (2cm/s)
        self.max_angular = 0.15        # 最大角速度
        
        # PD 控制参数
        self.kp = 0.0008               # 比例系数 (你调好的极低系数)
        self.kd = 0.003                # 微分系数 (防止极慢速下的画龙)
        
        # 记录上一帧的误差，用于计算微分
        self.prev_error_x = 0.0
        
        # 颜色阈值（你测试成功的绿色）
        self.lower_green = np.array([40, 80, 50])
        self.upper_green = np.array([80, 255, 255])
        # ===============================================================

        self.get_logger().info("智能视觉跟随节点已启动！(PD控制+动态死区)")

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'图像转换失败: {e}')
            return

        # 1. 图像处理，寻找目标
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_green, self.upper_green)
        
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        cmd = Twist()
        target_found = False
        dynamic_deadzone = 0 # 用于画图

        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            
            # 过滤噪点
            if cv2.contourArea(largest_contour) > 1000:
                target_found = True
                x, y, w, h = cv2.boundingRect(largest_contour)
                center_x = x + w // 2
                center_y = y + h // 2
                
                # 计算误差（目标在左边，error_x 为正；在右边为负）
                error_x = self.screen_center_x - center_x
                
                # ==================== 核心算法 1：动态死区 ====================
                # 目标越近（w越大），死区越宽，防止在跟前乱抖
                dynamic_deadzone = 50 + (w * 0.15) 

                # ==================== 核心算法 2：PD 控制转向 ====================
                if abs(error_x) > dynamic_deadzone:
                    # 计算误差的变化率
                    error_rate = error_x - self.prev_error_x
                    
                    # PD 公式
                    angular_z = (error_x * self.kp) + (error_rate * self.kd)
                    
                    # 限幅
                    cmd.angular.z = max(min(angular_z, self.max_angular), -self.max_angular)
                else:
                    # 在死区内，走直线
                    cmd.angular.z = 0.0
                    
                self.prev_error_x = error_x

                # ==================== 核心算法 3：距离控制 ====================
                if w > 280: 
                    # 太近了，停车（防止撞上）
                    cmd.linear.x = 0.0
                    cmd.angular.z = 0.0
                elif w > 180: 
                    # 距离适中，慢速跟随 (1cm/s)
                    cmd.linear.x = self.max_linear * 0.5
                else: 
                    # 距离较远，正常速度跟上 (2cm/s)
                    cmd.linear.x = self.max_linear
                
                # ==================== 核心算法 4：转弯降速 ====================
                # 如果在转弯，强制限制线速度，防止惯性甩尾
                if abs(cmd.angular.z) > 0.01:
                    cmd.linear.x = min(cmd.linear.x, 0.01) 

                # 调试画图
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)
                
                # 画出当前动态死区的范围（黄色线），方便你观察
                cv2.line(frame, (self.screen_center_x - int(dynamic_deadzone), 0), 
                         (self.screen_center_x - int(dynamic_deadzone), 480), (0, 255, 255), 1)
                cv2.line(frame, (self.screen_center_x + int(dynamic_deadzone), 0), 
                         (self.screen_center_x + int(dynamic_deadzone), 480), (0, 255, 255), 1)

        # 没找到目标，停车并清零历史误差
        if not target_found:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.prev_error_x = 0.0

        # 发布指令
        self.publisher_.publish(cmd)

        # ==================== 显示画面 ====================
        # 左上角显示当前状态文字
        status_text = f"L: {cmd.linear.x:.3f}  A: {cmd.angular.z:.3f}"
        cv2.putText(frame, status_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        
        cv2.imshow("Color Follower Debug", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ColorFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        stop_cmd = Twist()
        node.publisher_.publish(stop_cmd)
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()