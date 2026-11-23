#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class RobotDance(Node):
    def __init__(self):
        super().__init__('robot_dance')
        # 注意：这里的发布话题要和你底盘实际订阅的一致
        self.publisher_ = self.create_publisher(Twist, '/diff_drive_controller/cmd_vel_unstamped', 10)
        self.get_logger().info("🕺 舞蹈即将开始，请确保场地宽敞！")

    def play_move(self, duration, linear_x, angular_z, description=""):
        self.get_logger().info(f"▶ 动作: {description}")
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        
        # 按照 20Hz 的频率持续发送指令
        start_time = time.time()
        while time.time() - start_time < duration:
            self.publisher_.publish(msg)
            time.sleep(0.05)
            
        # 动作结束，刹车
        stop_msg = Twist()
        self.publisher_.publish(stop_msg)
        time.sleep(0.2) # 缓冲时间

    def dance(self):
        # ---- 舞步编排开始 ----
        
        # 1. 开场：原地快速转圈（托马斯回旋）
        self.play_move(2.0, 0.0, 0.8, "原地极速旋转")
        
        # 2. 太空步：慢慢后退的同时自转
        self.play_move(1.5, -0.05, 0.4, "后退回旋")
        
        # 3. 摇尾舞：左右扭动（画S型）
        for _ in range(3):
            self.play_move(0.4, 0.08, 0.6, "向右扭")
            self.play_move(0.4, 0.08, -0.6, "向左扭")
            
        # 4. 突进与急刹：表现出机械感
        self.play_move(0.3, 0.15, 0.0, "向前突进")
        time.sleep(0.5) # 停顿装死
        self.play_move(0.3, -0.15, 0.0, "向后暴退")
        
        # 5. 华尔兹：画圆圈
        self.play_move(3.0, 0.08, 0.3, "画个大圆")
        
        # 6. 结束动作：高傲地转半圈然后停下
        self.play_move(1.0, 0.0, -0.8, "反向回旋")
        
        self.get_logger().info("🎉 舞蹈结束！")
        self.publish_stop()

    def publish_stop(self):
        stop_msg = Twist()
        self.publisher_.publish(stop_msg)

def main(args=None):
    rclpy.init(args=args)
    dancer = RobotDance()
    
    try:
        dancer.dance()
    except KeyboardInterrupt:
        pass
    finally:
        dancer.publish_stop()
        dancer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()