#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ColorTrackerNode(Node):
    def __init__(self):
        super().__init__('color_tracker_node')
        # 订阅图像发布节点
        self.subscription = self.create_subscription(
            Image, '/image_raw', self.image_callback, 10)
        self.bridge = CvBridge()
        
        cv2.namedWindow("Color Tracker Test", cv2.WINDOW_AUTOSIZE)
        self.get_logger().info("颜色识别节点已启动，等待目标...")

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'图像转换失败: {e}')
            return

        # 1. 转换到 HSV 颜色空间
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 2. 定义目标颜色的 HSV 阈值范围
        # 这里以“亮绿色”为例（你可以根据你的便利贴颜色修改）
        # 提示：如果选红色，需要写两段范围，因为红色在 HSV 两头
        lower_green = np.array([40, 100, 100])
        upper_green = np.array([80, 255, 255])

        # 生成掩膜：在这个范围内的变成白色(255)，其他变成黑色(0)
        mask = cv2.inRange(hsv, lower_green, upper_green)

        # 3. 图像去噪
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)   # 擦除小噪点
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)  # 填补目标内部的空洞

        # 4. 寻找轮廓
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            # 找最大的轮廓（假设最大的色块就是你的手表标签）
            largest_contour = max(contours, key=cv2.contourArea)
            
            # 面积过滤：太小说明是噪点，忽略
            if cv2.contourArea(largest_contour) > 800:
                # 获取外接矩形
                x, y, w, h = cv2.boundingRect(largest_contour)
                
                # 计算中心点
                center_x = x + w // 2
                center_y = y + h // 2
                
                # 画框和中心点
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)
                
                # 画十字准星（画面正中心 320, 240）
                cv2.line(frame, (320, 0), (320, 480), (255, 0, 0), 1)
                cv2.line(frame, (0, 240), (640, 240), (255, 0, 0), 1)

                # 打印坐标
                cv2.putText(frame, f"Target: ({center_x}, {center_y})", 
                            (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                
                self.get_logger().info(f'找到目标，中心点: ({center_x}, {center_y})')
            else:
                self.get_logger().info('检测到颜色，但目标太小', throttle_duration_sec=2.0)
        else:
            self.get_logger().info('未找到目标颜色', throttle_duration_sec=2.0)

        # 显示画面
        cv2.imshow("Color Tracker Test", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ColorTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()