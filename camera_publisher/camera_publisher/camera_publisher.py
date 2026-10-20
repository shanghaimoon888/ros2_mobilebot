#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import types
import pathlib

# ============================================================
# 1. 临时绕过 Picamera2 的 DRM 预览模块（你的成功补丁）
# ============================================================
PREVIEW_DIR = pathlib.Path(
    "/home/liang/.local/lib/python3.10/site-packages/picamera2/previews"
)
previews = types.ModuleType("picamera2.previews")
previews.__path__ = [str(PREVIEW_DIR)]
sys.modules["picamera2.previews"] = previews

from picamera2.previews.null_preview import NullPreview

drm = types.ModuleType("picamera2.previews.drm_preview")
drm.DrmPreview = NullPreview
drm.list_devices = lambda: []
sys.modules["picamera2.previews.drm_preview"] = drm

previews.DrmPreview = NullPreview
previews.NullPreview = NullPreview
previews.QtGlPreview = None
previews.QtGlPreviewDirect = None
previews.QtPreview = None

# ============================================================
# 2. ROS 2 节点代码
# ============================================================
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from picamera2 import Picamera2

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        
        # 创建图像发布者
        self.publisher_ = self.create_publisher(Image, '/image_raw', 10)
        self.bridge = CvBridge()
        
        # 初始化摄像头
        self.picam2 = Picamera2()
        config = self.picam2.create_preview_configuration(
            main={"size": (640, 480), "format": "RGB888"}
        )
        self.picam2.configure(config)
        self.picam2.start()
        
        # 设置定时器，30Hz 发布
        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)
        self.get_logger().info("CSI 摄像头节点已启动，正在发布 /image_raw ...")

    def timer_callback(self):
        try:
            frame_rgb = self.picam2.capture_array()
            # ROS 的 cv_bridge 一般使用 bgr8 格式
            frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
            
            msg = self.bridge.cv2_to_imgmsg(frame_bgr, encoding="bgr8")
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "camera_link"
            self.publisher_.publish(msg)
        except Exception as e:
            self.get_logger().error(f"图像抓取失败: {e}")

    def destroy_node(self):
        self.picam2.stop()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()