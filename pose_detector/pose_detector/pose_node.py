#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import cv2
import numpy as np
import mediapipe as mp
import time

class PoseDetectorNode(Node):
    def __init__(self):
        super().__init__('pose_detector_node')
        
        from cv_bridge import CvBridge
        self.bridge = CvBridge()
        
        # 订阅摄像头图像
        self.subscription = self.create_subscription(Image, '/image_raw', self.image_callback, 10)
        # 发布速度指令
        self.cmd_pub = self.create_publisher(Twist, '/diff_drive_controller/cmd_vel_unstamped', 10)
        
        # 初始化 MediaPipe
        self.mp_pose = mp.solutions.pose
        self.mp_drawing = mp.solutions.drawing_utils
        self.pose = self.mp_pose.Pose(
            static_image_mode=False, model_complexity=0, 
            enable_segmentation=False, min_detection_confidence=0.5)
        
        # ==================== 核心参数 (适配 320x240) ====================
        self.screen_center_x = 160     # 画面中心
        self.deadzone = 60             # 死区
        
        self.max_linear = 0.02         # 最大线速度 (极慢)
        self.max_angular = 0.1         # 最大角速度
        
        self.kp = 0.0015               
        self.kd = 0.004                
        self.filter_alpha = 0.3
        
        self.last_cx = None
        self.last_error = 0.0
        self.last_time = 0.0
        
        self.current_v = 0.0
        self.current_w = 0.0
        
        self.is_starting = True        
        self.start_time = 0.0          
        # ======================================================

        # ==================== 视频录制初始化 ====================
        self.video_path = "pose_record.mp4" # 保存在运行节点的当前目录
        self.video_fps = 12.0               # 根据你的实际处理帧率调整，防快进
        self.video_size = (320, 240)        # 必须和摄像头分辨率一致
        
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        self.video_writer = cv2.VideoWriter(self.video_path, fourcc, self.video_fps, self.video_size)
        self.get_logger().info(f"视频将保存至: {self.video_path}")
        # ======================================================
        
        self.get_logger().info("骨骼跟随节点已启动 (防猛窜+录制版)...")
        cv2.namedWindow("Pose Follower", cv2.WINDOW_AUTOSIZE)

    def image_callback(self, msg):
        current_time = time.time()
        if current_time - self.last_time < 0.066:
            return
        dt = current_time - self.last_time
        self.last_time = current_time

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            results = self.pose.process(frame_rgb)
            
            target_v = 0.0
            target_w = 0.0
            status = "LOST"
            
            if results.pose_landmarks:
                if self.last_cx is None:
                    self.is_starting = True
                    self.start_time = current_time
                    self.current_v = 0.0 
                
                self.mp_drawing.draw_landmarks(
                    frame, results.pose_landmarks, self.mp_pose.POSE_CONNECTIONS,
                    landmark_drawing_spec=self.mp_drawing.DrawingSpec(color=(0, 255, 0), thickness=2, circle_radius=2),
                    connection_drawing_spec=self.mp_drawing.DrawingSpec(color=(255, 0, 0), thickness=2))
                
                h, w, _ = frame.shape
                landmarks = results.pose_landmarks.landmark
                
                raw_cx = int((landmarks[11].x + landmarks[12].x + landmarks[23].x + landmarks[24].x) / 4 * w)
                raw_cy = int((landmarks[11].y + landmarks[12].y + landmarks[23].y + landmarks[24].y) / 4 * h)
                
                if self.last_cx is None:
                    self.last_cx = raw_cx
                else:
                    self.last_cx = int(self.filter_alpha * raw_cx + (1 - self.filter_alpha) * self.last_cx)
                
                cx = self.last_cx
                cv2.circle(frame, (cx, raw_cy), 8, (0, 0, 255), -1)
                
                error_x = self.screen_center_x - cx
                if abs(error_x) > self.deadzone:
                    effective_error = error_x - np.sign(error_x) * self.deadzone
                    error_rate = (error_x - self.last_error) / dt
                    angular_z = (effective_error * self.kp) + (error_rate * self.kd)
                    target_w = max(min(angular_z, self.max_angular), -self.max_angular)
                else:
                    target_w = 0.0
                
                self.last_error = error_x
                
                person_height = abs(landmarks[27].y - landmarks[0].y) 
                if person_height > 0.75:
                    target_v = 0.0
                    status = "STOP"
                elif person_height > 0.4:
                    target_v = self.max_linear * 0.5
                    status = "FOLLOW"
                else:
                    target_v = self.max_linear
                    status = "APPROACH"
                    
                turn_factor = abs(target_w) / self.max_angular
                target_v = target_v * (1.0 - 0.5 * turn_factor) 
                
                if self.is_starting:
                    elapsed = current_time - self.start_time
                    if elapsed < 1.5:
                        target_v *= 0.2
                        status = "SOFT START..."
                    else:
                        self.is_starting = False

                cv2.putText(frame, f"Action: {status}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                cv2.putText(frame, f"E: {int(error_x)}", (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                
            else:
                self.last_cx = None 
                self.last_error = 0.0
                cv2.putText(frame, "Action: LOST - STOP", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

            # 软件平滑
            max_v_step = 0.01
            max_w_step = 0.02
            
            if target_v > self.current_v + max_v_step: self.current_v += max_v_step
            elif target_v < self.current_v - max_v_step: self.current_v -= max_v_step
            else: self.current_v = target_v
            
            if target_w > self.current_w + max_w_step: self.current_w += max_w_step
            elif target_w < self.current_w - max_w_step: self.current_w -= max_w_step
            else: self.current_w = target_w

            cmd = Twist()
            cmd.linear.x = float(self.current_v)
            cmd.angular.z = float(self.current_w)
            self.cmd_pub.publish(cmd)
            
            # ==================== 写入视频文件 ====================
            if frame.shape[1] == self.video_size[0] and frame.shape[0] == self.video_size[1]:
                self.video_writer.write(frame)
            # ======================================================

            cv2.imshow("Pose Follower", frame)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f'处理失败: {repr(e)}', throttle_duration_sec=2.0)

    def destroy_node(self):
        # 退出时确保小车停下
        self.cmd_pub.publish(Twist())
        
        # 释放视频写入器
        if hasattr(self, 'video_writer') and self.video_writer is not None:
            self.video_writer.release()
            self.get_logger().info(f"✅ 视频已保存到当前目录: {self.video_path}")

        self.pose.close()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = PoseDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()