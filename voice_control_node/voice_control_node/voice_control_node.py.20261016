import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sounddevice as sd
import numpy as np
import json
import threading
import time
from vosk import Model, KaldiRecognizer



class VoiceControlNode(Node):
    def __init__(self):
        super().__init__('voice_control_node')

        # 发布 /cmd_vel
        # self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.cmd_pub = self.create_publisher(Twist, '/diff_drive_controller/cmd_vel_unstamped', 10)

        # 加载 Vosk 中文模型
        model_path = "/home/liang/vosk-model-small-cn-0.22"
        self.get_logger().info(f"Loading Vosk model from {model_path}...")
        self.model = Model(model_path)
        self.rec = KaldiRecognizer(self.model, 16000)
        self.get_logger().info("Vosk model loaded.")

        # 指令映射
        self.command_map = {
            "前进": (0.1, 0.0),
            "向前": (0.1, 0.0),
            "后退": (-0.1, 0.0),
            "向后": (-0.1, 0.0),
            "左转": (0.0, 0.3),
            "右转": (0.0, -0.3),
            "停": (0.0, 0.0),
            "停止": (0.0, 0.0),
            "你好": (0.1, 0.0),
        }

        # 录音参数
        self.sample_rate = 16000
        self.record_duration = 3.0
        self.mic_device = None  # 或填设备索引

        # 运行标志
        self.running = True

        # 启动独立线程做录音识别
        self.thread = threading.Thread(target=self.voice_loop, daemon=True)
        self.thread.start()

        self.get_logger().info("语音控制节点已启动啦，请说话...")

    def voice_loop(self):
        """独立线程：循环录音 + 识别"""
        # 显式指定 USB 麦克风设备
        self.mic_device = None   # ← 改成你的设备索引

        self.get_logger().info(f"Using audio device: {self.mic_device}")

        while self.running and rclpy.ok():
            try:
                audio = sd.rec(
                    int(self.record_duration * self.sample_rate),
                    samplerate=self.sample_rate,
                    channels=1,
                    dtype='int16',
                    device=self.mic_device
                )
                sd.wait()
                audio_bytes = audio.tobytes()

                # 检查录音是否有声音
                max_val = np.max(np.abs(audio))
                if max_val < 100:
                    self.get_logger().warn(f"录音音量太低: max={max_val}，可能没录到声音")
                    continue

                if self.rec.AcceptWaveform(audio_bytes):
                    result = json.loads(self.rec.Result())
                    text = result.get('text', '').replace(' ', '')
                    if text:
                        self.get_logger().info(f"识别到: {text}")
                        self.process_command(text)

            except Exception as e:
                self.get_logger().error(f"录音/识别错误: {e}")
                time.sleep(0.5)

    def process_command(self, text):
        """匹配指令并发布"""
        for keyword, (linear_x, angular_z) in self.command_map.items():
            if keyword in text:
                self.get_logger().info(f"执行: {keyword}")
                self.publish_cmd(linear_x, angular_z)
                return
        self.get_logger().info(f"未匹配到指令: {text}")

    def publish_cmd(self, linear_x, angular_z):
        """发布 Twist 消息"""
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        # 持续发布 1 秒，确保控制器收到
        for _ in range(10):
            self.cmd_pub.publish(msg)
            time.sleep(0.1)

    def destroy_node(self):
        self.running = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VoiceControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()