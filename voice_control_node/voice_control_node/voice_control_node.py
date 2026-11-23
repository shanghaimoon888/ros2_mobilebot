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

        # 发布到 diff_drive_controller 实际订阅的话题
        self.cmd_pub = self.create_publisher(
            Twist, '/diff_drive_controller/cmd_vel_unstamped', 10)

        # 当前指令（由语音更新，定时器持续发布）
        self.current_cmd = (0.0, 0.0)
        self.cmd_lock = threading.Lock()
        self.last_cmd_time = time.time()

        # 加载 Vosk 模型
        model_path = "/home/liang/vosk-model-small-cn-0.22"
        self.get_logger().info(f"Loading Vosk model from {model_path}...")
        self.model = Model(model_path)
        grammar = '["前进", "向前", "后退", "向后", "左转", "右转", "停", "停止", "[unk]"]'
        self.rec = KaldiRecognizer(self.model, 16000, grammar)
        #self.rec = KaldiRecognizer(self.model, 16000)
        self.rec.SetWords(False)
        self.get_logger().info("Vosk model loaded.")

        # 指令映射（线速度, 角速度）
        self.command_map = {
            "前进": (0.2, 0.0),
            "向前": (0.2, 0.0),
            "后退": (-0.2, 0.0),
            "向后": (-0.2, 0.0),
            "左转": (0.0, 0.5),
            "右转": (0.0, -0.5),
            "停": (0.0, 0.0),
            "停止": (0.0, 0.0),
        }

        # reSpeaker Lite 设备索引 python3 -c "import sounddevice; print(sounddevice.query_devices())" 可查设备索引
        self.mic_device = 1
        self.sample_rate = 16000
        self.running = True

        # 音频流回调（在独立线程中运行）
        self.stream = None

        # 启动音频流
        self.start_audio_stream()

        # 启动持续发布定时器（0.1 秒一次）
        self.publish_timer = self.create_timer(0.1, self.publish_current_cmd)

        self.get_logger().info("语音控制节点已启动，请说话...")

    def start_audio_stream(self):
        """打开持续监听的音频流"""
        try:
            self.stream = sd.InputStream(
                samplerate=self.sample_rate,
                channels=1,
                dtype='int16',
                blocksize=1024,
                device=self.mic_device,
                callback=self.audio_callback
            )
            self.stream.start()
            self.get_logger().info(
                f"Audio stream started on device {self.mic_device}")
        except Exception as e:
            self.get_logger().error(f"Failed to open audio stream: {e}")

    def audio_callback(self, indata, frames, time_info, status):
        """每收到一小块音频就喂给 Vosk，边说边识别"""
        if status:
            self.get_logger().warn(f"Audio status: {status}")

        audio_bytes = indata.tobytes()

        if self.rec.AcceptWaveform(audio_bytes):
            # 识别出一个完整句子
            result = json.loads(self.rec.Result())
            text = result.get("text", "").replace(" ", "")
            if text:
                self.get_logger().info(f"识别到: {text}")
                self.process_command(text)
        else:
            # 边说边识别：显示临时结果
            partial = json.loads(self.rec.PartialResult())
            partial_text = partial.get("partial", "").replace(" ", "")
            if partial_text:
                self.get_logger().info(f"识别中... {partial_text}")

    def process_command(self, text):
        """匹配指令并更新当前指令"""
        for keyword, (linear_x, angular_z) in self.command_map.items():
            if keyword in text:
                self.get_logger().info(f"执行: {keyword}")
                with self.cmd_lock:
                    self.current_cmd = (linear_x, angular_z)
                    self.last_cmd_time = time.time()
                return
        self.get_logger().info(f"未匹配到指令: {text}")

    def publish_current_cmd(self):
        """每 0.1 秒发布一次当前指令，保持控制器活跃"""
        # 超过 5 秒没有新指令，自动停止（安全保护）
        if time.time() - self.last_cmd_time > 5.0:
            with self.cmd_lock:
                self.current_cmd = (0.0, 0.0)

        with self.cmd_lock:
            linear_x, angular_z = self.current_cmd

        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.cmd_pub.publish(msg)

    def destroy_node(self):
        self.running = False
        if self.stream is not None:
            self.stream.stop()
            self.stream.close()
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