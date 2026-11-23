#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <thread>
#include <mutex>
#include <atomic>
#include <vector>
#include <iostream>

class CameraPublisher : public rclcpp::Node
{
public:
    CameraPublisher() : Node("camera_publisher"), running_(true)
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/image_raw", 10);
        
        // 启动 rpicam-vid 进程，输出 MJPEG 流
        std::string cmd = "rpicam-vid -t 0 --codec mjpeg --width 320 --height 240 -n -o -";
        RCLCPP_INFO(this->get_logger(), "启动摄像头命令: %s", cmd.c_str());
        pipe_ = popen(cmd.c_str(), "r");
        
        if (!pipe_) {
            RCLCPP_ERROR(this->get_logger(), "无法启动 rpicam-vid 进程！");
            return;
        }

        // 开启后台线程专门读取管道
        reader_thread_ = std::thread(&CameraPublisher::reader_loop, this);
        
        // 开启定时器 30Hz 发布最新帧
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33),
            std::bind(&CameraPublisher::timer_callback, this));
            
        RCLCPP_INFO(this->get_logger(), "CSI 摄像头节点已启动，正在发布 /image_raw ...");
    }

    ~CameraPublisher()
    {
        running_ = false;
        if (reader_thread_.joinable()) {
            reader_thread_.join();
        }
        if (pipe_) {
            pclose(pipe_);
        }
    }

private:
    void reader_loop()
    {
        // MJPEG 流由连续的 JPEG 图像组成
        // JPEG 起始标记是 0xFF 0xD8，结束标记是 0xFF 0xD9
        std::vector<uchar> jpeg_buffer;
        bool in_frame = false;
        int b;
        
        while (running_ && (b = fgetc(pipe_)) != EOF)
        {
            if (!in_frame) {
                // 寻找帧头
                if (b == 0xFF) {
                    int b2 = fgetc(pipe_);
                    if (b2 == 0xD8) {
                        in_frame = true;
                        jpeg_buffer.clear();
                        jpeg_buffer.push_back(0xFF);
                        jpeg_buffer.push_back(0xD8);
                    }
                }
            } else {
                jpeg_buffer.push_back((uchar)b);
                // 检查帧尾
                if (jpeg_buffer.size() >= 2 && 
                    jpeg_buffer[jpeg_buffer.size() - 2] == 0xFF && 
                    jpeg_buffer[jpeg_buffer.size() - 1] == 0xD9) 
                {
                    // 解码 JPEG
                    cv::Mat frame = cv::imdecode(jpeg_buffer, cv::IMREAD_COLOR);
                    if (!frame.empty()) {
                        std::lock_guard<std::mutex> lock(mutex_);
                        latest_frame_ = frame.clone();
                    }
                    in_frame = false;
                }
            }
        }
    }

    void timer_callback()
    {
        cv::Mat frame;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (latest_frame_.empty()) return;
            frame = latest_frame_.clone();
        }

        try {
            auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
            msg->header.stamp = this->get_clock()->now();
            msg->header.frame_id = "camera_link";
            publisher_->publish(*msg);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "图像发布失败: %s", e.what());
        }
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    FILE* pipe_ = nullptr;
    
    std::thread reader_thread_;
    std::atomic<bool> running_;
    std::mutex mutex_;
    cv::Mat latest_frame_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CameraPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}