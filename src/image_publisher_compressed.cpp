#include <iostream>
#include <vector>
#include <string>
#include <memory>
#include <chrono>
#include <functional>
#include <filesystem>

#include <opencv2/opencv.hpp>
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <rclcpp/type_adapter.hpp>

#include "misora2_image_publisher/cv_mat_type_adapter.hpp"

namespace fs = std::filesystem;
using namespace std::chrono_literals;
using std::placeholders::_1;

class ImagePublisher : public rclcpp::Node
{
public:
    using MyAdaptedType = rclcpp::TypeAdapter<cv::Mat, sensor_msgs::msg::Image>;
    std::string path, topic, format;

    ImagePublisher() : Node("image_publisher")
    {
        this->declare_parameter("path", "src/misora2_image_publisher/photo1.png");
        this->declare_parameter("topic", "/image_raw");
        this->declare_parameter("format", "color");  // color, mono, black, compressed

        path = this->get_parameter("path").as_string();
        topic = this->get_parameter("topic").as_string();
        format = this->get_parameter("format").as_string();

        if (format == "compressed") {
            compressed_publisher_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(topic, 1);
            RCLCPP_INFO(this->get_logger(), "Compressed Image publisher started.");
        } else {
            publisher_ = this->create_publisher<MyAdaptedType>(topic, 1);
            RCLCPP_INFO(this->get_logger(), "Raw Image publisher started.");
        }

        timer_ = this->create_wall_timer(500ms, std::bind(&ImagePublisher::timer_callback, this));
    }

private:
    void bring_image(std::string p, std::string f)
    {
        fs::path path = p;
        cv::Mat image;

        if (fs::exists(path) == false) {
            throw std::runtime_error("Error: The file does not exist: " + path.string());
        }

        if (f == "color" || f == "compressed" || f == "yuv422_yuy2") {
            cv::Mat rgb_image = cv::imread(path.string(), cv::IMREAD_COLOR);
            if (rgb_image.empty()) {
                throw std::runtime_error("Error: Unable to load image: " + path.string());
            }
            cv::resize(rgb_image, rgb_image, cv::Size(640, 480));

            if (f == "color" || f == "compressed") {
                // RGBをROS標準フォーマットで送信
                cv::cvtColor(rgb_image, image, cv::COLOR_BGR2RGB);
            }
            // else if (f == "yuv422_yuy2") {
            //     // RGB -> YUV422 YUY2変換
            //     cv::cvtColor(rgb_image, image, cv::COLOR_BGR2YUV_YUYV);
            // }
        }
        else if (f == "mono") {
            image = cv::imread(path.string(), cv::IMREAD_GRAYSCALE);
        }
        else if (f == "black") {
            image = cv::Mat::zeros(480, 640, CV_8UC1);
            RCLCPP_INFO_STREAM(this->get_logger(), "Created black image channels: " << image.channels());
        }
        else {
            throw std::runtime_error("Error: The format does not exist: " + f);
        }

        if (image.empty()) {
            throw std::runtime_error("Error: Unable to load image: " + path.string());
        }

        cv::resize(image, image, cv::Size(640, 480));

        if (f == "compressed") {
            // JPEG圧縮してCompressedImageメッセージを作成
            std::vector<uchar> buf;
            cv::imencode(".jpg", image, buf);  // JPEG圧縮

            auto msg = sensor_msgs::msg::CompressedImage();
            msg.format = "jpeg";
            msg.data.assign(buf.begin(), buf.end());

            RCLCPP_INFO_STREAM(this->get_logger(), "Send: compressed image size = " << msg.data.size());
            compressed_publisher_->publish(msg);
        }
        // else if (f == "yuv422_yuy2") {
        //     // sensor_msgs::msg::Imageとして送信
        //     // encodingフィールドは"yuv422_yuy2"
        //     auto msg = sensor_msgs::msg::Image();
        //     msg.header.stamp = this->now();
        //     msg.height = image.rows;
        //     msg.width = image.cols;
        //     msg.encoding = "yuv422_yuy2";
        //     msg.is_bigendian = false;
        //     msg.step = static_cast<uint32_t>(image.cols * 2); // 2 bytes per pixel in YUY2
        //     msg.data.assign(image.datastart, image.dataend);

        //     RCLCPP_INFO_STREAM(this->get_logger(), "Send: yuv422_yuy2 image size = " << msg.data.size());
        //     publisher_->publish(msg);
        // }
        else {
            RCLCPP_INFO_STREAM(this->get_logger(), "Send: image channels = " << image.channels());
            publisher_->publish(image);
        }
    }

    void timer_callback()
    {
        bring_image(path, format);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<MyAdaptedType>::SharedPtr publisher_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_publisher_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImagePublisher>());
    rclcpp::shutdown();
    return 0;
}
