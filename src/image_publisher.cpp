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
#include <cv_bridge/cv_bridge.h>
#include <rclcpp/type_adapter.hpp>

#include "misora2_image_publisher/cv_mat_type_adapter.hpp"

namespace fs = std::filesystem;
using namespace std::chrono_literals;
using std::placeholders::_1;

class ImagePublisher : public rclcpp::Node
{
    public:
    using MyAdaptedType = rclcpp::TypeAdapter<cv::Mat, sensor_msgs::msg::Image>;
    std::string path,topic,format;

    ImagePublisher() : Node("image_publisher")
    {
        this->declare_parameter("path","src/misora2_image_publisher/photo1.png");
        this->declare_parameter("topic","/image_raw");
        this->declare_parameter("format","color");

        path = this->get_parameter("path").as_string();
        topic = this->get_parameter("topic").as_string();
        format = this->get_parameter("format").as_string();

        publisher_ = this->create_publisher<MyAdaptedType>(topic,1);
        // publisher_ = this->create_publisher<sensor_msgs::msg::Image>(topic,10);
        RCLCPP_INFO(this->get_logger(), "Image publisher node has benn started. ");
        
        timer_ = this->create_wall_timer(500ms, std::bind(&ImagePublisher::timer_callback, this));
        
    }
    private:
    void bring_image(std::string p,  std::string f){
        fs::path path = p;
        cv::Mat image;

        if(fs::exists(path) == false){
            throw std::runtime_error("Error: The file does not exist: " + path.string());
        }
        if(f == "color"){
            image = cv::imread(path.string(), cv::IMREAD_COLOR); // 色画像として読み込む
        }
        else if(f == "mono"){
            image = cv::imread(path.string(), cv::IMREAD_GRAYSCALE); // 色画像として読み込む
        }
        else throw std::runtime_error("Error: The format does not exist: " + f);

        image = cv::imread(path.string(), (f == "color") ? 1 : 0); // 色画像またはモノクロ画像として読み込む

        if (image.empty()) {
            throw std::runtime_error("Error: Unable to load image: " + path.string());
        }
        cv::resize(image, image, cv::Size(640, 480));
        
        publisher_->publish(image);
    }

    void timer_callback(){
        bring_image(path,format);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<MyAdaptedType>::SharedPtr publisher_;
    // rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
};

int main(int argc, char* argv[]){
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<ImagePublisher>());//ここがさっきの引数と連携している
    rclcpp::shutdown();
    return 0;
}