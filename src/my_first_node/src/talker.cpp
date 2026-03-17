#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"

using namespace std::chrono_literals;

class MyPublisher : public rclcpp::Node {
public:
  MyPublisher() : Node("student_node_v1") {
    this->declare_parameter<double>("min_voltage", 32.0);
    this->declare_parameter<double>("max_voltage", 42.0);
    publisher_ = this->create_publisher<std_msgs::msg::String>("node_name", 10);
    percentage_pub = this->create_publisher<std_msgs::msg::Float32>("battery_percentage", 10);
    battery_sub = this->create_subscription<std_msgs::msg::Float32>("battery_voltage", 10, std::bind(&MyPublisher::battery_callback, this, std::placeholders::_1));
    timer_ = this->create_wall_timer(500ms, std::bind(&MyPublisher::timer_callback, this));
  }

private:
  void timer_callback() {
    auto message = std_msgs::msg::String();
    message.data = "student_node_v1"; 
    publisher_->publish(message);
  }

  void battery_callback(const std_msgs::msg::Float32::SharedPtr msg) {
    double voltage = msg->data;
    double min_v = this->get_parameter("min_voltage").as_double();
    double max_v = this->get_parameter("max_voltage").as_double();
    double percentage = (voltage - min_v) / (max_v - min_v) * 100.0;

    auto out_msg = std_msgs::msg::Float32();
    out_msg.data = percentage;
    percentage_pub->publish(out_msg);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr percentage_pub;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr battery_sub;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MyPublisher>());
  rclcpp::shutdown();
  return 0;
}
