#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class StringPublisher : public rclcpp::Node
{
public:
  StringPublisher()
  : Node("string_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("String", 10);
    message = std_msgs::msg::String();
    message.data = "Hello, world!";
    for(auto i = 0; i < 14; ++i) message.data += message.data;
    
      auto timer_callback =
      [this]() -> void {
        //RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        this->publisher_->publish(this->message);
      };
      
    timer_ = this->create_wall_timer(0ms, timer_callback);
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  std_msgs::msg::String message;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StringPublisher>());
  rclcpp::shutdown();
  return 0;
}
