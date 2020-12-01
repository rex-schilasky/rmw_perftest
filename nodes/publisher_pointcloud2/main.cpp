#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

using namespace std::chrono_literals;

class PointCloudPublisher : public rclcpp::Node
{
public:
  PointCloudPublisher()
  : Node("pointcloud_publisher")
  {
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("PointCloud2", 10);
    message_ = sensor_msgs::msg::PointCloud2();

    FillMessage(message_);
   
      auto timer_callback =
      [this]() -> void {
        //RCLCPP_INFO(this->get_logger(), "Publishing");
        this->publisher_->publish(this->message_);
      };
      
    timer_ = this->create_wall_timer(0ms, timer_callback);
  }

  void FillMessage(sensor_msgs::msg::PointCloud2& msg)
  {
    const size_t   POINTS     = 4096;
    const uint32_t POINT_STEP = 256;
    msg.header.frame_id = 123;
    //msg.header.stamp = 456;
    msg.fields.resize(5);
    msg.fields[0].name = "x";
    msg.fields[0].offset = 0;
    msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields[0].count = 1;
    msg.fields[1].name = "y";
    msg.fields[1].offset = 4;
    msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields[1].count = 1;
    msg.fields[2].name = "z";
    msg.fields[2].offset = 8;
    msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields[2].count = 1;
    msg.fields[3].name = "intensity";
    msg.fields[3].offset = 16;
    msg.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields[3].count = 1;
    msg.fields[4].name = "ring";
    msg.fields[4].offset = 20;
    msg.fields[4].datatype = sensor_msgs::msg::PointField::UINT16;
    msg.fields[4].count = 1;
    msg.data.resize(POINTS * POINT_STEP, 0x00);
    msg.point_step = POINT_STEP;
    msg.row_step = static_cast<uint32_t>(msg.data.size());
    msg.height = 1;
    msg.width = msg.row_step / POINT_STEP;
    msg.is_bigendian = false;
    msg.is_dense = true;
    uint8_t* ptr = msg.data.data();

    for (size_t i = 0; i < POINTS; ++i)
    {
      *(reinterpret_cast<float*>(ptr + 0)) = 1.0f;
      *(reinterpret_cast<float*>(ptr + 4)) = 2.0f;
      *(reinterpret_cast<float*>(ptr + 8)) = 3.0f;
      *(reinterpret_cast<float*>(ptr + 16)) = 4.0f;
      *(reinterpret_cast<uint16_t*>(ptr + 20)) = 1234;
      ptr += POINT_STEP;
    }
  }

private:
  rclcpp::TimerBase::SharedPtr                                 timer_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr  publisher_;
  sensor_msgs::msg::PointCloud2                                message_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudPublisher>());
  rclcpp::shutdown();
  return 0;
}
