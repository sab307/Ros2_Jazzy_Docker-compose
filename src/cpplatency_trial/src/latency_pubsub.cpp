#include"rclcpp/rclcpp.hpp"
#include"sensor_msgs/msg/image.hpp"
#include"std_msgs/msg/float64.hpp"

class ImageTimestampListener : public rclcpp::Node
{
public:
  ImageTimestampListener()
  : Node("image_timestamp_listener")
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "camera1/image_raw",   // topic name
      10,                   // QoS history depth
      std::bind(&ImageTimestampListener::topic_callback, this, std::placeholders::_1));

    publisher_ = this->create_publisher<std_msgs::msg::Float64>("camera1/image_latency_ms",10);

    subscription2_ = this->create_subscription<sensor_msgs::msg::Image>(
      "camera2/image_raw",   // topic name
      10,                   // QoS history depth
      std::bind(&ImageTimestampListener::topic_callback2, this, std::placeholders::_1));

    publisher2_ = this->create_publisher<std_msgs::msg::Float64>("camera2/image_latency_ms",10);
  }

private:
  void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    const auto & stamp = msg->header.stamp;

    rclcpp::Time pub_time(msg->header.stamp);
    rclcpp::Time recv_time = this->get_clock()->now();

    //Latency
    rclcpp::Duration latency = recv_time - pub_time;
    double latency_ms = latency.seconds() * 1000.0; //convert to ms

    //Publish Latency
    std_msgs::msg::Float64 latency_msg;
    latency_msg.data = latency_ms;
    publisher_->publish(latency_msg);
    
     // Log it too
    RCLCPP_INFO(this->get_logger(),
      "Image latency: %.3f ms (pub: %f.%09lu, recv: %u.%09u)",
      latency_ms,
      msg->header.stamp.sec, msg->header.stamp.nanosec,
      recv_time.seconds(), recv_time.nanoseconds() % 1000000000);
  }

  void topic_callback2(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    const auto & stamp = msg->header.stamp;

    rclcpp::Time pub_time(msg->header.stamp);
    rclcpp::Time recv_time = this->get_clock()->now();

    //Latency
    rclcpp::Duration latency = recv_time - pub_time;
    double latency_ms = latency.seconds() * 1000.0; //convert to ms

    //Publish Latency
    std_msgs::msg::Float64 latency_msg;
    //latency_msg.header.stamp = this->get_clock()->now();
    latency_msg.data = latency_ms;
    publisher2_->publish(latency_msg);
    
     // Log it too
    // RCLCPP_INFO(this->get_logger(),
    //   "Image latency: %.3f ms (pub: %f.%09lu, recv: %u.%09u)",
    //   latency_ms,
    //   msg->header.stamp.sec, msg->header.stamp.nanosec,
    //   recv_time.seconds(), recv_time.nanoseconds() % 1000000000);
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription2_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher2_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImageTimestampListener>());
  rclcpp::shutdown();
  return 0;
}
