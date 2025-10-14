#include <chrono>
#include <memory>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class ListenerWithTime : public rclcpp::Node
{
public:
  ListenerWithTime()
  : Node("listener_with_time")
  {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "chatter", 10,
      std::bind(&ListenerWithTime::topic_callback, this, std::placeholders::_1));
  }

private:
  void topic_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    // Get system time at reception
    auto now = this->get_clock()->now();

    RCLCPP_INFO(
      this->get_logger(),
      "Received: '%s' at time: %f.%09lu",
      msg->data.c_str(),
      now.seconds(),
      now.nanoseconds() % 1000000000  // extract nanosecond part
    );
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ListenerWithTime>());
  rclcpp::shutdown();
  return 0;
}


// #include <cstdio>

// int main(int argc, char ** argv)
// {
//   (void) argc;
//   (void) argv;

//   printf("hello world twist_reader package\n");
//   return 0;
// }
