#include<chrono>
#include<memory>
#include<random>

#include<rclcpp/rclcpp.hpp>
#include"webots_ros2_msgs/msg/float_stamped.hpp"
//#include"rclcpp/timer.hpp"

using namespace std::chrono_literals;

class RandomPublisher : public rclcpp::Node
{
public:
    RandomPublisher() : Node("random_float_publisher"), gen_(rd_()), dist_(0.0, 60.0)
    {
        publisher_ = this->create_publisher<webots_ros2_msgs::msg::FloatStamped>(
            "/random_value", 10);
        
        //this->declare_parameter("publish_rate", 20); //Hz
        //double rate = this->get_parameter("publish_rate").as_double();

        timer_ = this->create_wall_timer(
            //std::chrono::milliseconds(100ms)//std::chrono::milliseconds(static_cast<int>(10000.0/ rate))
            10ms
            , std::bind(&RandomPublisher::publish_random_number, this));
    }
private:
    void publish_random_number()
    {
        auto message = webots_ros2_msgs::msg::FloatStamped();

        // Fill header with the current ROS Time
        message.header.stamp = this->get_clock()->now();
        message.header.frame_id = "random_generator"; //optional frame

        // Generate random number
        message.data = dist_(gen_);

        RCLCPP_INFO(this->get_logger(), "Publishing: %.2f", message.data);
        publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<webots_ros2_msgs::msg::FloatStamped>::SharedPtr publisher_;
    
    // Random number generator
    std::random_device rd_;
    std::mt19937 gen_;
    std::uniform_real_distribution<> dist_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RandomPublisher>());
    rclcpp::shutdown();
    return 0;
}