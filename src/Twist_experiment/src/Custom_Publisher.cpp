#include<chrono>
#include<memory>
#include"rclcpp/rclcpp.hpp"
#include"custom_messages/msg/custom.hpp"

using namespace std::chrono_literals;

class CustomPublisher : public rclcpp::Node
{
public:
    CustomPublisher() : Node("custom_publisher"), count_(0)
    {
        publisher_ = this->create_publisher<custom_messages::msg::Custom>("custom_msg", 10);

        auto timer_callback = [this](){
            auto message = custom_messages::msg::Custom();
            message.data = this->count_++;
            RCLCPP_INFO_STREAM(this->get_logger(), "Publishing: '" << message.data << "'");
            publisher_->publish(message);
        };
        timer_ = this->create_wall_timer(500ms, timer_callback);
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<custom_messages::msg::Custom>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CustomPublisher>());
    rclcpp::shutdown();
    return 0;
}