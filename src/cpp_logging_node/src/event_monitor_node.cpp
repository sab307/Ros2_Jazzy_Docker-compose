#include<chrono>
#include<fstream>
#include<iomanip>
#include<memory>
#include<string>

#include"rclcpp/rclcpp.hpp"
#include"webots_ros2_msgs/msg/float_stamped.hpp"

using std::placeholders::_1;

class Event_Monitor : public rclcpp::Node
{
public:
    Event_Monitor() : Node("event_monitor_cpp")
    {
        // open log file
        logfile_.open("events.log", std::ios::app);
        if(!logfile_.is_open())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open events. log for writing");
            throw std::runtime_error("log file open failed");
        }

        // Subscribe to topic
        sub_ = this->create_subscription<webots_ros2_msgs::msg::FloatStamped>(
            "/random_value", 10, std::bind(&Event_Monitor::callback, this, _1));

        RCLCPP_INFO(this->get_logger(), "Monitoring the random value and logging events < 40");
    }

    ~Event_Monitor()
    {
        if(logfile_.is_open())
        {
            logfile_.close();
        }
    }
private:
    void callback(const webots_ros2_msgs::msg::FloatStamped::SharedPtr msg)
    {
        double value = msg->data;
        if(value > 40.0)
        {
            auto now = this->get_clock()->now();

            // Convert system time to readable string
            auto sys_now = std::chrono::system_clock::now();
            std::time_t t = std::chrono::system_clock::to_time_t(sys_now);
            std::tm tm = *std::localtime(&t);

            std::ostringstream oss;
            oss << std::put_time(&tm, "%Y-%m-%d %H:%M:%S");

            // Log to file
            logfile_ << "[" << oss.str() << "]"
                     << "ROS TIME" << now.seconds() << "." << now.nanoseconds() % 1000000000
                     << "| Topic: /random_value"
                     << "| Value:" << value
                     << " Condtition: above 40"
                     << std::endl;

            RCLCPP_WARN(this->get_logger(), "Logged event: %.2f < 40", value);
        }
    }

    rclcpp::Subscription<webots_ros2_msgs::msg::FloatStamped>::SharedPtr sub_;
    std::ofstream logfile_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Event_Monitor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}