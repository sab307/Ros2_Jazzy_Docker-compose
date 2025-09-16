#include<chrono>
#include<fstream>
#include<iomanip>
#include<memory>
#include<sstream>
#include<string>
#include<vector>
#include "ament_index_cpp/get_package_share_directory.hpp"

#include"rclcpp/rclcpp.hpp"
#include"webots_ros2_msgs/msg/float_stamped.hpp"
#include"yaml-cpp/yaml.h"

struct Rule
{
    std::string topic;
    std::string type;
    std::string field;
    std::string condition; // below or above
    double threshold;
};

class MultipleMonitorNode : public rclcpp::Node
{
public:
    MultipleMonitorNode(const std::string &config_file) : Node("multiple_monitor_node")
    {
        // open log file
        logfile_.open("event_multiple.log", std::ios::app);
        if(!logfile_.is_open())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open event_multiple.log for writing");
            throw std::runtime_error("log file open failed");
        }

        // Load YAML config
        YAML::Node config = YAML::LoadFile(config_file);
        for(auto rule_node : config["topics"])
        {
            Rule rule;
            if(rule_node["name"]) rule.topic = rule_node["name"].as<std::string>();
            if(rule_node["type"]) rule.topic = rule_node["type"].as<std::string>();
            if(rule_node["field"]) rule.topic = rule_node["field"].as<std::string>();
            if(rule_node["condition"]) rule.topic = rule_node["condition"].as<std::string>();
            if(rule_node["threshold"]) rule.topic = rule_node["threshold"].as<double>();
            rules_.push_back(rule);

            // Only std_msgs/Float32 supported for now.
            if(rule.type == "webots_ros2_msgs/msg/FloatStamped")
            {
                auto sub = this->create_subscription<webots_ros2_msgs::msg::FloatStamped>(
                    rule.topic, 10,
                    [this, rule](webots_ros2_msgs::msg::FloatStamped::SharedPtr msg)
                    {
                        this->check_rule(msg->data, rule);
                    });
                subs_.push_back(sub);
                RCLCPP_INFO(this->get_logger(), "Monitoring %s (%s %s %.2f)",
                            rule.topic.c_str(),
                            rule.field.c_str(),
                            rule.condition.c_str(),
                            rule.threshold);
            }
        }
    }
    ~MultipleMonitorNode() override
    {
        if(logfile_.is_open())
        {
            logfile_.close();
        }
    }

private:
    void check_rule(double value, const Rule &rule)
    {
        bool triggered = false;
        if(rule.condition == "below" && value < rule.threshold) triggered = true;
        if(rule.condition == "above" && value > rule.threshold) triggered = true;

        if(triggered)
        {
            auto now = this->get_clock()->now();

            // Human readable wall clock time
            auto sys_now = std::chrono::system_clock::now();
            std::time_t t = std::chrono::system_clock::to_time_t(sys_now);
            std::tm tm = *std::localtime(&t);
            std::ostringstream oss;
            oss << std::put_time(&tm, "%Y-%m-%d %H:%M:%S");

            logfile_ << "[" << oss.str() << "]"
                     << "ROS " << now.seconds() << "." << now.nanoseconds() % 1000000000
                     << "| Topic: " << rule.topic
                     << "| Value:" << value
                     << "| Condition: " << rule.condition << " " << rule.threshold
                     << std::endl;

            RCLCPP_WARN(this->get_logger(), "Event logged: %.2f %s %.2f (%s)",
                        value, rule.condition.c_str(), rule.threshold, rule.topic.c_str());
        }
    }

    std::ofstream logfile_;
    std::vector<Rule> rules_;
    std::vector<rclcpp::SubscriptionBase::SharedPtr> subs_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    std::string pkg_share = ament_index_cpp::get_package_share_directory("cpp_logging_node");
    std::string config_file = pkg_share + "/config/rules.yaml";

    //std::string config_file = "config/rules.yaml";
    auto node = std::make_shared<MultipleMonitorNode>(config_file);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}