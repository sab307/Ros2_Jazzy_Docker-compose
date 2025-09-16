#include<rclcpp/rclcpp.hpp>
#include<std_msgs/msg/float64.hpp>
#include<yaml-cpp/yaml.h>
#include<sqlite3.h>
#include<fstream>
#include<iostream>
#include<memory>
#include<vector>
#include<functional>

struct Rule {
    std::string topic;
    std::string type;
    std::string field;
    std::string condition; //below or above
    double threshold;
};

class EventMonitor : public rclcpp::Node
{
public:
    EventMonitor(const std::string &yaml_file) : Node("event_monitor") 
    {
        // Open SQLite database
        if(sqlite3_open("events.db", &db_) != SQLITE_OK) 
        {
            RCLCPP_ERROR(get_logger(), "Cannot open database: %s", sqlite3_errmsg(db_));
            throw std::runtime_error("Cannot open SQLite DB");
        }

        // Create Table
        const char* create_table_sql = 
            "CREATE TABLE IF NOT EXISTS events ("
            "id INTEGER PRIMARY KEY AUTOINCREMENT, "
            "timestamp TEXT, "
            "ros_time_sec INTEGER, "
            "ros_time_nsec INTEGER, "
            "topic TEXT, "
            "field TEXT, "
            "value REAL, "
            "condition TEXT, "
            "threshold REAL);";

        char* errmsg;
        if(sqlite3_exec(db_, create_table_sql, 0, 0, &errmsg) != SQLITE_OK)
        {
            RCLCPP_ERROR(get_logger(), "Failed to create table: %s", errmsg);
            sqlite3_free(errmsg);
        }

        // Load YAML
        YAML::Node config = YAML::LoadFile(yaml_file);
        for (auto rule_node : config["topics"]) 
        {
            Rule rule;
            rule.topic = rule_node["name"].as<std::string>();
            rule.type = rule_node["type"].as<std::string>();
            rule.field = rule_node["field"].as<std::string>();
            rule.condition = rule_node["condition"].as<std::string>();
            rule.threshold = rule_node["threshold"].as<double>();
            rules_.push_back(rule);

            // Currently supporting Float64 only
            if(rule.type == "std_msgs/msg/Float64")
            {
                auto sub = this->create_subscription<std_msgs::msg::Float64>(
                    rule.topic, 10,
                    [this, rule](std_msgs::msg::Float64::SharedPtr msg) 
                    {
                        this->check_rule(msg->data, rule);
                    }
                );
                subs_.push_back(sub);
                RCLCPP_INFO(this->get_logger(), "Monitoring %s", rule.topic.c_str());

            }
        }
    }

    ~EventMonitor()
    {
        sqlite3_close(db_);
    }

private:
    void check_rule(double value, const Rule &rule)
    {
        bool trigger = false;
        if(rule.condition == "below" && value < rule.threshold) trigger = true;
        if(rule.condition == "above" && value > rule.threshold) trigger = true;

        if(trigger)
        {
            // ROS Time
            auto now = this->get_clock()->now();
            // std::string timestamp = now.to_chrono<std::chrono::seconds>().time_since_epoch().count()>0 ?
            //     std::to_string(now.seconds()) : "0";
            auto t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
            std::string timestamp = std::string(std::ctime(&t));
            timestamp.pop_back();

            // Insert into SQLite
            sqlite3_stmt* stmt;
            const char* sql = "INSERT INTO events "
                              "(timestamp, ros_time_sec, ros_time_nsec, topic, field, value, condition, threshold)"
                              "VALUES (?, ?, ?, ?, ?, ?, ?, ?);";

            if(sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr) == SQLITE_OK)
            {
                sqlite3_bind_text(stmt, 1, timestamp.c_str(), -1, SQLITE_TRANSIENT);
                sqlite3_bind_int64(stmt, 2, now.seconds()); // sec
                sqlite3_bind_int64(stmt, 3, now.nanoseconds() % 1000000000); // nsec
                sqlite3_bind_text(stmt, 4, rule.topic.c_str(), -1, SQLITE_TRANSIENT);
                sqlite3_bind_text(stmt, 5, rule.field.c_str(), -1, SQLITE_TRANSIENT);
                sqlite3_bind_double(stmt, 6, value);
                sqlite3_bind_text(stmt, 7, rule.condition.c_str(), -1, SQLITE_TRANSIENT);
                sqlite3_bind_double(stmt, 8, rule.threshold);
                if(sqlite3_step(stmt) != SQLITE_DONE) 
                {
                    RCLCPP_ERROR(this->get_logger(), "Failed to insert event");
                }
                sqlite3_finalize(stmt);
            }
            RCLCPP_WARN(this->get_logger(), "Event logged: %s value=%.2f %s %.2f",
                        rule.topic.c_str(), value, rule.condition.c_str(), rule.threshold);
        }
    }
    sqlite3* db_;
    std::vector<Rule> rules_;
    std::vector<rclcpp::SubscriptionBase::SharedPtr> subs_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EventMonitor>("rules.yaml");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}