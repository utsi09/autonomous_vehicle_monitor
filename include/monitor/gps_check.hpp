#pragma once
#include "behaviortree_ros2/bt_topic_sub_node.hpp"
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <cmath>
using namespace std;

class GpsChecker : public BT::SyncActionNode
{
public:
    GpsChecker(const std::string& name, const BT::NodeConfig& conf,
               const BT::RosNodeParams& params)
               : SyncActionNode(name, conf)
    {
        node_ = params.nh.lock();
        std::string topic;
        if (getInput("topic_name", topic) && !topic.empty()) {
            topic_ = topic;
            rclcpp::QoS qos(10);
            qos.best_effort();
            sub_ = node_->create_subscription<sensor_msgs::msg::NavSatFix>(
                topic_, qos,
                [this](const sensor_msgs::msg::NavSatFix::SharedPtr){
                    last_time_ = node_->now();
                }
            );
        }
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("topic_name"),
            BT::InputPort<double>("gps_ftti"),
        };
    }

    BT::NodeStatus tick() override
    {
        if (topic_.empty()) {
            return BT::NodeStatus::SUCCESS;
        }
        double ftti = 850.0;
        getInput("gps_ftti", ftti);
        double timeout = -1.0;
        if (last_time_.nanoseconds() != 0) {
            timeout = (node_->now() - last_time_).seconds() * 1000;
        }
        if (timeout < 0 || timeout > period_ms_ * 3) {
            fail_count_++;
        } else {
            fail_count_ = 0;
        }
        cout << name() << " timeout: " << timeout << "ms / ftti: " << ftti << "ms / fail: " << fail_count_ << endl;
        if (fail_count_ >= 3)
            return BT::NodeStatus::FAILURE;
        return BT::NodeStatus::SUCCESS;
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_;
    rclcpp::Time last_time_;
    std::string topic_;
    double period_ms_ = 50.0;
    int fail_count_ = 0;
};
