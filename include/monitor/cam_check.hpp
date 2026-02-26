#pragma once
#include "behaviortree_ros2/bt_topic_sub_node.hpp"
#include <sensor_msgs/msg/compressed_image.hpp>
#include <cmath>
using namespace std;

class CamChecker : public BT::SyncActionNode
{
public:
    CamChecker(const std::string& name, const BT::NodeConfig& conf,
               const BT::RosNodeParams& params)
               : SyncActionNode(name, conf)
    {
        node_ = params.nh.lock();
        std::string topic;
        if (getInput("topic_name", topic) && !topic.empty()) {
            topic_ = topic;
            rclcpp::QoS qos(10);
            qos.best_effort();
            sub_ = node_->create_subscription<sensor_msgs::msg::CompressedImage>(
                topic_, qos,
                [this](const sensor_msgs::msg::CompressedImage::SharedPtr){
                    last_time_ = node_->now();
                }
            );
        }
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("topic_name"),
            BT::InputPort<double>("cam_ftti"),
        };
    }

    BT::NodeStatus tick() override
    {
        if (topic_.empty()) {
            return BT::NodeStatus::SUCCESS;
        }
        double ftti = 850.0;
        getInput("cam_ftti", ftti);
        double timeout = -1.0;
        if (last_time_.nanoseconds() != 0) {
            timeout = (node_->now() - last_time_).seconds() * 1000;
        }
        cout << name() << " timeout: " << timeout << "ms / ftti: " << ftti << "ms" << endl;
        if (timeout < 0 || timeout > ftti)
            return BT::NodeStatus::FAILURE;
        else
            return BT::NodeStatus::SUCCESS;
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_;
    rclcpp::Time last_time_;
    std::string topic_;
};
