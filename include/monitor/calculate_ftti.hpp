#pragma once
#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_ros2/ros_node_params.hpp"
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <cmath>
#include <vector>
#include <string>
using namespace BT;
using namespace std;
class CalculateFtti : public SyncActionNode {
public:
    CalculateFtti(const std::string& name, const NodeConfig& config);

    static PortsList providedPorts();

    NodeStatus tick() override;

private:
    double calculate_vel(double vel_x, double vel_y);
    double calculate_ttc(double vel, double dist);
    vector<string> parsing_port(string raw);
};
