#include "monitor/calculate_ftti.hpp"
#include <tf2/utils.h>
#include <iostream>
#include <string.h>
using namespace std;

double CalculateFtti::calculate_vel(double vel_x, double vel_y) {
    return sqrt((vel_x * vel_x) + (vel_y * vel_y));
}

double CalculateFtti::calculate_ttc(double vel, double dist){
    if (vel != 0)
        return (dist/vel) * 1000;
    else
        return 1000.0;
}
vector<string> CalculateFtti::parsing_port(string raw){
    vector<string> result = {};
    size_t start = 0;
    size_t end = raw.find(',');
    while (end != string::npos){
        result.push_back(raw.substr(start,end-start));
        start = end+1;
        end = raw.find(',', start);
    }
    result.push_back(raw.substr(start));
    return result;
}

CalculateFtti::CalculateFtti(const string& name, const NodeConfig& config)
    : SyncActionNode(name,config)
{}

PortsList CalculateFtti::providedPorts() {
    return {
        InputPort<vector<string>>("lidar_names"),
        InputPort<vector<string>>("camera_names"),
        InputPort<double>("vel_x"),
        InputPort<double>("vel_y"),
        InputPort<double>("nearest_obstacle"),
        InputPort<string>("important_sensors"), 
    };
}
NodeStatus CalculateFtti::tick() {
    double vel_x = 0.0;
    double vel_y = 0.0;
    double nearest_obstacle = 1000.0;
    string important_sensors = "";
    vector<string> lidar_names;
    vector<string> camera_names;
    
    getInput("vel_x", vel_x);
    getInput("vel_y", vel_y);
    double vel = calculate_vel(vel_x, vel_y);

    getInput("important_sensors", important_sensors);

    getInput("nearest_obstacle", nearest_obstacle);

    if (nearest_obstacle <= 0.0){
        nearest_obstacle = 100.0;
    }

    double ttc = calculate_ttc(vel, nearest_obstacle);

    vector<string> important_sensors_names;
    if (!important_sensors.empty())
        important_sensors_names = parsing_port(important_sensors);


    cout << "dist " << nearest_obstacle << "m" << endl;
    cout << "ttc " << ttc << "ms" << endl;
    
    map<string,double> ftti_map;
    
    getInput("lidar_names", lidar_names);
    getInput("camera_names", camera_names);
    
    auto bb = config().blackboard;

    for (const auto& name : lidar_names) {
        ftti_map[name] = 850.0;
    }
    for (const auto& name : camera_names) {
        ftti_map[name] = 850.0;
    }
    for (const auto& name : important_sensors_names) {
        if (ttc < 850.0)
            ftti_map[name] = ttc;
    }

    for (auto& [name,value] : ftti_map){
        bb->set(name + "_ftti", value);
        RCLCPP_INFO(rclcpp::get_logger("ftti_names"), "%s : %.2f", name.c_str(), value);
    }
    

    return NodeStatus::SUCCESS;
}
