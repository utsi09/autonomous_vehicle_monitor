#include "monitor/calculate_ftti.hpp"
#include <tf2/utils.h>
#include <iostream>
#include <string.h>
using namespace std;

double CalculateFtti::calculate_vel(double vel_x, double vel_y) {
    return sqrt((vel_x * vel_x) + (vel_y * vel_y));
}

double CalculateFtti::calculate_ttc(double vel, double dist){
    return (dist/vel) * 1000;
}
vector<string> CalculateFtti::parsing_port(string raw){
    vector<string> result = {};
    uint start = 0;
    uint end = raw.find(',');
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
        InputPort<double>("vel_x"),
        InputPort<double>("vel_y"),
        InputPort<double>("nearest_obstacle"),
        InputPort<string>("important_sensors"),
        OutputPort<double>("lidar_ftti"),
        OutputPort<double>("cam_ftti"),
        OutputPort<double>("imu_ftti"),
        OutputPort<double>("gps_ftti"),   
    };
}
NodeStatus CalculateFtti::tick() {
    double vel_x = 0.0;
    double vel_y = 0.0;
    double nearest_obstacle = 1000.0;
    string important_sensors = "";
    double lidar_ftti = 10.0;
    double cam_ftti = 10.0;
    double imu_ftti = 10.0;
    double gps_ftti = 10.0;
    
    getInput("vel_x", vel_x);
    getInput("vel_y", vel_y);
    double vel = calculate_vel(vel_x, vel_y);

    getInput("important_sensors", important_sensors);

    getInput("nearest_obstacle", nearest_obstacle);
    if (!nearest_obstacle){
        nearest_obstacle = 100.0; //예외처리
    }
    double ttc = calculate_ttc(vel, nearest_obstacle);

    vector<string> sensor_vector = parsing_port(important_sensors);
    for (auto str : sensor_vector){
        cout << "파싱 테스트"<< str << endl;
    }
    cout << "dist " << nearest_obstacle << "m" << endl;
    cout << "ttc " << ttc << "ms" << endl;
    
    

    setOutput("lidar_ftti", lidar_ftti);
    setOutput("cam_ftti", cam_ftti);
    setOutput("imu_ftti", imu_ftti);
    setOutput("gps_ftti", gps_ftti);

    return NodeStatus::SUCCESS;
}
