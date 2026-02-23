#include "monitor/calculate_ftti.hpp"
#include <tf2/utils.h>
#include <iostream>
#include <string.h>
using namespace std;

CalculateFtti::CalculateFtti(const string& name, const NodeConfig& config
                                        )
    : SyncActionNode(name,config)
{}

PortsList CalculateFtti::providedPorts() {
    return {
        InputPort<double>("nearest_obstacle"),
        InputPort<string>("important_sensors"),
        OutputPort<double>("lidar_ftti"),
        OutputPort<double>("cam_ftti"),
        OutputPort<double>("imu_ftti"),
        OutputPort<double>("gps_ftti"),   
    };
}
NodeStatus CalculateFtti::tick() {
    string important_sensors = "";
    double lidar_ftti = 10.0;
    double cam_ftti = 10.0;
    double imu_ftti = 10.0;
    double gps_ftti = 10.0;
    getInput("important_sensors", important_sensors);
    cout << "입력" << important_sensors << endl;
    
    setOutput("lidar_ftti", lidar_ftti);
    setOutput("cam_ftti", cam_ftti);
    setOutput("imu_ftti", imu_ftti);
    setOutput("gps_ftti", gps_ftti);

    return NodeStatus::SUCCESS;
}
