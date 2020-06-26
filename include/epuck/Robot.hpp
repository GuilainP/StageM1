#pragma once

#include <array>
#include <opencv2/core/mat.hpp>

struct Pose {
    Pose(){}
    Pose(double x, double y, double th) {setPose(x,y,th);}
    void setPose(double x,double y, double th) {this->x=x; this->y=y; this->th=th;}
    double x{0}, y{0}, th{0};
};

struct Wheels {
    double left_velocity{0}, right_velocity{0};
    double left_position{0}, right_position{0};
};

struct Proximity_sensors {
    std::array<double, 8> ir{};
    std::array<double, 10> distances{};
};

struct RobotParameters {
    const std::array<double, 10> theta{-0.2268, -0.8371, -1.5708, -2.2391, 2.2391, 1.5708, 0.8371, 0.2268, 0.35, -0.35};
    const double robot_radius{0.033};
    const double wheel_radius{0.021};
    const double axis_length{0.052};
    const double dist_max{0.035};
};

class Robot {
public:
    Robot();
    ~Robot();

    std::string ip;
    
    Wheels wheels_state, wheels_command;
    Proximity_sensors proximity_sensors;
    Pose desired_pose, current_pose;
    double desired_velocity, current_velocity; // v
    double desired_angle, current_angle; // w
    cv::Mat vision_sensors;
    RobotParameters parameters;

};
// current_pose.th [-pi/2;pi/2]
// wheels_state.*_position [-pi, pi]