#pragma once

#include <array>
#include <opencv2/core/mat.hpp>

struct Pose {
    double x{0}, y{0}, th{0};
};

struct Wheels {
    double left_velocity{0}, right_velocity{0};
    double left_position{0}, right_position{0};
};

struct Proximity_sensors {
    std::array<double, 8> IR{};
};

struct robotParameters {
    const std::array<double, 10> theta{-0.2268, -0.8371, -1.5708, -2.2391, 2.2391, 1.5708, 0.8371, 0.2268, 0.35, -0.35};
    const double robRadius{0.033};
    const double wheel_radius{0.021};
    const double axisLength{0.052};
    const double distMax{0.035};
};

class Robot {
public:
    Robot();
    ~Robot();

    Wheels wheels_state, wheels_command;
    Proximity_sensors proximity_sensors;
    Pose desired_pose, current_pose;
    cv::Mat vision_sensors;

    uchar led2[3];
    uchar led4[3];
    uchar led6[3];
    uchar led8[3];
};
// current_pose.th [-pi/2;pi/2]
// wheels_state.*_position [-pi, pi]