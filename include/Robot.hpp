#pragma once

#include <iostream>
#include <array>
#include <fstream>
#include <string>
#include <vector>
#include <memory>
#include <ctime>
#include <cmath>

struct Pose{
    double x,y,th;
};

struct Wheels{
    double left_velocity, right_velocity;
    double left_position, right_position;
};

struct Proximity_sensors{
    std::array<double, 8> IR;
};

struct Vision_sensors{ // à corriger plus tard
    double** Image;
};

struct robotParameters {
    const double theta[10] = {-0.2268, -0.8371, -1.5708, -2.2391, 2.2391, 1.5708,  0.8371,  0.2268,  0.35,    -0.35};
    const double robRadius = 0.033;
    const double wheel_radius = 0.021;
    const double axisLength = 0.052;
    const double distMax = 0.035;
};

class Robot{
    public:
        Robot(){std::cout << "Robot CONSTRUCTED" << std::endl;}
        ~Robot(){std::cout << "Robot DESTRUCTED" << std::endl;}

        Wheels wheels_state, wheels_command;
        Proximity_sensors proximity_sensors;
        Vision_sensors vision_sensors;
        Pose desired_pose, current_pose;
};
