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
    double desired_velocity, current_velocity;
    cv::Mat vision_sensors;
    RobotParameters parameters;
    /*** ADDITIONNAL DATA FOR EPUCK V2 */

    // Sensors data variables

    float acceleration, orientation, inclination;		/**< acceleration data*/

    int16_t gyro_raw[3];
    float magnetic_field[3];
    uint8_t temperature;
    int light_avg;										/**< light sensor data*/
    
    uint16_t distance_cm;
    uint16_t mic_volume[4];								/**< microphone data*/
    uint16_t battery_raw;
    uint8_t micro_sd_state;
    uint8_t ir_check, ir_address, ir_data;
    uint8_t selector;
    int16_t ground_prox[3], ground_ambient[3];
    uint8_t button_state;
};
// current_pose.th [-pi/2;pi/2]
// wheels_state.*_position [-pi, pi]