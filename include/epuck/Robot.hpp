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

    std::string IP;
    
    Wheels wheels_state, wheels_command;
    Proximity_sensors proximity_sensors;
    Pose desired_pose, current_pose;
    cv::Mat vision_sensors;

    /*** ADDITIONNAL DATA FOR EPUCK V2 */

    // Sensors data variables

    float acceleration, orientation, inclination;		/**< acceleration data*/

    int16_t gyroRaw[3];
    float magneticField[3];
    uint8_t temperature;
    int lightAvg;										/**< light sensor data*/
    
    uint16_t distanceCm;
    uint16_t micVolume[4];								/**< microphone data*/
    int16_t motorSteps[2];
    uint16_t batteryRaw;
    uint8_t microSdState;
    uint8_t irCheck, irAddress, irData;
    uint8_t selector;
    int16_t groundProx[3], groundAmbient[3];
    uint8_t buttonState;
};
// current_pose.th [-pi/2;pi/2]
// wheels_state.*_position [-pi, pi]