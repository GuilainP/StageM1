#include "Robot.hpp"

#include <iostream>

Robot::Robot() {
    std::cout << "Robot CONSTRUCTED" << std::endl;
    vision_sensors = cv::Mat(128, 128, CV_8UC1);
}

Robot::~Robot() {
    std::cout << "Robot DESTRUCTED" << std::endl;
}

