#include "RobotDriver.hpp"

#include <iostream>

EPuckV1Driver::EPuckV1Driver(Robot& robot) : RobotDriver(robot) {
}

// TODO
bool EPuckV1Driver::Init() {
    std::cout << "init V1\n";
    return true;
};

// TODO
void EPuckV1Driver::Read() {
    std::cout << "read V1\n";
};

// TODO
void EPuckV1Driver::Send() {
    std::cout << "send V1\n";
};

// TODO
void EPuckV1Driver::getVisionSensor(Robot& robot) {
    std::cout << "Image V1\n";
}