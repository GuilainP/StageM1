#include "RobotDriver.hpp"

#include <iostream>

EPuckV2Driver::EPuckV2Driver(Robot& robot) : RobotDriver(robot) {
}

// TODO
void EPuckV2Driver::init() {
    std::cout << "init V1\n";
};

// TODO
void EPuckV2Driver::read() {
    std::cout << "read V1\n";
};

// TODO
void EPuckV2Driver::send() {
    std::cout << "send V1\n";
};

// TODO
void EPuckV2Driver::getImage(Robot& robot) {
    std::cout << "Image V2\n";
}