#include "RobotDriver.hpp"

#include <iostream>

EPuckV1Driver::EPuckV1Driver(Robot& robot) : RobotDriver(robot) {
}

// TODO
void EPuckV1Driver::init() {
    std::cout << "init V1\n";
};

// TODO
void EPuckV1Driver::read() {
    std::cout << "read V1\n";
};

// TODO
void EPuckV1Driver::send() {
    std::cout << "send V1\n";
};

// TODO
void EPuckV1Driver::getImage(Robot& robot) {
    std::cout << "Image V1\n";
}