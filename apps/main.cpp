#include "RobotDriver.hpp"

#include <csignal>
#include <iostream>
#include <memory>

sig_atomic_t signaled = 0;
void exitLoop(int sig) {
    std::cout << " Loop termination\n";
    signaled = 1;
}

int main(int argc, char** argv) {

    bool real_robot = true;
    Robot robot;
    std::unique_ptr<RobotDriver> driver;
    if (real_robot) {
        driver = std::make_unique<EPuckV2Driver>(robot);
    } else {
        driver = std::make_unique<EPuckVREPDriver>(robot);
    }

    std::cout << "Target Velocity : \n"
              << "Left Speed  [rad/s]: ";
    std::cin >> robot.wheels_command.left_velocity;
    std::cout << "Right Speed [rad/s]: ";
    std::cin >> robot.wheels_command.right_velocity;
    std::cout << std::endl;

    driver->init();
    
    std::signal(SIGINT, exitLoop);
    while (true) {
        if (signaled == 1) {
            break;
        }

        driver->read();

        driver->getVisionSensor(robot);

        driver->send();

        

    }
}
