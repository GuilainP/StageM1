#include "RobotDriver.hpp"

#include <csignal>
#include <iostream>

sig_atomic_t signaled = 0;
void exitLoop(int sig) {
    std::cout << " Loop termination\n";
    signaled = 1;
}

int main(int argc, char** argv) {

    bool real_robot = false;
    Robot robot;
    std::unique_ptr<RobotDriver> driver;
    if (real_robot) {
        driver = std::make_unique<EPuckV2Driver>(robot);
    } else {
        driver = std::make_unique<EPuckVREPDriver>(robot);
    }

    driver->init();

    std::signal(SIGINT, exitLoop);
    while (true) {
        if (signaled == 1) {
            break;
        }
        driver->read();
        driver->send();
    }
}
