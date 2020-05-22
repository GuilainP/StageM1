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

    std::string real_robot = argv[1];
    Robot robot;
    std::unique_ptr<RobotDriver> driver;
    if(real_robot == "" ) {
        driver = std::make_unique<EPuckVREPDriver>(robot);
    } else {
        robot.IP = argv[1];
        driver = std::make_unique<EPuckV2Driver>(robot);
    }

    std::cout << "Target Velocity : \n"
              << "Left Speed  [rad/s]: ";
    std::cin >> robot.wheels_command.left_velocity;
    std::cout << "Right Speed [rad/s]: ";
    std::cin >> robot.wheels_command.right_velocity;
    std::cout << std::endl;

    driver->Init();
    
    std::signal(SIGINT, exitLoop);
    while (true) {
        if (signaled == 1) {
            break;
        }

        driver->Read();

        driver->getVisionSensor(robot);

        driver->Send();
    }


}

