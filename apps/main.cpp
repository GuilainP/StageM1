#include "RobotDriver.hpp"

#include <csignal>
#include <iostream>
#include <memory>


/*****************************/
/**** Main Program ***********/
/*****************************/


bool signaled = false;
void exitLoop(int sig) {
    std::cout << " Loop termination\n";
    signaled = true;
}



int main(int argc, char** argv) {

    std::string real_robot = argv[1];
    Robot robot;
    std::unique_ptr<RobotDriver> driver;
    if(real_robot == "SIM" ) {
        driver = std::make_unique<EPuckVREPDriver>(robot);
    } else if (real_robot == "V2"){
        robot.ip = argv[2];
        driver = std::make_unique<EPuckV2Driver>(robot);
    } else {
        robot.ip = argv[2];
        driver = std::make_unique<EPuckV1Driver>(robot,argv, signaled);
    }

    if(real_robot == "SIM" || real_robot == "V2") {
        std::cout << "Target Velocity : \n"
              << "Left Speed  [rad/s]: ";
        std::cin >> robot.wheels_command.left_velocity;
        std::cout << "Right Speed [rad/s]: ";
        std::cin >> robot.wheels_command.right_velocity;
        std::cout << std::endl;
    }

    driver->Init();
    
    std::signal(SIGINT, exitLoop);
    while (true) {
        if (signaled == true) {
            break;
        }

        driver->Read();

        driver->getVisionSensor(robot);

        driver->Send();
    }


}

