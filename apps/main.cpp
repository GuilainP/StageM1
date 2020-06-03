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


    Robot robot;
    robot.ip = argv[2];
    std::unique_ptr<RobotDriver> driver;

    if(robot.ip.size() == 11 ) {
        driver = std::make_unique<EPuckV1Driver>(robot,argv);
    } else if (robot.ip.size() == 13){
        driver = std::make_unique<EPuckV2Driver>(robot,argv);
    } else if (robot.ip == "127.0.0.1"){  
        driver = std::make_unique<EPuckVREPDriver>(robot,argv);
    } else {
        return -1;
    }

    driver->init();
    
    std::signal(SIGINT, exitLoop);
    while (true) {
        driver->read();

        driver->getVisionSensor(robot);

        driver->sendCmd();

        if (signaled == true) {
            break;
        }
    }

}

