#include "RobotDriver.hpp"

RobotDriver::RobotDriver(){
    
}

RobotDriver::~RobotDriver(){
    
}


EPuckV1Driver::EPuckV1Driver(Robot r0){
    name = "EpuckV1" ;
    robot = r0;
    printName();
}

void EPuckV1Driver::printName(){
    std::cout << name << std::endl;
}

EPuckV2Driver::EPuckV2Driver(Robot r0){
    name = "EpuckV2" ;
    robot = r0;
    printName();
}

void EPuckV2Driver::printName(){
    std::cout << name << std::endl;
}

EPuckVREPDriver::EPuckVREPDriver(Robot r0){
    name = "EpuckV1" ;
    robot = r0;
    printName();
}

void EPuckVREPDriver::printName(){
    std::cout << name << std::endl;
}
