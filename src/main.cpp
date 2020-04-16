#include "RobotDriver.hpp"
#include <memory>

int main(){
    
    bool real_robot = true;
    Robot robot;
    std::unique_ptr<RobotDriver> driver;
    if(real_robot) {
        driver = std::make_unique<EPuckV2Driver>(robot);
    } else {
        driver = std::make_unique<EPuckVREPDriver>(robot);
    }

    driver->printName();

}