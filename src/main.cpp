#include "RobotDriver.hpp"

int main(){
    
    bool real_robot = false;
    Robot robot;
    std::unique_ptr<RobotDriver> driver;
    if(real_robot) {
        driver = std::make_unique<EPuckV2Driver>(robot);
    } else {
        driver = std::make_unique<EPuckVREPDriver>(robot);
    }
    
    driver->init();
    

    for(int i =0; i<100 ; i++)
        driver->read();

    driver->send();    

}