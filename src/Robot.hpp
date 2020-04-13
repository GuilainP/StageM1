#include <iostream>
#include <string>
#include <vector>
#include <memory>

#include "RobotDriver.hpp"

class Robot {
    public:
        Robot() 
            : driver() {};
        Robot(RobotDriver* epuckDriver) 
            : driver(epuckDriver) {};
        ~Robot(){};
        RobotDriver* driver;
    private:
        
};