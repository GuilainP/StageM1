#include "RobotDriver.hpp"

#include <csignal>
#include <iostream>
#include <memory>



void incorrectArguments(const int& argc);

bool signaled = false;
void exitLoop(int sig) {
    std::cout << " Loop termination\n";
    signaled = true;
}

int main(int argc, char** argv) {

    enum Controller { setWheelCmd, setVel, setRobVel, followWall, visServo };
    Controller c;

    Robot robot;
    robot.ip = argv[2];
    std::unique_ptr<RobotDriver> driver;

    if(robot.ip.size() == 11 ) {
        driver = std::make_unique<EPuckV1Driver>(robot);
    } else if (robot.ip.size() == 13){
        driver = std::make_unique<EPuckV2Driver>(robot);
    } else if (robot.ip == "127.0.0.1"){  
        driver = std::make_unique<EPuckVREPDriver>(robot);
    } else {
        return -1;
    }

    driver->init();
/*
    int argc = 0;
    while (argv[argc] != NULL) {
        argc++;
    }
    */
    std::string cont = argv[1];
    
    if (cont == "setWheelCmd") {
        if (argc != 5) {
            incorrectArguments(argc);
        } else {
            c = setWheelCmd;
            robot.wheels_command.left_velocity = std::stol(argv[3]);
            robot.wheels_command.right_velocity  = std::stol(argv[4]);
        }
    } else if (cont == "setVel") {
        if (argc != 5) {
            incorrectArguments(argc);
        } else {
            c = setVel;
            // vel --> robot().desired_velocity
            // omega --> robot().desired_angle
            robot.desired_velocity = std::stol(argv[3]);
            robot.desired_angle = std::stod(argv[4]);
        }
    } 
/*    else if (cont == "setRobVel") {
        if (argc != 5) {
            incorrectArguments(argc);
        } else {
            c = setRobVel;
            robot().desired_velocity = std::stod(argv[3]);
            robot().desired_angle = std::stod(argv[4]);
        }
    } else if (cont == "followWall") {
        if (argc != 3)
            incorrectArguments(argc);
        else
            c = followWall;
    } else if (cont == "visServo") {
        if (argc != 3)
            incorrectArguments(argc);
        else
            c = visServo;
    } else {
        incorrectArguments(argc);
    }
*/
    std::cout << "Controller is "<< argv[1] << "\n";
    
    std::signal(SIGINT, exitLoop);
    while (true) {
        driver->read();

        driver->getVisionSensor(robot);

        //control robot
        if (c == setWheelCmd) {
            // Euh , la fonction ici ne sers pas à grand chose
            setWheelCommands(robot); // set commmands to wheels
        } else if (c == setVel) {
            setVelocities(robot); // set operational velocities to robot to be done by students
        } 
    /*  else if (c == setRobVel) {
            setRobotVelocities(robot().parameters, startTime, robot().desired_velocity, robot().desired_angle, MotorCommand_); // send operational velocities to robot
        } else if (c == followWall) {
            controlRobotToFollowWall(startTime, (float)robot().desired_velocity, (float)robot().desired_angle); // make robot follow a wall using infrared measurements
            setRobotVelocities(robot().parameters, startTime, robot().desired_velocity, robot().desired_angle, MotorCommand_); // send operational velocities to robot
        } else if (c == visServo) {
            controlRobotWithVisualServoing(baryc, (float)robot().desired_velocity, (float)robot().desired_angle); // control robot using images from the camera
            setRobotVelocities(robot().parameters, startTime, robot().desired_velocity, robot().desired_angle, MotorCommand_); // send operational velocities to robot
        }
    */

        driver->sendCmd();

        if (signaled == true) {
            break;
        }
    }

}

void incorrectArguments(const int& argc) {
    std::cout << "There are "<< argc -1 << " arguments instead of 2 or 4\n";
    std::cout <<
        "The first argument should be one of the "
        "following:"
        "\n\tsetWheelCmd\n\tsetRobVel\n\tfollowWall\n\tvisServo\n";
    std::cout << "The following arguments should be:\n";
    std::cout << "\tsetWheelCmd: IP leftWheelCmd rightWheelCmd\n";
    std::cout << "\tsetVel: IP v(linear vel) w(angular vel)\n";
    std::cout << "\tsetRobVel: IP v(linear vel) w(angular vel)\n";
    std::cout << "\tfollowWall: IP\n";
    std::cout << "\tvisServo: IP\n";
    exit(0);
}

