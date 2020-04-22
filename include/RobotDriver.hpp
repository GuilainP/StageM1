#pragma once

#include "Robot.hpp"
#include "datafile.hpp"

#include "sim/extApiPlatform.h"
#include "sim/extApi.h"
#include "sim/simConst.h"

class RobotDriver{
    public:
        RobotDriver(Robot& robot) : robot_(robot) {}
        virtual ~RobotDriver() = default;

        virtual void init() = 0;
        virtual void read() = 0;
        //virtual void moveRobot(Robot) = 0;
        virtual void send() = 0;
        Robot& robot_;
};

class EPuckV1Driver : public RobotDriver {
    public:
        EPuckV1Driver(Robot& robot) : RobotDriver(robot) {}
        virtual ~EPuckV1Driver() = default;

        void init() override { std::cout << "init V1\n";};// TODO}
        void read() override { std::cout << "read V1\n";};// TODO}
        void send() override { std::cout << "send V1\n";};// TODO}
        std::string IP;
        std::string port;
};

class EPuckV2Driver : public RobotDriver {
    public:
        EPuckV2Driver(Robot& robot) : RobotDriver(robot) {}
        virtual ~EPuckV2Driver() = default;

        void init() override { std::cout << "init V2\n";};// TODO}
        void read() override { std::cout << "read V2\n";};// TODO}
        void send() override { std::cout << "sen V2\n";};// TODO}

        std::string IP;
        std::string port;
};

class EPuckVREPDriver : public RobotDriver {
    public:
        EPuckVREPDriver(Robot& robot) 
            : RobotDriver(robot) {}
        virtual ~EPuckVREPDriver();

        void init() override ;
        void read() override ;
        void send() override ;

        void setVelocity(double, double);
        void PrintSensors();
        void dataToRobot();

        std::string Portname;
        int clientID, pingTime;
        int ePuckHandle,sphereHandle;
        int rightJointHandle, leftJointHandle;
        float ePuckPosition[3];
        int proxSensorsHandle[8];
        float simIR[8][3];
};
