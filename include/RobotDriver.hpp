#pragma once

#include "Robot.hpp"
#include "datafile.hpp"

#include "extApiPlatform.h"
#include "extApi.h"
#include "simConst.h"

class RobotDriver{
    public:
        RobotDriver(Robot& robot) : robot_(robot) {}
        virtual ~RobotDriver() = default;

        virtual void init() = 0;
        virtual void read() = 0;
        //virtual void moveRobot(Robot) = 0;
        virtual void send() = 0;
        Robot& robot_;
        Logger log_;
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
        void send() override { std::cout << "send V2\n";};// TODO}

        std::string IP;
        std::string port;
};

class EPuckVREPDriver : public RobotDriver {
    public:
        EPuckVREPDriver(Robot& robot) 
            : RobotDriver(robot), ajouter(1) {}
        virtual ~EPuckVREPDriver();

        void init() override ;
        void read() override ;
        void send() override ;

        void setVelocity(double, double);
        void PrintSensors();
        void dataToRobot();

    private:
        int clientID, pingTime;
        int ePuckHandle,sphereHandle;
        int rightJointHandle, leftJointHandle;
        int proxSensorsHandle[8];
        int visionHandle;int res[2];
        float ePuckPosition[3];
        float simIR[8][3];

        int handleCounts;

        simxUChar* simImage;
        int ajouter;

};
