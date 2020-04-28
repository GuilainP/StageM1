#pragma once

#include "Robot.hpp"
#include "datafile.hpp"

class RobotDriver {
public:
    RobotDriver(Robot& robot) : robot_(robot) {
    }
    virtual ~RobotDriver() = default;

    virtual void init() = 0;
    virtual void read() = 0;
    virtual void send() = 0;

    Robot& robot();
    Logger& log();

private:
    Robot& robot_;
    Logger log_;
};

class EPuckV1Driver : public RobotDriver {
public:
    EPuckV1Driver(Robot& robot);
    virtual ~EPuckV1Driver() = default;

    void init() override;
    void read() override;
    void send() override;

private:
    std::string IP;
    std::string port;
};

class EPuckV2Driver : public RobotDriver {
public:
    EPuckV2Driver(Robot& robot);
    virtual ~EPuckV2Driver() = default;

    void init() override;
    void read() override;
    void send() override;

private:
    std::string IP;
    std::string port;
};

class EPuckVREPDriver : public RobotDriver {
public:
    EPuckVREPDriver(Robot& robot);
    virtual ~EPuckVREPDriver();

    void init() override;
    void read() override;
    void send() override;

    void setVelocity(double, double);
    void PrintSensors();
    void dataToRobot();

private:
    int clientID, pingTime;
    int ePuckHandle, sphereHandle;
    int rightJointHandle, leftJointHandle;
    int proxSensorsHandle[8];
    int visionHandle;
    int res[2];
    float ePuckPosition[3];
    float simIR[8][3];

    int handleCounts;

    uint8_t* simImage;
    int ajouter;
};
