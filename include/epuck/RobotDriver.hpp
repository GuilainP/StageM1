#pragma once

#include "Robot.hpp"
#include "datafile.hpp"

class RobotDriver {
public:
    RobotDriver(Robot& robot) : robot_(robot) {
    }
    virtual ~RobotDriver() = default;

    virtual bool Init() = 0;
    virtual void Read() = 0;
    virtual void Send() = 0;
    virtual void getVisionSensor(Robot& robot) = 0;

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

    bool Init() override;
    void Read() override;
    void Send() override;
    void getVisionSensor(Robot& robot) override ;

private:
    std::string robotIP;
    std::string robotID;
};

class EPuckV2Driver : public RobotDriver {
public:
    EPuckV2Driver(Robot& robot);
    virtual ~EPuckV2Driver();

    bool Init() override;
    void Read() override;
    void Send() override;
    void getVisionSensor(Robot& robot) override ;

private:
    std::string robotIP;
    std::string robotID;
};

class EPuckVREPDriver : public RobotDriver {
public:
    EPuckVREPDriver(Robot& robot);
    virtual ~EPuckVREPDriver();

    bool Init() override;
    void Read() override;
    void Send() override;
    void getVisionSensor(Robot& robot) override ;

    void setVelocity(double, double);
    void PrintSensors();
    void dataToRobot();
    

private:
    int clientID, pingTime;
    int ePuckHandle;
    int rightJointHandle, leftJointHandle;
    int proxSensorsHandle[8];
    int visionHandle;
    int res[2];
    float ePuckPosition[3];
    float leftJointPosition, rightJointPosition;
    float leftJointVelocity[3], rightJointVelocity[3];
    float eulerAngles[3];
    uint8_t detectionStateIR[8];
    float detectedPointIR[8][3];
    uint8_t* simImage;
    int ajouter;

    double minl,maxl,minr,maxr;
};
