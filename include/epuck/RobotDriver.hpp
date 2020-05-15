#pragma once

#include "Robot.hpp"
#include "datafile.hpp"

#include <sstream>
#include <math.h>
#include <time.h>
#include <sys/time.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <opencv2/opencv.hpp>

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
    void PrintSensors();
    void closeConnection();

private:
    std::string robotIP;
    std::string robotID;

    unsigned char image[160*120*2];
    int ajouter;

    // Communication variables
    struct sockaddr_in robot_addr;
    int fd;
    unsigned char command[21];
    unsigned char header, sensor[104];
    int bytes_sent, bytes_recv ;
    bool camera_enabled, ground_sensors_enabled;
    uint8_t expected_recv_packets;
    bool newImageReceived;

    // Sensors data variables

    float acceleration, orientation, inclination;		/**< acceleration data*/
    int16_t accData[3];
    int16_t gyroRaw[3];
    float magneticField[3];
    uint8_t temperature;
    int proxData[8]; /**< proximity sensors data*/
    int lightAvg;										/**< light sensor data*/
    uint16_t distanceCm;
    uint16_t micVolume[4];								/**< microphone data*/
    int16_t motorSteps[2];
    uint16_t batteryRaw;
    uint8_t microSdState;
    uint8_t irCheck, irAddress, irData;
    uint8_t selector;
    int16_t groundProx[3], groundAmbient[3];
    uint8_t buttonState;

    double leftStepsDiff, rightStepsDiff;
    double leftStepsPrev, rightStepsPrev;
    signed long int leftStepsRawPrev, rightStepsRawPrev;
    signed long int motorPositionDataCorrect[2];
    double xPos, yPos, theta;
    double deltaSteps, deltaTheta;

    int overflowCountLeft, overflowCountRight ;
    int16_t gyroOffset[3] ; // Used if making an initial calibration of the gyro.
    int speedLeft, speedRight;

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
