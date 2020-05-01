#include "RobotDriver.hpp"

#include "extApiPlatform.h"
#include "extApi.h"

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <iostream>
#include <thread>
#include <chrono>

#define COLOR_COUT_RESET "\033[0m"

#define COLOR_COUT_BLACK "\033[0;30m"
#define COLOR_COUT_RED "\033[0;31m"
#define COLOR_COUT_GREEN "\033[0;32m"
#define COLOR_COUT_YELLOW "\033[0;33m"
#define COLOR_COUT_BLUE "\033[0;34m"
#define COLOR_COUT_MAGENTA "\033[0;35m"
#define COLOR_COUT_CYAN "\033[0;36m"
#define COLOR_COUT_WHITE "\033[0;37m"

#define COLOR_COUT_BLACK_BRIGHT "\033[1;30m"
#define COLOR_COUT_RED_BRIGHT "\033[1;31m"
#define COLOR_COUT_GREEN_BRIGHT "\033[1;32m"
#define COLOR_COUT_YELLOW_BRIGHT "\033[1;33m"
#define COLOR_COUT_BLUE_BRIGHT "\033[1;34m"
#define COLOR_COUT_MAGENTA_BRIGHT "\033[1;35m"
#define COLOR_COUT_CYAN_BRIGHT "\033[1;36m"
#define COLOR_COUT_WHITE_BRIGHT "\033[1;37m"

EPuckVREPDriver::EPuckVREPDriver(Robot& robot) : RobotDriver(robot), ajouter(1) {
}
 
void EPuckVREPDriver::init() {
    clientID = simxStart((simxChar*)"127.0.0.1", 19997, true, true, 2000, 5);
    std::cout << "clientID = " << clientID << std::endl;

    if (clientID == -1) {
        std::cout << ("Could not connect to V-REP remote API server") << std::endl;
        simxFinish(clientID);
    } else {

        std::cout << ("Connected to remote API server") << std::endl;

        simxSynchronous(clientID, true);
        simxStartSimulation(clientID, simx_opmode_oneshot);

        // Handles
        simxGetObjectHandle(clientID, "ePuck", &ePuckHandle, simx_opmode_blocking);

        simxGetObjectHandle(clientID, "ePuck_rightJoint", &rightJointHandle, simx_opmode_blocking);
        simxGetObjectHandle(clientID, "ePuck_leftJoint", &leftJointHandle, simx_opmode_blocking);

        simxGetObjectHandle(clientID, "ePuck_camera", &visionHandle, simx_opmode_blocking);

        simxGetObjectHandle(clientID, "ePuck_proxSensor0", &proxSensorsHandle[0], simx_opmode_blocking);
        simxGetObjectHandle(clientID, "ePuck_proxSensor1", &proxSensorsHandle[1], simx_opmode_blocking);
        simxGetObjectHandle(clientID, "ePuck_proxSensor2", &proxSensorsHandle[2], simx_opmode_blocking);
        simxGetObjectHandle(clientID, "ePuck_proxSensor3", &proxSensorsHandle[3], simx_opmode_blocking);
        simxGetObjectHandle(clientID, "ePuck_proxSensor4", &proxSensorsHandle[4], simx_opmode_blocking);
        simxGetObjectHandle(clientID, "ePuck_proxSensor5", &proxSensorsHandle[5], simx_opmode_blocking);
        simxGetObjectHandle(clientID, "ePuck_proxSensor6", &proxSensorsHandle[6], simx_opmode_blocking);
        simxGetObjectHandle(clientID, "ePuck_proxSensor7", &proxSensorsHandle[7], simx_opmode_blocking);

        simxReadProximitySensor(clientID, proxSensorsHandle[0],&detectionStateIR[0],detectedPointIR[0], NULL,NULL, simx_opmode_streaming);
        simxReadProximitySensor(clientID, proxSensorsHandle[1],&detectionStateIR[1],detectedPointIR[1], NULL,NULL, simx_opmode_streaming);
        simxReadProximitySensor(clientID, proxSensorsHandle[2],&detectionStateIR[2],detectedPointIR[2], NULL,NULL, simx_opmode_streaming);
        simxReadProximitySensor(clientID, proxSensorsHandle[3],&detectionStateIR[3],detectedPointIR[3], NULL,NULL, simx_opmode_streaming);
        simxReadProximitySensor(clientID, proxSensorsHandle[4],&detectionStateIR[4],detectedPointIR[4], NULL,NULL, simx_opmode_streaming);
        simxReadProximitySensor(clientID, proxSensorsHandle[5],&detectionStateIR[5],detectedPointIR[5], NULL,NULL, simx_opmode_streaming);
        simxReadProximitySensor(clientID, proxSensorsHandle[6],&detectionStateIR[6],detectedPointIR[6], NULL,NULL, simx_opmode_streaming);
        simxReadProximitySensor(clientID, proxSensorsHandle[7],&detectionStateIR[7],detectedPointIR[7], NULL,NULL, simx_opmode_streaming);

        simxGetObjectPosition(clientID, ePuckHandle, -1, ePuckPosition, simx_opmode_streaming);
        simxGetObjectOrientation(clientID, ePuckHandle, -1, eulerAngles, simx_opmode_streaming);

        simxGetVisionSensorImage(clientID, visionHandle, res, &simImage, 0, simx_opmode_streaming);

        simxGetJointPosition(clientID, leftJointHandle, &leftJointPosition, simx_opmode_streaming);
        simxGetJointPosition(clientID, leftJointHandle, &rightJointPosition, simx_opmode_streaming);

        simxGetObjectVelocity(clientID, leftJointHandle, NULL, leftJointVelocity, simx_opmode_streaming);
        simxGetObjectVelocity(clientID, rightJointHandle, NULL, rightJointVelocity, simx_opmode_streaming);

       

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}



void EPuckVREPDriver::read() {

    simxSynchronousTrigger(clientID);

    simxReadProximitySensor(clientID, proxSensorsHandle[0],&detectionStateIR[0],detectedPointIR[0],NULL, NULL, simx_opmode_buffer);
    simxReadProximitySensor(clientID, proxSensorsHandle[1],&detectionStateIR[1],detectedPointIR[1],NULL, NULL, simx_opmode_buffer);
    simxReadProximitySensor(clientID, proxSensorsHandle[2],&detectionStateIR[2],detectedPointIR[2],NULL, NULL, simx_opmode_buffer);
    simxReadProximitySensor(clientID, proxSensorsHandle[3],&detectionStateIR[3],detectedPointIR[3],NULL, NULL, simx_opmode_buffer);
    simxReadProximitySensor(clientID, proxSensorsHandle[4],&detectionStateIR[4],detectedPointIR[4],NULL, NULL, simx_opmode_buffer);
    simxReadProximitySensor(clientID, proxSensorsHandle[5],&detectionStateIR[5],detectedPointIR[5],NULL, NULL, simx_opmode_buffer);
    simxReadProximitySensor(clientID, proxSensorsHandle[6],&detectionStateIR[6],detectedPointIR[6],NULL, NULL, simx_opmode_buffer);
    simxReadProximitySensor(clientID, proxSensorsHandle[7],&detectionStateIR[7],detectedPointIR[7],NULL, NULL, simx_opmode_buffer);

    simxGetObjectPosition(clientID, ePuckHandle, -1, ePuckPosition, simx_opmode_buffer);
    simxGetObjectOrientation(clientID, ePuckHandle, -1, eulerAngles, simx_opmode_buffer);

    simxGetVisionSensorImage(clientID, visionHandle, res, &simImage, 0, simx_opmode_buffer);

    simxGetJointPosition(clientID, leftJointHandle, &leftJointPosition, simx_opmode_buffer);
    simxGetJointPosition(clientID, leftJointHandle, &rightJointPosition, simx_opmode_buffer);

    simxGetObjectVelocity(clientID, leftJointHandle, NULL, leftJointVelocity, simx_opmode_buffer);
    simxGetObjectVelocity(clientID, rightJointHandle, NULL, rightJointVelocity, simx_opmode_buffer);

    


    simxGetPingTime(clientID, &pingTime);


    PrintSensors();
}

void EPuckVREPDriver::send() {
    simxPauseCommunication(clientID, 1);
    simxSetJointTargetVelocity(clientID, rightJointHandle, robot().wheels_command.right_velocity, simx_opmode_oneshot);
    simxSetJointTargetVelocity(clientID, leftJointHandle, robot().wheels_command.left_velocity, simx_opmode_oneshot);
    simxPauseCommunication(clientID, 0);
}

void EPuckVREPDriver::getVisionSensor(Robot& robot) {
	cv::Mat img(res[0], res[1], CV_8UC3, (unsigned char*) simImage);
	cv::flip(img, robot.vision_sensors, 0);
    cv::cvtColor(robot.vision_sensors, robot.vision_sensors, CV_BGR2RGB);
	cv::namedWindow("Camera", CV_WINDOW_AUTOSIZE);
	cv::imshow("Camera", robot.vision_sensors);
	cv::waitKey(1);

	cv::imwrite(log().folder_ + "/image/" +  std::to_string(ajouter/2) + ".png", robot.vision_sensors);

	++ajouter;
}

EPuckVREPDriver::~EPuckVREPDriver() {
    simxStopSimulation(clientID, simx_opmode_blocking);
    simxFinish(clientID);
    std::cout << "End of the program" << std::endl;
}

void EPuckVREPDriver::PrintSensors() {
    dataToRobot();

    std::cout << "ePuck location : " << robot().current_pose.x << ", " << robot().current_pose.y << ", " << robot().current_pose.th << "\n"
              << "Position : " << robot().wheels_state.left_position << ", " << robot().wheels_state.right_position << "\n"
              << "Velocity : "  << robot().wheels_state.left_velocity << ", " << robot().wheels_state.right_velocity << std::endl;


    if(detectionStateIR[0]!=0) {
        std::cout << "P0 : " << robot().proximity_sensors.IR[0] << "\n";
    }     
    else{
         std::cout << COLOR_COUT_BLACK << "P0 : NONE" << COLOR_COUT_RESET << "\n";
    }
    
        
    if(detectionStateIR[1]!=0) {
        std::cout << "P1 : " << robot().proximity_sensors.IR[1] << "\n";
    }
    else{ 
        std::cout << COLOR_COUT_BLACK << "P1 : NONE" << COLOR_COUT_RESET << "\n";
    }
        
    if(detectionStateIR[2]!=0) {
        std::cout << "P2 : " << robot().proximity_sensors.IR[2] << "\n";
    }
    else{
        std::cout << COLOR_COUT_BLACK << "P2 : NONE" << COLOR_COUT_RESET << "\n";
    }

    if(detectionStateIR[3]!=0) {
        std::cout << "P3 : " << robot().proximity_sensors.IR[3] << "\n";
    }
    else{
        std::cout << COLOR_COUT_BLACK << "P3 : NONE" << COLOR_COUT_RESET << "\n";
    }

    if(detectionStateIR[4]!=0) {
        std::cout << "P4 : " << robot().proximity_sensors.IR[4] << "\n";
    }
    else{
        std::cout << COLOR_COUT_BLACK << "P4 : NONE" << COLOR_COUT_RESET << "\n";
    }
        
    if(detectionStateIR[5]!=0) {
        std::cout << "P5 : " << robot().proximity_sensors.IR[5] << "\n";
    }
    else{
        std::cout << COLOR_COUT_BLACK << "P5 : NONE" << COLOR_COUT_RESET << "\n";
    }
        
    if(detectionStateIR[6]!=0) {
        std::cout << "P6 : " << robot().proximity_sensors.IR[6] << "\n";
    }
    else{
        std::cout << COLOR_COUT_BLACK << "P6 : NONE" << COLOR_COUT_RESET << "\n";
    }
        
    if(detectionStateIR[7]!=0) {
        std::cout << "P7 : " << robot().proximity_sensors.IR[7] << "\n";
    }
    else{
        std::cout << COLOR_COUT_BLACK << "P7 : NONE" << COLOR_COUT_RESET << "\n";
    }
       
    log().addIn(log().file_ePuckPose[0], robot().current_pose.x);
    log().addIn(log().file_ePuckPose[1], robot().current_pose.y);
    log().addIn(log().file_ePuckPose[2], robot().current_pose.th);

    log().addIn(log().file_IR[0], robot().proximity_sensors.IR[0]);
    log().addIn(log().file_IR[1], robot().proximity_sensors.IR[1]);
    log().addIn(log().file_IR[2], robot().proximity_sensors.IR[2]);
    log().addIn(log().file_IR[3], robot().proximity_sensors.IR[3]);
    log().addIn(log().file_IR[4], robot().proximity_sensors.IR[4]);
    log().addIn(log().file_IR[5], robot().proximity_sensors.IR[5]);
    log().addIn(log().file_IR[6], robot().proximity_sensors.IR[6]);
    log().addIn(log().file_IR[7], robot().proximity_sensors.IR[7]);

    log().addIn(log().file_ePuckLeftWheelPosition, robot().wheels_state.left_position);
    log().addIn(log().file_ePuckRightWheelPosition, robot().wheels_state.right_position);

    log().addIn(log().file_ePuckLeftWheelVelocity, robot().wheels_state.left_velocity);
    log().addIn(log().file_ePuckRightWheelVelocity, robot().wheels_state.right_velocity);

}

void EPuckVREPDriver::dataToRobot() {
    for (int i = 0; i < 8; i++) {
        if(detectionStateIR[i]!=0){
            if (std::abs(detectedPointIR[i][2]) < 0.01 || std::abs(detectedPointIR[i][2]) > 1) {
                robot().proximity_sensors.IR[i] = 0;
                } else {
                    robot().proximity_sensors.IR[i] = std::abs(detectedPointIR[i][2]);
                }
        }
        else {
            robot().proximity_sensors.IR[i] = -1;
        }

    }

    robot().wheels_state.left_velocity = leftJointVelocity[0];
    robot().wheels_state.right_velocity= rightJointVelocity[0];

    robot().current_pose.x = ePuckPosition[0];
    robot().current_pose.y = ePuckPosition[1];
    robot().current_pose.th = eulerAngles[1];

    robot().wheels_state.left_position = leftJointPosition;
    robot().wheels_state.right_position = rightJointPosition;
    
}