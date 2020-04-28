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
        simxGetObjectHandle(clientID, "Sphere", &sphereHandle, simx_opmode_blocking);

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

        simxReadProximitySensor(clientID, proxSensorsHandle[0],&detectionState[0],detectedPoint[0], NULL, simIR[0], simx_opmode_streaming);
        simxReadProximitySensor(clientID, proxSensorsHandle[1],&detectionState[1],detectedPoint[1], NULL, simIR[1], simx_opmode_streaming);
        simxReadProximitySensor(clientID, proxSensorsHandle[2],&detectionState[2],detectedPoint[2], NULL, simIR[2], simx_opmode_streaming);
        simxReadProximitySensor(clientID, proxSensorsHandle[3],&detectionState[3],detectedPoint[3], NULL, simIR[3], simx_opmode_streaming);
        simxReadProximitySensor(clientID, proxSensorsHandle[4],&detectionState[4],detectedPoint[4], NULL, simIR[4], simx_opmode_streaming);
        simxReadProximitySensor(clientID, proxSensorsHandle[5],&detectionState[5],detectedPoint[5], NULL, simIR[5], simx_opmode_streaming);
        simxReadProximitySensor(clientID, proxSensorsHandle[6],&detectionState[6],detectedPoint[6], NULL, simIR[6], simx_opmode_streaming);
        simxReadProximitySensor(clientID, proxSensorsHandle[7],&detectionState[7],detectedPoint[7], NULL, simIR[7], simx_opmode_streaming);

        simxGetObjectPosition(clientID, ePuckHandle, sphereHandle, ePuckPosition, simx_opmode_streaming);
        simxGetVisionSensorImage(clientID, visionHandle, res, &simImage, 0, simx_opmode_streaming);

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}



void EPuckVREPDriver::read() {

    simxSynchronousTrigger(clientID);

    simxReadProximitySensor(clientID, proxSensorsHandle[0],&detectionState[0],detectedPoint[0],NULL, simIR[0], simx_opmode_buffer);
    simxReadProximitySensor(clientID, proxSensorsHandle[1],&detectionState[1],detectedPoint[1],NULL, simIR[1], simx_opmode_buffer);
    simxReadProximitySensor(clientID, proxSensorsHandle[2],&detectionState[2],detectedPoint[2],NULL, simIR[2], simx_opmode_buffer);
    simxReadProximitySensor(clientID, proxSensorsHandle[3],&detectionState[3],detectedPoint[3],NULL, simIR[3], simx_opmode_buffer);
    simxReadProximitySensor(clientID, proxSensorsHandle[4],&detectionState[4],detectedPoint[4],NULL, simIR[4], simx_opmode_buffer);
    simxReadProximitySensor(clientID, proxSensorsHandle[5],&detectionState[5],detectedPoint[5],NULL, simIR[5], simx_opmode_buffer);
    simxReadProximitySensor(clientID, proxSensorsHandle[6],&detectionState[6],detectedPoint[6],NULL, simIR[6], simx_opmode_buffer);
    simxReadProximitySensor(clientID, proxSensorsHandle[7],&detectionState[7],detectedPoint[7],NULL, simIR[7], simx_opmode_buffer);

    simxGetObjectPosition(clientID, ePuckHandle, sphereHandle, ePuckPosition, simx_opmode_buffer);

    simxGetVisionSensorImage(clientID, visionHandle, res, &simImage, 0, simx_opmode_buffer);

    simxGetPingTime(clientID, &pingTime);


    PrintSensors();
}

void EPuckVREPDriver::send() {
    simxPauseCommunication(clientID, 1);
    simxSetJointTargetVelocity(clientID, rightJointHandle, robot().wheels_command.right_velocity, simx_opmode_oneshot);
    simxSetJointTargetVelocity(clientID, leftJointHandle, robot().wheels_command.left_velocity, simx_opmode_oneshot);
    simxPauseCommunication(clientID, 0);
}

void EPuckVREPDriver::getImage(Robot& robot) {
	cv::Mat img(res[0], res[1], CV_8UC3, (unsigned char*) simImage);
	cv::flip(img, robot.vision_sensors, 0);
    cv::cvtColor(robot.vision_sensors, robot.vision_sensors, CV_BGR2RGB);
	cv::namedWindow("Camera", CV_WINDOW_AUTOSIZE);
	cv::imshow("Camera", robot.vision_sensors);
	cv::waitKey(1);

	if(ajouter%2 == 0){
		cv::imwrite(log().folder_ + "/image/" +  std::to_string(ajouter/2) + ".jpg", robot.vision_sensors);

	}
	++ajouter;
}

EPuckVREPDriver::~EPuckVREPDriver() {
    simxStopSimulation(clientID, simx_opmode_blocking);
    simxFinish(clientID);
    std::cout << "End of the program" << std::endl;
}

void EPuckVREPDriver::PrintSensors() {
    dataToRobot();

    std::cout << "ePuck location : " << robot().current_pose.x << ", " << robot().current_pose.y << std::endl;
    std::cout << std::endl;

    if(detectionState[0]!=0)
        std::cout << "P0 : " << robot().proximity_sensors.IR[0] << " | Point : " << detectedPoint[0][2] <<"\n";
        
    if(detectionState[1]!=0)
        std::cout << "P1 : " << robot().proximity_sensors.IR[1] << " | Point : " << detectedPoint[1][2] <<"\n";
    if(detectionState[2]!=0)
        std::cout << "P2 : " << robot().proximity_sensors.IR[2] << " | Point : " << detectedPoint[2][2] <<"\n";
    if(detectionState[3]!=0)
        std::cout << "P3 : " << robot().proximity_sensors.IR[3] << " | Point : " << detectedPoint[3][2] <<"\n";
    if(detectionState[4]!=0)
        std::cout << "P4 : " << robot().proximity_sensors.IR[4] << " | Point : " << detectedPoint[4][2] <<"\n";
    if(detectionState[5]!=0)
        std::cout << "P5 : " << robot().proximity_sensors.IR[5] << " | Point : " << detectedPoint[5][2] <<"\n";
    if(detectionState[6]!=0)
        std::cout << "P6 : " << robot().proximity_sensors.IR[6] << " | Point : " << detectedPoint[6][2] <<"\n";
    if(detectionState[7]!=0)
        std::cout << "P7 : " << robot().proximity_sensors.IR[7] << " | Point : " << detectedPoint[7][2] <<"\n";

    log().addIn(log().file_ePuckPosition[0], ePuckPosition[0]);
    log().addIn(log().file_ePuckPosition[1], ePuckPosition[1]);

    log().addIn(log().file_IR[0], robot().proximity_sensors.IR[0]);
    log().addIn(log().file_IR[1], robot().proximity_sensors.IR[1]);
    log().addIn(log().file_IR[2], robot().proximity_sensors.IR[2]);
    log().addIn(log().file_IR[3], robot().proximity_sensors.IR[3]);
    log().addIn(log().file_IR[4], robot().proximity_sensors.IR[4]);
    log().addIn(log().file_IR[5], robot().proximity_sensors.IR[5]);
    log().addIn(log().file_IR[6], robot().proximity_sensors.IR[6]);
    log().addIn(log().file_IR[7], robot().proximity_sensors.IR[7]);
}

void EPuckVREPDriver::dataToRobot() {
    for (int i = 0; i < 8; i++) {
        if (std::abs(simIR[i][2]) < 0.01 || std::abs(simIR[i][2]) > 1) {
            robot().proximity_sensors.IR[i] = 0;
        } else {
            robot().proximity_sensors.IR[i] = std::abs(simIR[i][2]);
        }
    }
    robot().current_pose.x = ePuckPosition[0];
    robot().current_pose.y = ePuckPosition[1];
}
