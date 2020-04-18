#include "RobotDriver.hpp"

#include "extApiPlatform.h"
#include "extApiPlatform.c"
#include "extApi.h"
#include "extApi.c"

int clientID;


simxInt proxSensors[8];
simxFloat detectedPoint[8][3];
simxInt rightJoint;
simxInt leftJoint;

//simxFloat leftSpeed(0.0),rightSpeed(0.0);

void EPuckVREPDriver::init(){
    clientID = simxStart((simxChar*)"127.0.0.1",19997,true,true,2000,5);
	std::cout<<"clientID = "<<clientID<<std::endl;

	if (clientID==-1){
		std::cout<<("Could not connect to V-REP remote API server")<<std::endl;
		simxFinish(clientID);
	}else{

		std::cout<<("Connected to remote API server")<<std::endl;
		
		simxStartSimulation(clientID,simx_opmode_oneshot);

		//Handles
		simxGetObjectHandle(clientID, "ePuck_rightJoint", &rightJoint, simx_opmode_oneshot_wait);
		simxGetObjectHandle(clientID, "ePuck_leftJoint", &leftJoint, simx_opmode_oneshot_wait);

		simxGetObjectHandle(clientID, "ePuck_proxSensor0", &proxSensors[0], simx_opmode_oneshot_wait);
		simxGetObjectHandle(clientID, "ePuck_proxSensor1", &proxSensors[1], simx_opmode_oneshot_wait);
		simxGetObjectHandle(clientID, "ePuck_proxSensor2", &proxSensors[2], simx_opmode_oneshot_wait);
		simxGetObjectHandle(clientID, "ePuck_proxSensor3", &proxSensors[3], simx_opmode_oneshot_wait);
		simxGetObjectHandle(clientID, "ePuck_proxSensor4", &proxSensors[4], simx_opmode_oneshot_wait);
		simxGetObjectHandle(clientID, "ePuck_proxSensor5", &proxSensors[5], simx_opmode_oneshot_wait);
		simxGetObjectHandle(clientID, "ePuck_proxSensor6", &proxSensors[6], simx_opmode_oneshot_wait);
		simxGetObjectHandle(clientID, "ePuck_proxSensor7", &proxSensors[7], simx_opmode_oneshot_wait);


		simxReadProximitySensor(clientID,proxSensors[0],NULL,NULL,NULL,detectedPoint[0],simx_opmode_streaming);
		simxReadProximitySensor(clientID,proxSensors[1],NULL,NULL,NULL,detectedPoint[1],simx_opmode_streaming);
		simxReadProximitySensor(clientID,proxSensors[2],NULL,NULL,NULL,detectedPoint[2],simx_opmode_streaming);
		simxReadProximitySensor(clientID,proxSensors[3],NULL,NULL,NULL,detectedPoint[3],simx_opmode_streaming);
		simxReadProximitySensor(clientID,proxSensors[4],NULL,NULL,NULL,detectedPoint[4],simx_opmode_streaming);
		simxReadProximitySensor(clientID,proxSensors[5],NULL,NULL,NULL,detectedPoint[5],simx_opmode_streaming);
		simxReadProximitySensor(clientID,proxSensors[6],NULL,NULL,NULL,detectedPoint[6],simx_opmode_streaming);
		simxReadProximitySensor(clientID,proxSensors[7],NULL,NULL,NULL,detectedPoint[7],simx_opmode_streaming);

        std::cout << "Initialisation Completed!\n";

		simxFloat lSpeed(1),rSpeed(1);
    	simxSetJointTargetVelocity(clientID, rightJoint, lSpeed, simx_opmode_oneshot_wait);
    	simxSetJointTargetVelocity(clientID,  leftJoint, rSpeed, simx_opmode_oneshot_wait);

    }
}

void EPuckVREPDriver::read(){
	simxReadProximitySensor(clientID,proxSensors[0],NULL,NULL,NULL,detectedPoint[0],simx_opmode_buffer);
	simxReadProximitySensor(clientID,proxSensors[1],NULL,NULL,NULL,detectedPoint[1],simx_opmode_buffer);
	simxReadProximitySensor(clientID,proxSensors[2],NULL,NULL,NULL,detectedPoint[2],simx_opmode_buffer);
	simxReadProximitySensor(clientID,proxSensors[3],NULL,NULL,NULL,detectedPoint[3],simx_opmode_buffer);
	simxReadProximitySensor(clientID,proxSensors[4],NULL,NULL,NULL,detectedPoint[4],simx_opmode_buffer);
	simxReadProximitySensor(clientID,proxSensors[5],NULL,NULL,NULL,detectedPoint[5],simx_opmode_buffer);
	simxReadProximitySensor(clientID,proxSensors[6],NULL,NULL,NULL,detectedPoint[6],simx_opmode_buffer);
	simxReadProximitySensor(clientID,proxSensors[7],NULL,NULL,NULL,detectedPoint[7],simx_opmode_buffer);

	std::cout << "-Proximity Sensors Normal Values-" << std::endl;
	std::cout 	<< "P0 : " << abs(detectedPoint[0][2]) << "\n"
				<< "P1 : " << abs(detectedPoint[1][2]) << "\n"
	 			<< "P2 : " << abs(detectedPoint[2][2]) << "\n"
	 			<< "P3 : " << abs(detectedPoint[3][2]) << "\n"
	 			<< "P4 : " << abs(detectedPoint[4][2]) << "\n"
	 			<< "P5 : " << abs(detectedPoint[5][2]) << "\n"
				<< "P6 : " << abs(detectedPoint[6][2]) << "\n"
	 		    << "P7 : " << abs(detectedPoint[7][2]) << "\n\n\n";

	usleep(100000);
}

void EPuckVREPDriver::send(){
	simxStopSimulation(clientID,simx_opmode_oneshot);
	simxFinish(clientID);
	std::cout<<"End of the program"<<std::endl;
}