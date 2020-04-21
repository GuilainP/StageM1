#include "RobotDriver.hpp"

#include "extApiPlatform.h"
#include "extApiPlatform.c"
#include "extApi.h"
#include "extApi.c"

#define TO_MS 1000

void PrintSensors();
void sensorsRange(float* sensor);

int clientID(0),pingTime(0);
int proxSensorsHandle[8];
int ePuckHandle,sphereHandle;
float ePuckPosition[3];
float IR0[3],IR1[3],IR2[3],IR3[3],IR4[3],IR5[3],IR6[3],IR7[3];
int rightJointHandle(0), leftJointHandle(0);
float rightSpeed, leftSpeed;

//simxFloat leftSpeed(0.0),rightSpeed(0.0);
Logger fileToWrite;

void EPuckVREPDriver::init(){

	std::cout << "Target Velocity : \n " << "Left Speed : " ;
	std::cin >> leftSpeed ;
	std::cout << "Right Speed : ";
	std::cin >> rightSpeed;
	std::cout<<std::endl;

    clientID = simxStart((simxChar*)"127.0.0.1",19997,true,true,2000,5);
	std::cout<<"clientID = "<<clientID<<std::endl;

	if (clientID==-1){
		std::cout<<("Could not connect to V-REP remote API server")<<std::endl;
		simxFinish(clientID);
	}else{
		
		std::cout<<("Connected to remote API server")<<std::endl;

		simxStartSimulation(clientID,simx_opmode_oneshot);

		//Handles
		simxGetObjectHandle(clientID, "ePuck", &ePuckHandle, simx_opmode_blocking );
		simxGetObjectHandle(clientID, "Sphere", &sphereHandle, simx_opmode_blocking );

		simxGetObjectHandle(clientID, "ePuck_rightJoint", &rightJointHandle, simx_opmode_blocking );
		simxGetObjectHandle(clientID, "ePuck_leftJoint", &leftJointHandle, simx_opmode_blocking );

		simxGetObjectHandle(clientID, "ePuck_proxSensor0", &proxSensorsHandle[0], simx_opmode_blocking );
		simxGetObjectHandle(clientID, "ePuck_proxSensor1", &proxSensorsHandle[1], simx_opmode_blocking );
		simxGetObjectHandle(clientID, "ePuck_proxSensor2", &proxSensorsHandle[2], simx_opmode_blocking );
		simxGetObjectHandle(clientID, "ePuck_proxSensor3", &proxSensorsHandle[3], simx_opmode_blocking );
		simxGetObjectHandle(clientID, "ePuck_proxSensor4", &proxSensorsHandle[4], simx_opmode_blocking );
		simxGetObjectHandle(clientID, "ePuck_proxSensor5", &proxSensorsHandle[5], simx_opmode_blocking );
		simxGetObjectHandle(clientID, "ePuck_proxSensor6", &proxSensorsHandle[6], simx_opmode_blocking );
		simxGetObjectHandle(clientID, "ePuck_proxSensor7", &proxSensorsHandle[7], simx_opmode_blocking );



		simxReadProximitySensor(clientID,proxSensorsHandle[0],NULL,NULL,NULL,IR0,simx_opmode_streaming);
		simxReadProximitySensor(clientID,proxSensorsHandle[1],NULL,NULL,NULL,IR1,simx_opmode_streaming);
		simxReadProximitySensor(clientID,proxSensorsHandle[2],NULL,NULL,NULL,IR2,simx_opmode_streaming);
		simxReadProximitySensor(clientID,proxSensorsHandle[3],NULL,NULL,NULL,IR3,simx_opmode_streaming);
		simxReadProximitySensor(clientID,proxSensorsHandle[4],NULL,NULL,NULL,IR4,simx_opmode_streaming);
		simxReadProximitySensor(clientID,proxSensorsHandle[5],NULL,NULL,NULL,IR5,simx_opmode_streaming);
		simxReadProximitySensor(clientID,proxSensorsHandle[6],NULL,NULL,NULL,IR6,simx_opmode_streaming);
		simxReadProximitySensor(clientID,proxSensorsHandle[7],NULL,NULL,NULL,IR7,simx_opmode_streaming);

		simxGetObjectPosition(clientID,ePuckHandle,sphereHandle,ePuckPosition,simx_opmode_streaming);

		simxSetJointTargetVelocity(clientID,rightJointHandle,rightSpeed, simx_opmode_oneshot);
		simxSetJointTargetVelocity(clientID,leftJointHandle,leftSpeed, simx_opmode_oneshot);
    }
}

void EPuckVREPDriver::read(){


	simxSetJointTargetVelocity(clientID,rightJointHandle,rightSpeed, simx_opmode_oneshot);
	simxSetJointTargetVelocity(clientID,leftJointHandle,leftSpeed, simx_opmode_oneshot);

	simxSynchronousTrigger(clientID);

	simxReadProximitySensor(clientID,proxSensorsHandle[0],NULL,NULL,NULL,IR0,simx_opmode_buffer);
	simxReadProximitySensor(clientID,proxSensorsHandle[1],NULL,NULL,NULL,IR1,simx_opmode_buffer);
	simxReadProximitySensor(clientID,proxSensorsHandle[2],NULL,NULL,NULL,IR2,simx_opmode_buffer);
	simxReadProximitySensor(clientID,proxSensorsHandle[3],NULL,NULL,NULL,IR3,simx_opmode_buffer);
	simxReadProximitySensor(clientID,proxSensorsHandle[4],NULL,NULL,NULL,IR4,simx_opmode_buffer);
	simxReadProximitySensor(clientID,proxSensorsHandle[5],NULL,NULL,NULL,IR5,simx_opmode_buffer);
	simxReadProximitySensor(clientID,proxSensorsHandle[6],NULL,NULL,NULL,IR6,simx_opmode_buffer);
	simxReadProximitySensor(clientID,proxSensorsHandle[7],NULL,NULL,NULL,IR7,simx_opmode_buffer);

	simxGetObjectPosition(clientID,ePuckHandle,sphereHandle,ePuckPosition,simx_opmode_buffer);
	
	simxGetPingTime(clientID,&pingTime);
	std::cout<< "\nPing = " << pingTime << " ;" << std::endl;

	
	fileToWrite.addIn(ePuckPosition[0],ePuckPosition[1]);
	std::cout << "ePuck location : " << ePuckPosition[0] << ", " << ePuckPosition[1] << std::endl;
	PrintSensors();
	
}

void EPuckVREPDriver::send(){

	simxStopSimulation(clientID,simx_opmode_oneshot);
	simxFinish(clientID);
	std::cout<<"End of the program"<<std::endl;
}

void PrintSensors(){

	sensorsRange(IR0);
	sensorsRange(IR1);
	sensorsRange(IR2);
	sensorsRange(IR3);
	sensorsRange(IR4);
	sensorsRange(IR5);
	sensorsRange(IR6);
	sensorsRange(IR7);

	std::cout<<std::endl;
	std::cout << "P0 : " << IR0[2] << "\n";
	std::cout << "P1 : " << IR1[2] << "\n";
	std::cout << "P2 : " << IR2[2] << "\n";
	std::cout << "P3 : " << IR3[2] << "\n";
	std::cout << "P4 : " << IR4[2] << "\n";
	std::cout << "P5 : " << IR5[2] << "\n";
	std::cout << "P6 : " << IR6[2] << "\n";
	std::cout << "P7 : " << IR7[2] << "\n"; 
}

void sensorsRange(float* sensor){
	if( std::abs(sensor[2]) < 0.01 ||  std::abs(sensor[3]) > 1){
		sensor[2] = 0;
	}
	else{
		sensor[2] = std::abs(sensor[2]);
	}
}