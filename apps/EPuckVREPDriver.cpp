#include "RobotDriver.hpp"

#include "extApiPlatform.h"
#include "extApiPlatform.c"
#include "extApi.h"
#include "extApi.c"

#define PI 3.14159
#define TO_MS 1000

void PrintSensors();

int clientID(0),pingTime(0);
int proxSensorsHandle[8];
int ePuckHandle,sphereHandle;
float ePuckPosition[3];
float detectedSurfaceNormalVector[8][3];

int rightJointHandle(0), leftJointHandle(0);
float rightSpeed, leftSpeed;

std::ofstream fileToWrite;
std::string filedatename;

//simxFloat leftSpeed(0.0),rightSpeed(0.0);

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
		filedatename = fileName();
		

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



		simxReadProximitySensor(clientID,proxSensorsHandle[0],NULL,NULL,NULL,detectedSurfaceNormalVector[0],simx_opmode_blocking);
		simxReadProximitySensor(clientID,proxSensorsHandle[1],NULL,NULL,NULL,detectedSurfaceNormalVector[1],simx_opmode_blocking);
		simxReadProximitySensor(clientID,proxSensorsHandle[2],NULL,NULL,NULL,detectedSurfaceNormalVector[2],simx_opmode_blocking);
		simxReadProximitySensor(clientID,proxSensorsHandle[3],NULL,NULL,NULL,detectedSurfaceNormalVector[3],simx_opmode_blocking);
		simxReadProximitySensor(clientID,proxSensorsHandle[4],NULL,NULL,NULL,detectedSurfaceNormalVector[4],simx_opmode_blocking);
		simxReadProximitySensor(clientID,proxSensorsHandle[5],NULL,NULL,NULL,detectedSurfaceNormalVector[5],simx_opmode_blocking);
		simxReadProximitySensor(clientID,proxSensorsHandle[6],NULL,NULL,NULL,detectedSurfaceNormalVector[6],simx_opmode_blocking);
		simxReadProximitySensor(clientID,proxSensorsHandle[7],NULL,NULL,NULL,detectedSurfaceNormalVector[7],simx_opmode_blocking);

		simxSetJointTargetVelocity(clientID,rightJointHandle,rightSpeed, simx_opmode_oneshot);
		simxSetJointTargetVelocity(clientID,leftJointHandle,leftSpeed, simx_opmode_oneshot);
    }
}

void EPuckVREPDriver::read(){
	fileToWrite.open(filedatename, std::ios::app);
	
	simxGetPingTime(clientID,&pingTime);
	std::cout<< "\n Ping = " << pingTime << " ;" << std::endl;

	simxPauseCommunication(clientID,1);

	simxSetJointTargetVelocity(clientID,rightJointHandle,rightSpeed, simx_opmode_oneshot);
	simxSetJointTargetVelocity(clientID,leftJointHandle,leftSpeed, simx_opmode_oneshot);

	simxPauseCommunication(clientID,0);	

	simxReadProximitySensor(clientID,proxSensorsHandle[0],NULL,NULL,NULL,detectedSurfaceNormalVector[0],simx_opmode_blocking);
	simxReadProximitySensor(clientID,proxSensorsHandle[1],NULL,NULL,NULL,detectedSurfaceNormalVector[1],simx_opmode_blocking);
	simxReadProximitySensor(clientID,proxSensorsHandle[2],NULL,NULL,NULL,detectedSurfaceNormalVector[2],simx_opmode_blocking);
	simxReadProximitySensor(clientID,proxSensorsHandle[3],NULL,NULL,NULL,detectedSurfaceNormalVector[3],simx_opmode_blocking);
	simxReadProximitySensor(clientID,proxSensorsHandle[4],NULL,NULL,NULL,detectedSurfaceNormalVector[4],simx_opmode_blocking);
	simxReadProximitySensor(clientID,proxSensorsHandle[5],NULL,NULL,NULL,detectedSurfaceNormalVector[5],simx_opmode_blocking);
	simxReadProximitySensor(clientID,proxSensorsHandle[6],NULL,NULL,NULL,detectedSurfaceNormalVector[6],simx_opmode_blocking);
	simxReadProximitySensor(clientID,proxSensorsHandle[7],NULL,NULL,NULL,detectedSurfaceNormalVector[7],simx_opmode_blocking);
	simxGetObjectPosition(clientID,ePuckHandle,sphereHandle,ePuckPosition,simx_opmode_blocking);

	if(fileToWrite.is_open()){
		fileToWrite << ePuckPosition[0] << ", " << ePuckPosition[1] << "\n";
		
	}
			
	std::cout << "ePuck location : " << ePuckPosition[0] << ", " << ePuckPosition[1] << std::endl;
	PrintSensors();
	fileToWrite.close();
}

void EPuckVREPDriver::send(){
	
	simxStopSimulation(clientID,simx_opmode_oneshot);
	simxFinish(clientID);
	std::cout<<"End of the program"<<std::endl;
}

void PrintSensors(){
	for(int i = 0 ; i < 8 ;i++  )
	{
		if( std::abs(detectedSurfaceNormalVector[i][2]) < 0.01 ||  std::abs(detectedSurfaceNormalVector[i][2]) > 1){
			detectedSurfaceNormalVector[i][2] = 0;
		}
		else{
			detectedSurfaceNormalVector[i][2] = std::abs(detectedSurfaceNormalVector[i][2]);
		}
	}
	std::cout<<std::endl;
	std::cout << "P0 : " << detectedSurfaceNormalVector[0][2] << "\n";
	std::cout << "P1 : " << detectedSurfaceNormalVector[1][2] << "\n";
	std::cout << "P2 : " << detectedSurfaceNormalVector[2][2] << "\n";
	std::cout << "P3 : " << detectedSurfaceNormalVector[3][2] << "\n";
	std::cout << "P4 : " << detectedSurfaceNormalVector[4][2] << "\n";
	std::cout << "P5 : " << detectedSurfaceNormalVector[5][2] << "\n";
	std::cout << "P6 : " << detectedSurfaceNormalVector[6][2] << "\n";
	std::cout << "P7 : " << detectedSurfaceNormalVector[7][2] << "\n"; 
}

