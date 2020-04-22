#include "RobotDriver.hpp"

#include "extApiPlatform.h"
#include "extApiPlatform.c"
#include "extApi.h"
#include "extApi.c"

#define TO_MS 1000

Logger fileToWrite;

void EPuckVREPDriver::init(){

	std::cout << "Target Velocity : \n " << "Left Speed : " ;
	std::cin >> robot_.wheels_command.left_velocity ;
	std::cout << "Right Speed : ";
	std::cin >> robot_.wheels_command.right_velocity;
	std::cout<<std::endl;

    clientID = simxStart((simxChar*)"127.0.0.1",19997,true,true,2000,5);
	std::cout<<"clientID = "<< clientID <<std::endl;

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



		simxReadProximitySensor(clientID,proxSensorsHandle[0],NULL,NULL,NULL,simIR[0],simx_opmode_blocking);
		simxReadProximitySensor(clientID,proxSensorsHandle[1],NULL,NULL,NULL,simIR[1],simx_opmode_blocking);
		simxReadProximitySensor(clientID,proxSensorsHandle[2],NULL,NULL,NULL,simIR[2],simx_opmode_blocking);
		simxReadProximitySensor(clientID,proxSensorsHandle[3],NULL,NULL,NULL,simIR[3],simx_opmode_blocking);
		simxReadProximitySensor(clientID,proxSensorsHandle[4],NULL,NULL,NULL,simIR[4],simx_opmode_blocking);
		simxReadProximitySensor(clientID,proxSensorsHandle[5],NULL,NULL,NULL,simIR[5],simx_opmode_blocking);
		simxReadProximitySensor(clientID,proxSensorsHandle[6],NULL,NULL,NULL,simIR[6],simx_opmode_blocking);
		simxReadProximitySensor(clientID,proxSensorsHandle[7],NULL,NULL,NULL,simIR[7],simx_opmode_blocking);

		simxGetObjectPosition(clientID,ePuckHandle,sphereHandle,ePuckPosition,simx_opmode_blocking);

    }
}

void EPuckVREPDriver::read(){
	//simxSynchronousTrigger(clientID);

	simxReadProximitySensor(clientID,proxSensorsHandle[0],NULL,NULL,NULL,simIR[0],simx_opmode_blocking);
	simxReadProximitySensor(clientID,proxSensorsHandle[1],NULL,NULL,NULL,simIR[1],simx_opmode_blocking);
	simxReadProximitySensor(clientID,proxSensorsHandle[2],NULL,NULL,NULL,simIR[2],simx_opmode_blocking);
	simxReadProximitySensor(clientID,proxSensorsHandle[3],NULL,NULL,NULL,simIR[3],simx_opmode_blocking);
	simxReadProximitySensor(clientID,proxSensorsHandle[4],NULL,NULL,NULL,simIR[4],simx_opmode_blocking);
	simxReadProximitySensor(clientID,proxSensorsHandle[5],NULL,NULL,NULL,simIR[5],simx_opmode_blocking);
	simxReadProximitySensor(clientID,proxSensorsHandle[6],NULL,NULL,NULL,simIR[6],simx_opmode_blocking);
	simxReadProximitySensor(clientID,proxSensorsHandle[7],NULL,NULL,NULL,simIR[7],simx_opmode_blocking);

	simxGetObjectPosition(clientID,ePuckHandle,sphereHandle,ePuckPosition,simx_opmode_blocking);
	
	simxGetPingTime(clientID,&pingTime);
	std::cout<< "\nPing = " << pingTime << " ;\n" << std::endl;

	
	fileToWrite.addIn(ePuckPosition[0],ePuckPosition[1]);
	
	PrintSensors();
	
}
void EPuckVREPDriver::send(){
	simxPauseCommunication(clientID,1);
		simxSetJointTargetVelocity(clientID,rightJointHandle,robot_.wheels_command.right_velocity , simx_opmode_oneshot);
		simxSetJointTargetVelocity(clientID,leftJointHandle,robot_.wheels_command.left_velocity, simx_opmode_oneshot);
	simxPauseCommunication(clientID,0);
}

EPuckVREPDriver::~EPuckVREPDriver(){
	simxPauseSimulation(clientID,simx_opmode_blocking);
	usleep(1000);
	simxStopSimulation(clientID,simx_opmode_oneshot);
	usleep(1000);
	simxFinish(clientID);
	std::cout<<"End of the program"<<std::endl;
}

void EPuckVREPDriver::PrintSensors(){
	dataToRobot();

	std::cout << "ePuck location : " << ePuckPosition[0] << ", " << ePuckPosition[1] << std::endl;
	std::cout<<std::endl;
	std::cout << "P0 : " << robot_.proximity_sensors.IR[0] << "\n";
	std::cout << "P1 : " << robot_.proximity_sensors.IR[1] << "\n";
	std::cout << "P2 : " << robot_.proximity_sensors.IR[2] << "\n";
	std::cout << "P3 : " << robot_.proximity_sensors.IR[3] << "\n";
	std::cout << "P4 : " << robot_.proximity_sensors.IR[4] << "\n";
	std::cout << "P5 : " << robot_.proximity_sensors.IR[5] << "\n";
	std::cout << "P6 : " << robot_.proximity_sensors.IR[6] << "\n";
	std::cout << "P7 : " << robot_.proximity_sensors.IR[7] << "\n"; 
}

void EPuckVREPDriver::dataToRobot(){
	for(int i=0; i < 8 ; i++){
			if( std::abs(simIR[i][2]) < 0.01 ||  std::abs(simIR[i][2]) > 1){
				robot_.proximity_sensors.IR[i] = 0;
			}
			else{
				robot_.proximity_sensors.IR[i] = std::abs(simIR[i][2]);
			}
	}
	robot_.current_pose.x = ePuckPosition[0];
	robot_.current_pose.y = ePuckPosition[1];
}