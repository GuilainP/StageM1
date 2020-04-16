#include <iostream>

#include "extApiPlatform.h"
#include "extApiPlatform.c"
#include "extApi.h"
#include "extApi.c"

#define PI 3.14159

simxInt proxSensors[8];
float detectedPoint0[3];
float detectedPoint1[3];
float detectedPoint2[3];
float detectedPoint3[3];
float detectedPoint4[3];
float detectedPoint5[3];
float detectedPoint6[3];
float detectedPoint7[3];
simxInt rightJoint;
simxInt leftJoint;

double rightM(0.0),leftM(0.0);

void printD(){
	std::cout << "P0 : " << detectedPoint0[0] << ", " << detectedPoint0[1] << ", "<< detectedPoint0[2] << std::endl;
	std::cout << "P1 : " << detectedPoint1[0] << ", " << detectedPoint1[1] << ", "<< detectedPoint1[2] << std::endl;
	std::cout << "P2 : " << detectedPoint2[0] << ", " << detectedPoint2[1] << ", "<< detectedPoint2[2] << std::endl;
	std::cout << "P3 : " << detectedPoint3[0] << ", " << detectedPoint3[1] << ", "<< detectedPoint3[2] << std::endl;
	std::cout << "P4 : " << detectedPoint4[0] << ", " << detectedPoint4[1] << ", "<< detectedPoint4[2] << std::endl;
	std::cout << "P5 : " << detectedPoint5[0] << ", " << detectedPoint5[1] << ", "<< detectedPoint5[2] << std::endl;
	std::cout << "P6 : " << detectedPoint6[0] << ", " << detectedPoint6[1] << ", "<< detectedPoint6[2] << std::endl;
	std::cout << "P7 : " << detectedPoint7[0] << ", " << detectedPoint7[1] << ", "<< detectedPoint7[2] << std::endl;
	
}

int main(int argc,char* argv[]){
    int clientID= simxStart((simxChar*)"127.0.0.1",19997,true,true,2000,5);
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


		simxReadProximitySensor(clientID,proxSensors[0],NULL,NULL,NULL,detectedPoint0,simx_opmode_streaming);
		simxReadProximitySensor(clientID,proxSensors[1],NULL,NULL,NULL,detectedPoint1,simx_opmode_streaming);
		simxReadProximitySensor(clientID,proxSensors[2],NULL,NULL,NULL,detectedPoint2,simx_opmode_streaming);
		simxReadProximitySensor(clientID,proxSensors[3],NULL,NULL,NULL,detectedPoint3,simx_opmode_streaming);
		simxReadProximitySensor(clientID,proxSensors[4],NULL,NULL,NULL,detectedPoint4,simx_opmode_streaming);
		simxReadProximitySensor(clientID,proxSensors[5],NULL,NULL,NULL,detectedPoint5,simx_opmode_streaming);
		simxReadProximitySensor(clientID,proxSensors[6],NULL,NULL,NULL,detectedPoint6,simx_opmode_streaming);
		simxReadProximitySensor(clientID,proxSensors[7],NULL,NULL,NULL,detectedPoint7,simx_opmode_streaming);

			simxSetJointTargetVelocity(clientID, rightJoint, rightM, simx_opmode_oneshot_wait);
        	simxSetJointTargetVelocity(clientID,  leftJoint, leftM, simx_opmode_oneshot_wait);

		for(int i =0; i<1000; i++){

			
			//rightM += PI/50;
			//leftM -=  PI/50;
			simxReadProximitySensor(clientID,proxSensors[0],NULL,NULL,NULL,detectedPoint0,simx_opmode_buffer);
			simxReadProximitySensor(clientID,proxSensors[1],NULL,NULL,NULL,detectedPoint1,simx_opmode_buffer);
			simxReadProximitySensor(clientID,proxSensors[2],NULL,NULL,NULL,detectedPoint2,simx_opmode_buffer);
			simxReadProximitySensor(clientID,proxSensors[3],NULL,NULL,NULL,detectedPoint3,simx_opmode_buffer);
			simxReadProximitySensor(clientID,proxSensors[4],NULL,NULL,NULL,detectedPoint4,simx_opmode_buffer);
			simxReadProximitySensor(clientID,proxSensors[5],NULL,NULL,NULL,detectedPoint5,simx_opmode_buffer);
			simxReadProximitySensor(clientID,proxSensors[6],NULL,NULL,NULL,detectedPoint6,simx_opmode_buffer);
			simxReadProximitySensor(clientID,proxSensors[7],NULL,NULL,NULL,detectedPoint7,simx_opmode_buffer);


			std::cout << " count = " << i << '\n';
			printD();
		
			usleep(5000);
		}
        

        

        simxPauseSimulation(clientID,simx_opmode_oneshot);

		
		
		simxStopSimulation(clientID,simx_opmode_oneshot);

	
		simxFinish(clientID);
	}
	std::cout<<"End of the program"<<std::endl;
}
