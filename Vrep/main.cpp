#include <iostream>

#include "extApiPlatform.h"
#include "extApiPlatform.c"
#include "extApi.h"
#include "extApi.c"

#include <unistd.h>
#include <pthread.h>

#include <opencv2/highgui.hpp>

int main(int argc,char* argv[]){
    int clientID= simxStart((simxChar*)"127.0.0.1",19997,true,true,2000,5);
	std::cout<<"clientID = "<<clientID<<std::endl;
	if (clientID==-1){
		std::cout<<("Could not connect to V-REP remote API server")<<std::endl;
		simxFinish(clientID);
	}else{
		std::cout<<("Connected to remote API server")<<std::endl;

        
        simxStartSimulation(clientID,simx_opmode_oneshot);
        
		sleep(5);

        simxPauseSimulation(clientID,simx_opmode_oneshot);

		
		
		simxStopSimulation(clientID,simx_opmode_oneshot);

	
		simxFinish(clientID);
	}
	std::cout<<"End of the program"<<std::endl;
}
