//#include "RobotDriver.hpp"
#include "EPuckV2Driver.hpp"

#include <iostream>

EPuckV2Driver::EPuckV2Driver(Robot& robot) : RobotDriver(robot) {
}

EPuckV2Driver::~EPuckV2Driver() {
    std::stringstream ss;
    if(close(fd) < 0) {
        ss.str("");
        ss << "[" << epuckname << "] " << "Can't close tcp socket";
        if(DEBUG_CONNECTION_INIT)perror(ss.str().c_str());
    }
}

void closeConnection(){
	std::stringstream ss;
    if(close(fd) < 0) {
        ss.str("");
        ss << "[" << epuckname << "] " << "Can't close tcp socket";
        if(DEBUG_CONNECTION_INIT)perror(ss.str().c_str());
    }
}

// TODO
bool EPuckV2Driver::Init() {
    std::cout << "init V1\n";
    int ret_value;
	std::stringstream ss;
    struct timeval tv;
    socklen_t len = sizeof(tv);
	uint8_t trials = 0;
	
   	robot_addr.sin_family = AF_INET;
   	robot_addr.sin_addr.s_addr = inet_addr(epuckAddress.c_str());
   	robot_addr.sin_port = htons(EPUCK_PORT);

	if(DEBUG_CONNECTION_INIT)fprintf(stderr, "Try to connect to %s:%d (TCP)\n", inet_ntoa(robot_addr.sin_addr), htons(robot_addr.sin_port));
	
   	fd = socket(AF_INET, SOCK_STREAM, 0);
	if(fd < 0) {
		perror("TCP cannot create socket: ");
		return false;
	}
	
	// Set to non-blocking mode during connection otherwise it will block for too much time if the robot isn't ready to accept connections
    if( (ret_value = fcntl(fd, F_GETFL, 0)) < 0) {
		perror("Cannot get flag status: ");
		return false;
    }

	ret_value |= O_NONBLOCK;
	if(fcntl(fd, F_SETFL, ret_value) < 0) {
		perror("Cannot set non-blocking mode: ");
		return false;
	}
	
	while(trials < MAX_CONNECTION_TRIALS) {
		// Connection to the robot (server).
		ret_value = connect(fd, (struct sockaddr *) &robot_addr, sizeof(robot_addr));					
		if (ret_value == 0) {
			break;
		} else {
			trials++;
			if(DEBUG_CONNECTION_INIT)fprintf(stderr, "Connection trial %d\n", trials);
			sleep(3);			
		}
	}
	
	if(trials == MAX_CONNECTION_TRIALS) {
		ss.str("");
		ss << "[" << epuckname << "] " << "Error, can't connect to tcp socket";
		if(DEBUG_CONNECTION_INIT)perror(ss.str().c_str());
		return false;
	}
	
	// Set to blocking mode.
    if( (ret_value = fcntl(fd, F_GETFL, 0)) < 0) {
		perror("Cannot get flag status: ");
		return false;
    }

	ret_value &= (~O_NONBLOCK);
	if(fcntl(fd, F_SETFL, ret_value) < 0) {
		perror("Cannot set blocking mode: ");
		return false;
	}

	// Set the reception timeout. This is used when blocking mode is activated after connection.
	tv.tv_sec = READ_TIMEOUT_SEC;
	tv.tv_usec = READ_TIMEOUT_USEC;
	ret_value = setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
	if(ret_value < 0) {
		perror("Cannot set rx timeout: ");
		return false;
	}	
	
	std::cout << "Cmd initialization" << std::endl; 
	
	command[0] = 0x80;
	command[1] = 2;		// Sensors enabled.
	command[2] = 1;		// Calibrate proximity sensors.
	command[3] = 0;		// left motor LSB
	command[4] = 0;		// left motor MSB
	command[5] = 0;		// right motor LSB
	command[6] = 0;		// right motor MSB
	command[7] = 0;		// lEDs
	command[8] = 0;		// LED2 red
	command[9] = 0;		// LED2 green
	command[10] = 0;	// LED2 blue
	command[11] = 0;	// LED4 red
	command[12] = 0;	// LED4 green
	command[13] = 0;	// LED4 blue
	command[14] = 0;	// LED6 red
	command[15] = 0;	// LED6 green
	command[16] = 0;	// LED6 blue
	command[17] = 0;	// LED8 red
	command[18] = 0;	// LED8 green
	command[19] = 0;	// LED8 blue
	command[20] = 0;	// speaker   
	expected_recv_packets = 1;

	return true;
};

// TODO
void EPuckV2Driver::Read() {
 
	int bytes_recv = 0, ret_value;
	long mantis = 0;
	short exp = 0;
	float flt = 0.0;
				

			
	while(expected_recv_packets > 0) {
		bytes_recv = recv(fd, (char *)&header, 1, 0);
		if (bytes_recv <= 0) {
			closeConnection();
			if(Init() == false ) {
				std::cerr << "Lost connection with the robot" << std::endl;
				exit(1);
			} else {
				return; // Wait for the next sensor request
			}
		}
		
		switch(header) {
			case 0x01:	// Camera.				
				bytes_recv = 0;
				while(bytes_recv < sizeof(image)) {
					ret_value = recv(fd, (char *)&image[bytes_recv], sizeof(image)-bytes_recv, 0);
					if(ret_value <= 0) {
						closeConnection();
						if(Init() == false) {
							std::cerr << "Lost connection with the robot" << std::endl;
							exit(1);
						} else {
							return; // Wait for the next sensor request
						}
					} else {
						bytes_recv += ret_value;
						std::cout << "image read = " << bytes_recv << std::endl;
					}
				}
				
				if(DEBUG_UPDATE_SENSORS_DATA)std::cout << "[" << epuckname << "] " << "camera read correctly" << std::endl;
				newImageReceived = true;
				
				//red = image[0] & 0xf8;
				//green = image[0] << 5;
				//green += (image[1] & 0xf8) >> 3;
				//blue = image[1] << 3;
				//printf("1st pixel = %d, %d, %d\r\n", red, green, blue);
				break;
			
			case 0x02: // Sensors.
				bytes_recv = 0;
				while(bytes_recv < sizeof(sensor)) {
					ret_value = recv(fd, (char *)&sensor[bytes_recv], sizeof(sensor)-bytes_recv, 0);
					if(ret_value <= 0) {
						closeConnection();
						if(Init() == false ) {
							std::cerr << "Lost connection with the robot" << std::endl;
							exit(1);
						} else {
							return; // Wait for the next sensor request
						}
					} else {
						bytes_recv += ret_value;
						std::cout << "sensors read = " << bytes_recv << std::endl;
					}
				}

                accData[0] = sensor[0] + sensor[1]*256;
                accData[1] = sensor[2] + sensor[3]*256;
                accData[2] = sensor[4] + sensor[5]*256;			
				if(DEBUG_UPDATE_SENSORS_DATA)std::cout << "[" << epuckname << "] " << "acc: " << accData[0] << "," << accData[1] << "," << accData[2] << std::endl;
				
				// Compute acceleration
				mantis = (sensor[6] & 0xff) + ((sensor[7] & 0xffl) << 8) + (((sensor[8] &0x7fl) | 0x80) << 16);
				exp = (sensor[9] & 0x7f) * 2 + ((sensor[8] & 0x80) ? 1 : 0);
				if (sensor[9] & 0x80) {
					mantis = -mantis;
				}
				flt = (mantis || exp) ? ((float) ldexp (mantis, (exp - 127 - 23))): 0;
				acceleration=flt;
				if(DEBUG_UPDATE_SENSORS_DATA)std::cout << "[" << epuckname << "] " << "acceleration: " << acceleration << std::endl;

				// Compute orientation.
				mantis = (sensor[10] & 0xff) + ((sensor[11] & 0xffl) << 8) + (((sensor[12] &0x7fl) | 0x80) << 16);
				exp = (sensor[13] & 0x7f) * 2 + ((sensor[12] & 0x80) ? 1 : 0);
				if (sensor[13] & 0x80)
					mantis = -mantis;
				flt = (mantis || exp) ? ((float) ldexp (mantis, (exp - 127 - 23))): 0;
				orientation=flt;
				if (orientation < 0.0 )
					orientation=0.0;
				if (orientation > 360.0 )
					orientation=360.0;
				if(DEBUG_UPDATE_SENSORS_DATA)std::cout << "[" << epuckname << "] " << "orientation: " << orientation << std::endl;

				// Compute inclination.
				mantis = (sensor[14] & 0xff) + ((sensor[15] & 0xffl) << 8) + (((sensor[16] &0x7fl) | 0x80) << 16);
				exp = (sensor[17] & 0x7f) * 2 + ((sensor[16] & 0x80) ? 1 : 0);
				if (sensor[17] & 0x80)
					mantis = -mantis;
				flt = (mantis || exp) ? ((float) ldexp (mantis, (exp - 127 - 23))): 0;
				inclination=flt;
				if (inclination < 0.0 )
					inclination=0.0;
				if (inclination > 180.0 )
					inclination=180.0;
				if(DEBUG_UPDATE_SENSORS_DATA)std::cout << "[" << epuckname << "] " << "inclination: " << inclination << std::endl;

				// Gyro
				gyroRaw[0] = sensor[18]+sensor[19]*256;
				gyroRaw[1] = sensor[20]+sensor[21]*256;
				gyroRaw[2] = sensor[22]+sensor[23]*256;
				if(DEBUG_UPDATE_SENSORS_DATA)std::cout << "[" << epuckname << "] " << "gyro: " << gyroRaw[0] << "," << gyroRaw[1] << "," << gyroRaw[2] << std::endl;					

				// Magnetometer
				magneticField[0] = *((float*)&sensor[24]);
				magneticField[1] = *((float*)&sensor[28]);
				magneticField[2] = *((float*)&sensor[32]);
				if(DEBUG_UPDATE_SENSORS_DATA)std::cout << "[" << epuckname << "] " << "mag: " << magneticField[0] << "," << magneticField[1] << "," << magneticField[2] << std::endl;	

				// Temperature.
				temperature = sensor[36];
				if(DEBUG_UPDATE_SENSORS_DATA)std::cout << "[" << epuckname << "] " << "temperature: " << (int)temperature << std::endl;

				// Proximity sensors data.
				proxData[0] = sensor[37]+sensor[38]*256;
				proxData[1] = sensor[39]+sensor[40]*256;
				proxData[2] = sensor[41]+sensor[42]*256;
				proxData[3] = sensor[43]+sensor[44]*256;
				proxData[4] = sensor[45]+sensor[46]*256;
				proxData[5] = sensor[47]+sensor[48]*256;
				proxData[6] = sensor[49]+sensor[50]*256;
				proxData[7] = sensor[51]+sensor[52]*256;
				if(proxData[0]<0) {
					proxData[0]=0;
				}
				if(proxData[1]<0) {
					proxData[1]=0;
				}
				if(proxData[2]<0) {
					proxData[2]=0;
				}
				if(proxData[3]<0) {
					proxData[3]=0;
				}
				if(proxData[4]<0) {
					proxData[4]=0;
				}
				if(proxData[5]<0) {
					proxData[5]=0;
				}
				if(proxData[6]<0) {
					proxData[6]=0;
				}
				if(proxData[7]<0) {
					proxData[7]=0;
				}
				if(DEBUG_UPDATE_SENSORS_DATA)std::cout << "[" << epuckname << "] " << "prox: " << proxData[0] << "," << proxData[1] << "," << proxData[2] << "," << proxData[3] << "," << proxData[4] << "," << proxData[5] << "," << proxData[6] << "," << proxData[7] << std::endl;				

				// Compute abmient light.
				lightAvg += (sensor[53]+sensor[54]*256);
				lightAvg += (sensor[55]+sensor[56]*256);
				lightAvg += (sensor[57]+sensor[58]*256);
				lightAvg += (sensor[59]+sensor[60]*256);
				lightAvg += (sensor[61]+sensor[62]*256);
				lightAvg += (sensor[63]+sensor[64]*256);
				lightAvg += (sensor[65]+sensor[66]*256);
				lightAvg += (sensor[67]+sensor[68]*256);
				lightAvg = (int) (lightAvg/8);
				lightAvg = (lightAvg>4000)?4000:lightAvg;
				if(lightAvg<0) {
					lightAvg=0;
				}
				if(DEBUG_UPDATE_SENSORS_DATA)std::cout << "[" << epuckname << "] " << "lightAvg: " << lightAvg << std::endl;
				
				// ToF
				distanceCm = (uint16_t)(((uint8_t)sensor[70]<<8)|((uint8_t)sensor[69]))/10;
				if(distanceCm > 200) {
					distanceCm = 200;
				}
				if(DEBUG_UPDATE_SENSORS_DATA)std::cout << "[" << epuckname << "] " << "distanceCm: " << distanceCm << "(" << (int)sensor[69] << "," << (int)sensor[70] << ")" << std::endl;

				// Microphone
				micVolume[0] = ((uint8_t)sensor[71]+(uint8_t)sensor[72]*256);
				micVolume[1] = ((uint8_t)sensor[73]+(uint8_t)sensor[74]*256);
				micVolume[2] = ((uint8_t)sensor[75]+(uint8_t)sensor[76]*256);
				micVolume[3] = ((uint8_t)sensor[77]+(uint8_t)sensor[78]*256);
				if(DEBUG_UPDATE_SENSORS_DATA)std::cout << "[" << epuckname << "] " << "mic: " << micVolume[0] << "," << micVolume[1] << "," << micVolume[2] << "," << micVolume[3] << std::endl;

				// Left steps
				motorSteps[0] = (sensor[79]+sensor[80]*256);
				// Right steps
				motorSteps[1] = (sensor[81]+sensor[82]*256);
				if(DEBUG_UPDATE_SENSORS_DATA)std::cout << "[" << epuckname << "] " << "steps: " << motorSteps[0] << "," << motorSteps[1] << std::endl;

				// Battery
				batteryRaw = (uint8_t)sensor[83]+(uint8_t)sensor[84]*256;
				if(DEBUG_UPDATE_SENSORS_DATA)std::cout << "[" << epuckname << "] " << "batteryRaw: " << batteryRaw << std::endl;
				
				// Micro sd state.
				microSdState = sensor[85];
				if(DEBUG_UPDATE_SENSORS_DATA)std::cout << "[" << epuckname << "] " << "microSdState: " << (int)microSdState << std::endl;

				// Tv remote.
				irCheck = sensor[86];
				irAddress = sensor[87];
				irData = sensor[88];
				if(DEBUG_UPDATE_SENSORS_DATA)std::cout << "[" << epuckname << "] " << "tv remote: " << (int)irCheck << "," << (int)irAddress << "," << (int)irData << std::endl;

				// Selector.
				selector = sensor[89];
				if(DEBUG_UPDATE_SENSORS_DATA)std::cout << "[" << epuckname << "] " << "selector: " << (int)selector << std::endl;

				// Ground sensor proximity.
				groundProx[0] = sensor[90]+sensor[91]*256;
				groundProx[1] = sensor[92]+sensor[93]*256;
				groundProx[2] = sensor[94]+sensor[95]*256;
				if(DEBUG_UPDATE_SENSORS_DATA)std::cout << "[" << epuckname << "] " << "groundProx: " << groundProx[0] << "," << groundProx[1] << "," << groundProx[2] << std::endl;

				// Ground sensor ambient light.
				groundAmbient[0] = sensor[96]+sensor[97]*256;
				groundAmbient[1] = sensor[98]+sensor[99]*256;
				groundAmbient[2] = sensor[100]+sensor[101]*256;
				if(DEBUG_UPDATE_SENSORS_DATA)std::cout << "[" << epuckname << "] " << "groundAmbient: " << groundAmbient[0] << "," << groundAmbient[1] << "," << groundAmbient[2] << std::endl;

				// Button state.
				buttonState = sensor[102];			
				if(DEBUG_UPDATE_SENSORS_DATA)std::cout << "[" << epuckname << "] " << "buttonState: " << (int)buttonState << std::endl;
				
				break;
			
			case 0x03:
				printf("empty packet\r\n");
				break;
			
			default:
				printf("unexpected packet\r\n");
				break;
		}
		expected_recv_packets--;
	}

	if(camera_enabled) {
		expected_recv_packets = 2;
	} else {
		expected_recv_packets = 1;
	}
};

void ToggleLed(uchar*);
// TODO
void EPuckV2Driver::Send() {
    //std::cout << "send V1\n";
	command[0] = 0x80;
	command[1] = 2;		// Sensors enabled.
	command[2] = 1;		// Calibrate proximity sensors.
	command[3] = 0;		// left motor LSB
	command[4] = 0;		// left motor MSB
	command[5] = 0;		// right motor LSB
	command[6] = 0;		// right motor MSB
	command[7] = 0;		// lEDs
	command[8] = 0;		// LED2 red
	command[9] = 0;		// LED2 green
	command[10] = 0;	// LED2 blue
	command[11] = 0;	// LED4 red
	command[12] = 0;	// LED4 green
	command[13] = 0;	// LED4 blue
	command[14] = 0;	// LED6 red
	command[15] = 0;	// LED6 green
	command[16] = 0;	// LED6 blue
	//ToggleLed(command);
	command[17] = 0;	// LED8 red
	command[18] = 0;	// LED8 green
	command[19] = 0;	// LED8 blue
	command[20] = 0;	// speaker

   	int bytes_sent = 0;
	bytes_sent = 0;
	while(bytes_sent < sizeof(command)) {
		bytes_sent += send(fd, (char *)&command[bytes_sent], sizeof(command)-bytes_sent, 0);
	}
	command[2] = 0; // Stop proximity calibration.

};

void ToggleLed(uchar* led) {
	if(led[14] == 0 && led[15] == 0 && led[16] == 0){
		led[14] = 1;
		led[15] = 0;
		led[16] = 0;
	}else if (led[14] == 1 && led[15] == 0 && led[16] == 0){
		led[14] = 0;
		led[15] = 1;
		led[16] = 0;
	}else if (led[15] == 1 && led[14] == 0 && led[16] == 0){
		led[14] = 0;
		led[15] = 0;
		led[16] = 1;
	}else if (led[16] == 1 && led[15] == 0 && led[14] == 0){
		led[14] = 0;
		led[15] = 0;
		led[16] = 0;
	}
}

// TODO
void EPuckV2Driver::getVisionSensor(Robot& robot) {
    //std::cout << "Image V2\n";
}