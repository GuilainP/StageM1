#include "RobotDriver.hpp"
#include "EPuckV2Driver.hpp"

EPuckV2Driver::EPuckV2Driver(Robot& robot) : RobotDriver(robot) {
	robot.vision_sensors = cv::Mat(120, 160, CV_8UC3);
	robotIP = "192.168.1.101";
	robotID = "4550";
	bytes_sent = 0;
	bytes_recv = 0;
	expected_recv_packets = 0;
	newImageReceived = false;

	leftStepsDiff = 0, rightStepsDiff = 0;
    leftStepsPrev = 0, rightStepsPrev = 0;
    leftStepsRawPrev = 0, rightStepsRawPrev = 0;

    overflowCountLeft = 0, overflowCountRight = 0;
    gyroOffset[0]=0, gyroOffset[1]=0, gyroOffset[2]=0; // Used if making an initial calibration of the gyro.
    speedLeft = 0, speedRight = 0;


}

EPuckV2Driver::~EPuckV2Driver() {
    std::stringstream ss;

	command[0] = 0x80;
	command[1] = 0;		// 0b10 Sensors enabled , 0b01 Image enabled
	command[2] = 0;		// Calibrate proximity sensors.
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

	int bytes_sent = 0;
	while(bytes_sent < sizeof(command)) {
		bytes_sent += send(fd, (char *)&command[bytes_sent], sizeof(command)-bytes_sent, 0);
	}

    if(close(fd) < 0) {
        ss.str("");
        ss << "[" << robotIP << "] " << "Can't close tcp socket";
        if(DEBUG_CONNECTION_INIT)perror(ss.str().c_str());
    }

	log().~Logger();
}

void EPuckV2Driver::closeConnection(){
	std::stringstream ss;
    if(close(fd) < 0) {
        ss.str("");
        ss << "[" << robotIP << "] " << "Can't close tcp socket";
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
   	robot_addr.sin_addr.s_addr = inet_addr(robotIP.c_str());
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
		ss << "[" << robotIP << "] " << "Error, can't connect to tcp socket";
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
	command[1] = 0b11;		// 0b10 Sensors enabled , 0b01 Image enabled
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
				while(bytes_recv < sizeof(this->image)) {
					ret_value = recv(fd, (char *)&this->image[bytes_recv], sizeof(this->image)-bytes_recv, 0);
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
						//std::cout << "image read = " << bytes_recv << std::endl;
					}
				}
				
				if(DEBUG_UPDATE_SENSORS_DATA)std::cout << "[" << robotIP << "] " << "camera read correctly" << std::endl;
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
				if(DEBUG_UPDATE_SENSORS_DATA)std::cout << "[" << robotIP << "] " << "acc: " << accData[0] << "," << accData[1] << "," << accData[2] << std::endl;
				
				// Compute acceleration
				mantis = (sensor[6] & 0xff) + ((sensor[7] & 0xffl) << 8) + (((sensor[8] &0x7fl) | 0x80) << 16);
				exp = (sensor[9] & 0x7f) * 2 + ((sensor[8] & 0x80) ? 1 : 0);
				if (sensor[9] & 0x80) {
					mantis = -mantis;
				}
				flt = (mantis || exp) ? ((float) ldexp (mantis, (exp - 127 - 23))): 0;
				acceleration=flt;
				if(DEBUG_UPDATE_SENSORS_DATA)std::cout << "[" << robotIP << "] " << "acceleration: " << acceleration << std::endl;

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
				if(DEBUG_UPDATE_SENSORS_DATA)std::cout << "[" << robotIP << "] " << "orientation: " << orientation << std::endl;
/*
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
				if(DEBUG_UPDATE_SENSORS_DATA)std::cout << "[" << robotIP << "] " << "inclination: " << inclination << std::endl;

				// Gyro
				gyroRaw[0] = sensor[18]+sensor[19]*256;
				gyroRaw[1] = sensor[20]+sensor[21]*256;
				gyroRaw[2] = sensor[22]+sensor[23]*256;
				if(DEBUG_UPDATE_SENSORS_DATA)std::cout << "[" << robotIP << "] " << "gyro: " << gyroRaw[0] << "," << gyroRaw[1] << "," << gyroRaw[2] << std::endl;					

				// Magnetometer
				magneticField[0] = *((float*)&sensor[24]);
				magneticField[1] = *((float*)&sensor[28]);
				magneticField[2] = *((float*)&sensor[32]);
				if(DEBUG_UPDATE_SENSORS_DATA)std::cout << "[" << robotIP << "] " << "mag: " << magneticField[0] << "," << magneticField[1] << "," << magneticField[2] << std::endl;	

				// Temperature.
				temperature = sensor[36];
				if(DEBUG_UPDATE_SENSORS_DATA)std::cout << "[" << robotIP << "] " << "temperature: " << (int)temperature << std::endl;
*/
				// Proximity sensors data.
				robot().proximity_sensors.IR[0] = sensor[37]+sensor[38]*256;
				robot().proximity_sensors.IR[1] = sensor[39]+sensor[40]*256;
				robot().proximity_sensors.IR[2] = sensor[41]+sensor[42]*256;
				robot().proximity_sensors.IR[3] = sensor[43]+sensor[44]*256;
				robot().proximity_sensors.IR[4] = sensor[45]+sensor[46]*256;
				robot().proximity_sensors.IR[5] = sensor[47]+sensor[48]*256;
				robot().proximity_sensors.IR[6] = sensor[49]+sensor[50]*256;
				robot().proximity_sensors.IR[7] = sensor[51]+sensor[52]*256;
				if(robot().proximity_sensors.IR[0]<0) {
					robot().proximity_sensors.IR[0]=0;
				}
				if(robot().proximity_sensors.IR[1]<0) {
					robot().proximity_sensors.IR[1]=0;
				}
				if(robot().proximity_sensors.IR[2]<0) {
					robot().proximity_sensors.IR[2]=0;
				}
				if(robot().proximity_sensors.IR[3]<0) {
					robot().proximity_sensors.IR[3]=0;
				}
				if(robot().proximity_sensors.IR[4]<0) {
					robot().proximity_sensors.IR[4]=0;
				}
				if(robot().proximity_sensors.IR[5]<0) {
					robot().proximity_sensors.IR[5]=0;
				}
				if(robot().proximity_sensors.IR[6]<0) {
					robot().proximity_sensors.IR[6]=0;
				}
				if(robot().proximity_sensors.IR[7]<0) {
					robot().proximity_sensors.IR[7]=0;
				}
				if(DEBUG_UPDATE_SENSORS_DATA)std::cout << "[" << robotIP << "] " << "prox: " << robot().proximity_sensors.IR[0] << "," << robot().proximity_sensors.IR[1] << "," << robot().proximity_sensors.IR[2] << "," << robot().proximity_sensors.IR[3] << "," << robot().proximity_sensors.IR[4] << "," << robot().proximity_sensors.IR[5] << "," << robot().proximity_sensors.IR[6] << "," << robot().proximity_sensors.IR[7] << std::endl;				
/*
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
				if(DEBUG_UPDATE_SENSORS_DATA)std::cout << "[" << robotIP << "] " << "lightAvg: " << lightAvg << std::endl;
				
				// ToF
				distanceCm = (uint16_t)(((uint8_t)sensor[70]<<8)|((uint8_t)sensor[69]))/10;
				if(distanceCm > 200) {
					distanceCm = 200;
				}
				if(DEBUG_UPDATE_SENSORS_DATA)std::cout << "[" << robotIP << "] " << "distanceCm: " << distanceCm << "(" << (int)sensor[69] << "," << (int)sensor[70] << ")" << std::endl;

				// Microphone
				micVolume[0] = ((uint8_t)sensor[71]+(uint8_t)sensor[72]*256);
				micVolume[1] = ((uint8_t)sensor[73]+(uint8_t)sensor[74]*256);
				micVolume[2] = ((uint8_t)sensor[75]+(uint8_t)sensor[76]*256);
				micVolume[3] = ((uint8_t)sensor[77]+(uint8_t)sensor[78]*256);
				if(DEBUG_UPDATE_SENSORS_DATA)std::cout << "[" << robotIP << "] " << "mic: " << micVolume[0] << "," << micVolume[1] << "," << micVolume[2] << "," << micVolume[3] << std::endl;
*/
				// Left steps
				robot().wheels_state.left_position = (double)(sensor[79]+sensor[80]*256);
				// Right steps
				robot().wheels_state.right_position = (double)(sensor[81]+sensor[82]*256);
				//if(DEBUG_UPDATE_SENSORS_DATA)std::cout << "[" << robotIP << "] " << "steps: " << motorSteps[0] << "," << motorSteps[1] << std::endl;
/*
				// Battery
				batteryRaw = (uint8_t)sensor[83]+(uint8_t)sensor[84]*256;
				if(DEBUG_UPDATE_SENSORS_DATA)std::cout << "[" << robotIP << "] " << "batteryRaw: " << batteryRaw << std::endl;
				
				// Micro sd state.
				microSdState = sensor[85];
				if(DEBUG_UPDATE_SENSORS_DATA)std::cout << "[" << robotIP << "] " << "microSdState: " << (int)microSdState << std::endl;

				// Tv remote.
				irCheck = sensor[86];
				irAddress = sensor[87];
				irData = sensor[88];
				if(DEBUG_UPDATE_SENSORS_DATA)std::cout << "[" << robotIP << "] " << "tv remote: " << (int)irCheck << "," << (int)irAddress << "," << (int)irData << std::endl;

				// Selector.
				selector = sensor[89];
				if(DEBUG_UPDATE_SENSORS_DATA)std::cout << "[" << robotIP << "] " << "selector: " << (int)selector << std::endl;

				// Ground sensor proximity.
				groundProx[0] = sensor[90]+sensor[91]*256;
				groundProx[1] = sensor[92]+sensor[93]*256;
				groundProx[2] = sensor[94]+sensor[95]*256;
				if(DEBUG_UPDATE_SENSORS_DATA)std::cout << "[" << robotIP << "] " << "groundProx: " << groundProx[0] << "," << groundProx[1] << "," << groundProx[2] << std::endl;

				// Ground sensor ambient light.
				groundAmbient[0] = sensor[96]+sensor[97]*256;
				groundAmbient[1] = sensor[98]+sensor[99]*256;
				groundAmbient[2] = sensor[100]+sensor[101]*256;
				if(DEBUG_UPDATE_SENSORS_DATA)std::cout << "[" << robotIP << "] " << "groundAmbient: " << groundAmbient[0] << "," << groundAmbient[1] << "," << groundAmbient[2] << std::endl;

				// Button state.
				buttonState = sensor[102];			
				if(DEBUG_UPDATE_SENSORS_DATA)std::cout << "[" << robotIP << "] " << "buttonState: " << (int)buttonState << std::endl;
*/
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
	PrintSensors();
};

void ToggleLed(uchar*);
// TODO
void EPuckV2Driver::Send() {
    //std::cout << "send V1\n";
	command[0] = 0x80;
	command[1] = 0b11;	// 0b10 Sensors enabled , 0b01 Image enabled
	command[2] = 1;		// Calibrate proximity sensors.
	command[3] = 0;		// left motor LSB
	command[4] = robot().wheels_command.left_velocity;		// left motor MSB
	command[5] = 0;		// right motor LSB
	command[6] = robot().wheels_command.right_velocity;		// right motor MSB
	command[7] = 0b0;	// lEDs
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

   	int bytes_sent = 0;
	bytes_sent = 0;
	while(bytes_sent < sizeof(command)) {
		bytes_sent += send(fd, (char *)&command[bytes_sent], sizeof(command)-bytes_sent, 0);
	}
	command[2] = 0; // Stop proximity calibration.

};

void RGB565toRGB888(int width, int height, unsigned char *src, unsigned char *dst) {
    int line, column;
    int index_src=0, index_dst=0;

    for (line = 0; line < height; ++line) {
        for (column = 0; column < width; ++column) {
            dst[index_dst++] = (unsigned char)(src[index_src] & 0xF8);
            dst[index_dst++] = (unsigned char)((src[index_src]&0x07)<<5) | (unsigned char)((src[index_src+1]&0xE0)>>3);
            dst[index_dst++] = (unsigned char)((src[index_src+1]&0x1F)<<3);
            index_src+=2;
        }
    }
}

void EPuckV2Driver::getVisionSensor(Robot& robot) {
	if(command[1]%2 == 1){
		RGB565toRGB888(160, 120, &image[0], robot.vision_sensors.data);  
		cv::namedWindow("Camera", cv::WINDOW_NORMAL);
		cv::resizeWindow("Camera", 160*2, 120*2);
		cv::imshow("Camera", robot.vision_sensors);
		cv::waitKey(1);
		
		cv::imwrite(log().folder_ + "/image/image" +  std::string( 4 - std::to_string(ajouter).length(), '0').append( std::to_string(ajouter)) + ".png", robot.vision_sensors);

		++ajouter;
	}

}

void EPuckV2Driver::PrintSensors() {
    std::cout << "Iteration N"<<ajouter << "\n"
              //<< "ePuck location :  x : " << robot().current_pose.x << ", y : " << robot().current_pose.y << ", th : " << robot().current_pose.th << "\n"
              << "Joint position [rad] :  Left : " << robot().wheels_state.left_position << ", Right : " << robot().wheels_state.right_position << "\n";
              //<< "Speed [rad/s]   : Left : "  << robot().wheels_state.left_velocity << ", Right : " << robot().wheels_state.right_velocity << std::endl;


    std::cout << "IR0 : " << robot().proximity_sensors.IR[0] << "\n";
    std::cout << "IR0 : " << robot().proximity_sensors.IR[1] << "\n";
    std::cout << "IR2 : " << robot().proximity_sensors.IR[2] << "\n";
    std::cout << "IR3 : " << robot().proximity_sensors.IR[3] << "\n";
    std::cout << "IR4 : " << robot().proximity_sensors.IR[4] << "\n";
    std::cout << "IR5 : " << robot().proximity_sensors.IR[5] << "\n";
    std::cout << "IR6 : " << robot().proximity_sensors.IR[6] << "\n";
    std::cout << "IR7 : " << robot().proximity_sensors.IR[7] << "\n";
/*
    log().addIn(log().file_ePuckPose[0], robot().current_pose.x);
    log().addIn(log().file_ePuckPose[1], robot().current_pose.y);
    log().addIn(log().file_ePuckPose[2], robot().current_pose.th);
*/
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
/*
    log().addIn(log().file_ePuckLeftWheelVelocity, robot().wheels_state.left_velocity);
    log().addIn(log().file_ePuckRightWheelVelocity, robot().wheels_state.right_velocity);
*/
}