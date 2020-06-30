#include "RobotDriver.hpp"

#include <chrono>
#include <thread>

#define DEBUG_CONNECTION_INIT 0
#define DEBUG_UPDATE_SENSORS_DATA 0
#define DEBUG_ODOMETRY 0

#define READ_TIMEOUT_SEC 10
#define READ_TIMEOUT_USEC 0
#define MAX_CONNECTION_TRIALS 3

#define WHEEL_DIAMETER 4        // cm.
#define WHEEL_DISTANCE 0.053    // Distance between wheels in meters (axis length).
#define WHEEL_CIRCUMFERENCE ((WHEEL_DIAMETER*M_PI)/100.0)    // Wheel circumference (meters).
#define MOT_STEP_DIST (WHEEL_CIRCUMFERENCE/1000.0)      // Distance for each motor step (meters); a complete turn is 1000 steps (0.000125 meters per step (m/steps)).
#define ROBOT_RADIUS 0.035 // meters.

#define DEG2RAD(deg) (deg / 180 * M_PI)
#define GYRO_RAW2DPS (250.0/32768.0f)   //250DPS (degrees per second) scale for int16 raw value
#define STANDARD_GRAVITY 9.80665f

EPuckV2Driver::EPuckV2Driver(Robot& robot) : RobotDriver(robot) {
	robot.vision_sensors = cv::Mat(120, 160, CV_8UC3);
	cnt_iter == 1;

	bytes_sent_ = 0;
	bytes_recv_ = 0;
	expected_recv_packets_ = 0;
	new_image_received_ = false;

	x_pos_ = 0 , y_pos_ = 0, theta_ = 0;
	left_steps_diff_ = 0, right_steps_diff_ = 0;
    left_steps_prev_ = 0, right_steps_prev_ = 0;
    left_steps_raw_prev_ = 0, right_steps_raw_prev_ = 0;
	left_steps_diff_avg_ = 0, right_steps_diff_avg_ = 0, time_ten_iter_ = 0; 

    overflow_count_left_ = 0, overflow_count_right_ = 0;
    gyro_offset_[0]=0, gyro_offset_[1]=0, gyro_offset_[2]=0; // Used if making an initial calibration of the gyro.
}

EPuckV2Driver::~EPuckV2Driver() {

    //std::cout << "send V1\n";
	command_[0] = 0x80;
	command_[1] = 0b11;	// 0b10 Sensors enabled , 0b01 Image enabled
	command_[2] = 0;		// Calibrate proximity sensors.
	command_[3] = 0;		// left motor LSB
	command_[4] = 0;		// left motor MSB
	command_[5] = 0;		// right motor LSB
	command_[6] = 0;		// right motor MSB
	command_[7] = 0b0; //0b111111;	// lEDs
	command_[8] = 0;		// LED2 red
	command_[9] = 0;		// LED2 green
	command_[10] = 0;	// LED2 blue
	command_[11] = 0;	// LED4 red
	command_[12] = 0;	// LED4 green
	command_[13] = 0;	// LED4 blue
	command_[14] = 0;	// LED6 red
	command_[15] = 0;	// LED6 green
	command_[16] = 0;	// LED6 blue
	command_[17] = 0;	// LED8 red
	command_[18] = 0;	// LED8 green
	command_[19] = 0;	// LED8 blue
	command_[20] = 0;	// speaker

	std::this_thread::sleep_for(std::chrono::milliseconds(500));
	for(int i = 0; i<10; ++i) {
		bytes_sent_ = 0;
		while(bytes_sent_ < sizeof(command_)) {
			bytes_sent_ += send(fd_, (char *)&command_[bytes_sent_], sizeof(command_)-bytes_sent_, 0);
		}
	}
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));

	closeConnection();
	std::cout << "DRIVER V2 DESTRUCTED\n";
}

void EPuckV2Driver::closeConnection(){
	std::stringstream ss;
    if(close(fd_) < 0) {
        ss.str("");
        ss << "[" << robot().ip  << "] " << "Can't close tcp socket";
        if(DEBUG_CONNECTION_INIT)perror(ss.str().c_str());
    }
}

bool EPuckV2Driver::init() {

    std::cout << "init V1\n";
    int ret_value;
	std::stringstream ss;
    struct timeval tv;
    socklen_t len = sizeof(tv);
	uint8_t trials = 0;
	
   	robot_addr_.sin_family = AF_INET;
   	robot_addr_.sin_addr.s_addr = inet_addr(robot().ip .c_str());
   	robot_addr_.sin_port = htons(1000); // E-puck V2 port

	if(DEBUG_CONNECTION_INIT)fprintf(stderr, "Try to connect to %s:%d (TCP)\n", inet_ntoa(robot_addr_.sin_addr), htons(robot_addr_.sin_port));
	
   	fd_ = socket(AF_INET, SOCK_STREAM, 0);
	if(fd_ < 0) {
		perror("TCP cannot create socket: ");
		return false;
	}
	
	// Set to non-blocking mode during connection otherwise it will block for too much time if the robot isn't ready to accept connections
    if( (ret_value = fcntl(fd_, F_GETFL, 0)) < 0) {
		perror("Cannot get flag status: ");
		return false;
    }

	ret_value |= O_NONBLOCK;
	if(fcntl(fd_, F_SETFL, ret_value) < 0) {
		perror("Cannot set non-blocking mode: ");
		return false;
	}
	
	while(trials < MAX_CONNECTION_TRIALS) {
		// Connection to the robot (server).
		ret_value = connect(fd_, (struct sockaddr *) &robot_addr_, sizeof(robot_addr_));					
		if (ret_value == 0) {
			break;
		} else {
			trials++;
			if(DEBUG_CONNECTION_INIT)fprintf(stderr, "Connection trial %d\n", trials);
			std::this_thread::sleep_for(std::chrono::milliseconds(1000));			
		}
	}
	
	if(trials == MAX_CONNECTION_TRIALS) {
		ss.str("");
		ss << "[" << robot().ip  << "] " << "Error, can't connect to tcp socket";
		if(DEBUG_CONNECTION_INIT)perror(ss.str().c_str());
		return false;
	}
	
	// Set to blocking mode.
    if( (ret_value = fcntl(fd_, F_GETFL, 0)) < 0) {
		perror("Cannot get flag status: ");
		return false;
    }

	ret_value &= (~O_NONBLOCK);
	if(fcntl(fd_, F_SETFL, ret_value) < 0) {
		perror("Cannot set blocking mode: ");
		return false;
	}

	// Set the reception timeout. This is used when blocking mode is activated after connection.
	tv.tv_sec = READ_TIMEOUT_SEC;
	tv.tv_usec = READ_TIMEOUT_USEC;
	ret_value = setsockopt(fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
	if(ret_value < 0) {
		perror("Cannot set rx timeout: ");
		return false;
	}	
	
	std::cout << "Cmd initialization" << std::endl; 
	
	command_[0] = 0x80;
	command_[1] = 0b11;	// 0b10 Sensors enabled , 0b01 Image enabled
	command_[2] = 1;//1;		// Calibrate proximity sensors.
	command_[3] = 0;		// left motor LSB
	command_[4] = 0;		// left motor MSB
	command_[5] = 0;		// right motor LSB
	command_[6] = 0;		// right motor MSB
	command_[7] = 0;		// lEDs
	command_[8] = 0;		// LED2 red
	command_[9] = 0;		// LED2 green
	command_[10] = 0;	// LED2 blue
	command_[11] = 0;	// LED4 red
	command_[12] = 0;	// LED4 green
	command_[13] = 0;	// LED4 blue
	command_[14] = 0;	// LED6 red
	command_[15] = 0;	// LED6 green
	command_[16] = 0;	// LED6 blue
	command_[17] = 0;	// LED8 red
	command_[18] = 0;	// LED8 green
	command_[19] = 0;	// LED8 blue
	command_[20] = 0;	// speaker   
	expected_recv_packets_ = 1;


	bytes_sent_ = 0;
	while(bytes_sent_ < sizeof(command_)) {
		bytes_sent_ += send(fd_, (char *)&command_[bytes_sent_], sizeof(command_)-bytes_sent_, 0);
	}
	command_[2] = 0; // Stop proximity calibration.

	start_time = std::chrono::high_resolution_clock::now();

	return true;
};

// TODO
void EPuckV2Driver::read() {
 
	bytes_recv_ = 0;
	int ret_value;
	long mantis = 0;
	short exp = 0;
	float flt = 0.0;

	while(expected_recv_packets_ > 0) {
		bytes_recv_ = recv(fd_, (char *)&header_, 1, 0);
		if (bytes_recv_ <= 0) {
			closeConnection();
			if(init() == false ) {
				std::cerr << "Lost connection with the robot" << std::endl;
				log().closeAll();
				exit(1);
			} else {
				return; // Wait for the next sensor_ request
			}
		}
		
		switch(header_) {
			case 0x01:	// Camera.				
				bytes_recv_ = 0;
				while(bytes_recv_ < sizeof(this->image_)) {
					ret_value = recv(fd_, (char *)&this->image_[bytes_recv_], sizeof(this->image_)-bytes_recv_, 0);
					if(ret_value <= 0) {
						closeConnection();
						if(init() == false) {
							std::cerr << "Lost connection with the robot" << std::endl;
							log().closeAll();
							exit(1);
						} else {
							return; // Wait for the next sensor_ request
						}
					} else {
						bytes_recv_ += ret_value;
						//std::cout << "image read = " << bytes_recv << std::endl;
					}
				}
				
				if(DEBUG_UPDATE_SENSORS_DATA)std::cout << "[" << robot().ip  << "] " << "camera read correctly" << std::endl;
				new_image_received_ = true;


				
				//red = image_[0] & 0xf8;
				//green = image_[0] << 5;
				//green += (image_[1] & 0xf8) >> 3;
				//blue = image_[1] << 3;
				//printf("1st pixel = %d, %d, %d\r\n", red, green, blue);
				break;
			
			case 0x02: // Sensors.
				bytes_recv_ = 0;
				while(bytes_recv_ < sizeof(sensor_)) {
					ret_value = recv(fd_, (char *)&sensor_[bytes_recv_], sizeof(sensor_)-bytes_recv_, 0);
					if(ret_value <= 0) { 
						closeConnection();
						if(init() == false ) {
							std::cerr << "Lost connection with the robot" << std::endl;
							log().closeAll();
							exit(1);
						} else {
							return; // Wait for the next sensor_ request
						}
					} else {
						bytes_recv_ += ret_value;
						//std::cout << "sensors read = " << bytes_recv << std::endl;
					}
				}

                acc_data_[0] = sensor_[0] + sensor_[1]*256;
                acc_data_[1] = sensor_[2] + sensor_[3]*256;
                acc_data_[2] = sensor_[4] + sensor_[5]*256;			
				if(DEBUG_UPDATE_SENSORS_DATA)std::cout << "[" << robot().ip  << "] " << "acc: " << acc_data_[0] << "," << acc_data_[1] << "," << acc_data_[2] << std::endl;

				// Compute acceleration
				mantis = (sensor_[6] & 0xff) + ((sensor_[7] & 0xffl) << 8) + (((sensor_[8] &0x7fl) | 0x80) << 16);
				exp = (sensor_[9] & 0x7f) * 2 + ((sensor_[8] & 0x80) ? 1 : 0);
				if (sensor_[9] & 0x80) {
					mantis = -mantis;
				}
				flt = (mantis || exp) ? ((float) ldexp (mantis, (exp - 127 - 23))): 0;
				acceleration_=flt;
				if(DEBUG_UPDATE_SENSORS_DATA)std::cout << "[" << robot().ip  << "] " << "acceleration: " << acceleration_ << std::endl;

				// Compute orientation.
				mantis = (sensor_[10] & 0xff) + ((sensor_[11] & 0xffl) << 8) + (((sensor_[12] &0x7fl) | 0x80) << 16);
				exp = (sensor_[13] & 0x7f) * 2 + ((sensor_[12] & 0x80) ? 1 : 0);
				if (sensor_[13] & 0x80)
					mantis = -mantis;
				flt = (mantis || exp) ? ((float) ldexp (mantis, (exp - 127 - 23))): 0;
				orientation_=flt;
				if (orientation_ < 0.0 )
					orientation_=0.0;
				if (orientation_ > 360.0 )
					orientation_=360.0;
				if(DEBUG_UPDATE_SENSORS_DATA)std::cout << "[" << robot().ip  << "] " << "orientation: " << orientation_ << std::endl;

				// Compute inclination.
				mantis = (sensor_[14] & 0xff) + ((sensor_[15] & 0xffl) << 8) + (((sensor_[16] &0x7fl) | 0x80) << 16);
				exp = (sensor_[17] & 0x7f) * 2 + ((sensor_[16] & 0x80) ? 1 : 0);
				if (sensor_[17] & 0x80)
					mantis = -mantis;
				flt = (mantis || exp) ? ((float) ldexp (mantis, (exp - 127 - 23))): 0;
				inclination_=flt;
				if (inclination_ < 0.0 )
					inclination_=0.0;
				if (inclination_ > 180.0 )
					inclination_=180.0;
				if(DEBUG_UPDATE_SENSORS_DATA)std::cout << "[" << robot().ip  << "] " << "inclination: " << inclination_ << std::endl;

				// Gyro
				gyro_raw_[0] = sensor_[18]+sensor_[19]*256;
				gyro_raw_[1] = sensor_[20]+sensor_[21]*256;
				gyro_raw_[2] = sensor_[22]+sensor_[23]*256;
				if(DEBUG_UPDATE_SENSORS_DATA)std::cout << "[" << robot().ip  << "] " << "gyro: " << gyro_raw_[0] << "," << gyro_raw_[1] << "," << gyro_raw_[2] << std::endl;					

				// Magnetometer
				magnetic_field_[0] = *((float*)&sensor_[24]);
				magnetic_field_[1] = *((float*)&sensor_[28]);
				magnetic_field_[2] = *((float*)&sensor_[32]);
				if(DEBUG_UPDATE_SENSORS_DATA)std::cout << "[" << robot().ip  << "] " << "mag: " << magnetic_field_[0] << "," << magnetic_field_[1] << "," << magnetic_field_[2] << std::endl;	

				// Temperature.
				temperature_ = sensor_[36];
				if(DEBUG_UPDATE_SENSORS_DATA)std::cout << "[" << robot().ip  << "] " << "temperature: " << (int)temperature_ << std::endl;

				// Proximity sensors data.
				robot().proximity_sensors.ir[0] = sensor_[37]+sensor_[38]*256;
				robot().proximity_sensors.ir[1] = sensor_[39]+sensor_[40]*256;
				robot().proximity_sensors.ir[2] = sensor_[41]+sensor_[42]*256;
				robot().proximity_sensors.ir[3] = sensor_[43]+sensor_[44]*256;
				robot().proximity_sensors.ir[4] = sensor_[45]+sensor_[46]*256;
				robot().proximity_sensors.ir[5] = sensor_[47]+sensor_[48]*256;
				robot().proximity_sensors.ir[6] = sensor_[49]+sensor_[50]*256;
				robot().proximity_sensors.ir[7] = sensor_[51]+sensor_[52]*256;

				if(DEBUG_UPDATE_SENSORS_DATA)std::cout << "[" << robot().ip  << "] " << "prox: " << robot().proximity_sensors.ir[0] << "," << robot().proximity_sensors.ir[1] << "," << robot().proximity_sensors.ir[2] << "," << robot().proximity_sensors.ir[3] << "," << robot().proximity_sensors.ir[4] << "," << robot().proximity_sensors.ir[5] << "," << robot().proximity_sensors.ir[6] << "," << robot().proximity_sensors.ir[7] << std::endl;							

				// Compute abmient light.
				light_avg_ += (sensor_[53]+sensor_[54]*256);
				light_avg_ += (sensor_[55]+sensor_[56]*256);
				light_avg_ += (sensor_[57]+sensor_[58]*256);
				light_avg_ += (sensor_[59]+sensor_[60]*256);
				light_avg_ += (sensor_[61]+sensor_[62]*256);
				light_avg_ += (sensor_[63]+sensor_[64]*256);
				light_avg_ += (sensor_[65]+sensor_[66]*256);
				light_avg_ += (sensor_[67]+sensor_[68]*256);
				light_avg_ = (int) (light_avg_/8);
				light_avg_ = (light_avg_>4000)?4000:light_avg_;
				if(light_avg_<0) {
					light_avg_=0;
				}
				if(DEBUG_UPDATE_SENSORS_DATA)std::cout << "[" << robot().ip << "] " << "lightAvg: " << light_avg_ << std::endl;
				
				// ToF
				distance_cm_ = (uint16_t)(((uint8_t)sensor_[70]<<8)|((uint8_t)sensor_[69]))/10;
				if(distance_cm_ > 200) {
					distance_cm_ = 200;
				}
				if(DEBUG_UPDATE_SENSORS_DATA)std::cout << "[" << robot().ip  << "] " << "distanceCm: " << distance_cm_ << "(" << (int)sensor_[69] << "," << (int)sensor_[70] << ")" << std::endl;

				// Microphone
				mic_volume_[0] = ((uint8_t)sensor_[71]+(uint8_t)sensor_[72]*256);
				mic_volume_[1] = ((uint8_t)sensor_[73]+(uint8_t)sensor_[74]*256);
				mic_volume_[2] = ((uint8_t)sensor_[75]+(uint8_t)sensor_[76]*256);
				mic_volume_[3] = ((uint8_t)sensor_[77]+(uint8_t)sensor_[78]*256);
				if(DEBUG_UPDATE_SENSORS_DATA)std::cout << "[" << robot().ip  << "] " << "mic: " << mic_volume_[0] << "," << mic_volume_[1] << "," << mic_volume_[2] << "," << mic_volume_[3] << std::endl;

				// Left steps
				motor_steps_[0] = (double)(sensor_[79]+sensor_[80]*256);
				// Right steps
				motor_steps_[1] = (double)(sensor_[81]+sensor_[82]*256);
				//if(DEBUG_UPDATE_SENSORS_DATA)std::cout << "[" << robot().ip  << "] " << "steps: " << motorSteps[0] << "," << motorSteps[1] << std::endl;

				// Battery
				battery_raw_ = (uint8_t)sensor_[83]+(uint8_t)sensor_[84]*256;
				if(DEBUG_UPDATE_SENSORS_DATA)std::cout << "[" << robot().ip  << "] " << "batteryRaw: " << battery_raw_ << std::endl;
				
				// Micro sd state.
				micro_sd_state_ = sensor_[85];
				if(DEBUG_UPDATE_SENSORS_DATA)std::cout << "[" << robot().ip  << "] " << "microSdState: " << (int)micro_sd_state_ << std::endl;

				// Tv remote.
				ir_check_ = sensor_[86];
				ir_address_ = sensor_[87];
				ir_data_ = sensor_[88];
				if(DEBUG_UPDATE_SENSORS_DATA)std::cout << "[" << robot().ip  << "] " << "tv remote: " << (int)ir_check_ << "," << (int)ir_address_ << "," << (int)ir_data_ << std::endl;

				// Selector.
				selector_ = sensor_[89];
				if(DEBUG_UPDATE_SENSORS_DATA)std::cout << "[" << robot().ip  << "] " << "selector: " << (int)selector_ << std::endl;

				// Ground sensor_ proximity.
				ground_prox_[0] = sensor_[90]+sensor_[91]*256;
				ground_prox_[1] = sensor_[92]+sensor_[93]*256;
				ground_prox_[2] = sensor_[94]+sensor_[95]*256;
				if(DEBUG_UPDATE_SENSORS_DATA)std::cout << "[" << robot().ip  << "] " << "groundProx: " << ground_prox_[0] << "," << ground_prox_[1] << "," << ground_prox_[2] << std::endl;

				// Ground sensor_ ambient light.
				ground_ambient_[0] = sensor_[96]+sensor_[97]*256;
				ground_ambient_[1] = sensor_[98]+sensor_[99]*256;
				ground_ambient_[2] = sensor_[100]+sensor_[101]*256;
				if(DEBUG_UPDATE_SENSORS_DATA)std::cout << "[" << robot().ip  << "] " << "groundAmbient: " << ground_ambient_[0] << "," << ground_ambient_[1] << "," << ground_ambient_[2] << std::endl;

				// Button state.
				button_state_ = sensor_[102];			
				if(DEBUG_UPDATE_SENSORS_DATA)std::cout << "[" << robot().ip  << "] " << "buttonState: " << (int)button_state_ << std::endl;

				break;
			
			case 0x03:
				printf("empty packet\r\n");
				break;
			
			default:
				printf("unexpected packet\r\n");
				break;
		}
		expected_recv_packets_--;
	}

	if(camera_enabled_) {
		expected_recv_packets_ = 2;
	} else {
		expected_recv_packets_ = 1;
	}
	printSensors();
	showAndSaveRobotImage();
};

void EPuckV2Driver::sendCmd() {

	command_[0] = 0x80;
	command_[1] = 0b11;	// 0b10 Sensors enabled , 0b01 Image enabled
	command_[2] = 0;		// Calibrate proximity sensors.
	command_[3] = ((int)robot().wheels_command.left_velocity)&0xFF;		// left motor LSB
	command_[4] = ((int)robot().wheels_command.left_velocity>>8)&0xFF;		// left motor MSB
	command_[5] = ((int)robot().wheels_command.right_velocity)&0xFF;	// right motor LSB
	command_[6] = ((int)robot().wheels_command.right_velocity>>8)&0xFF;	// right motor MSB
	command_[7] = 0b0; //0b111111;	// lEDs
	command_[8] = 0;		// LED2 red
	command_[9] = 0;		// LED2 green
	command_[10] = 0;	// LED2 blue
	command_[11] = 0;	// LED4 red
	command_[12] = 0;	// LED4 green
	command_[13] = 0;	// LED4 blue
	command_[14] = 0;	// LED6 red
	command_[15] = 0;	// LED6 green
	command_[16] = 0;	// LED6 blue
	command_[17] = 0;	// LED8 red
	command_[18] = 0;	// LED8 green
	command_[19] = 0;	// LED8 blue
	command_[20] = 0;	// speaker

	bytes_sent_ = 0;
	while(bytes_sent_ < sizeof(command_)) {
		bytes_sent_ += send(fd_, (char *)&command_[bytes_sent_], sizeof(command_)-bytes_sent_, 0);
	}

};

void EPuckV2Driver::RGB565toRGB888(int width, int height, unsigned char *src, unsigned char *dst) {
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

void EPuckV2Driver::showAndSaveRobotImage() {
	RGB565toRGB888(160, 120, &image_[0], robot().vision_sensors.data);
	cv::namedWindow("Camera", cv::WINDOW_NORMAL);
	cv::resizeWindow("Camera", 160*2, 120*2);
	cv::imshow("Camera", robot().vision_sensors);
	cv::waitKey(1);
	
	cv::imwrite(log().folder_ + "/image/image" +  std::string( 4 - std::to_string(cnt_iter).length(), '0').append( std::to_string(cnt_iter)) + ".png", robot().vision_sensors);

	++cnt_iter;
}

void EPuckV2Driver::printSensors() {
	positionDataCorrectionOdometryAndWheelsSpeed();
	proxDataRawValuesToMeters();

	cur_time = std::chrono::high_resolution_clock::now();
	time_since_start = cur_time - start_time;

    std::cout << COLOR_COUT_BLUE << "Iteration N "<< cnt_iter << "\n" << COLOR_COUT_RESET
              << "ePuck location :  x : " << robot().current_pose.x << ", y : " << robot().current_pose.y << ", th : " << robot().current_pose.th << "\n"
              << "Joint position [rad] :  Left : " << robot().wheels_state.left_position << ", Right : " << robot().wheels_state.right_position << "\n"
              << "Speed [rad/s]   : Left : "  << robot().wheels_state.left_velocity << ", Right : " << robot().wheels_state.right_velocity << "\n"
			  << "TimeSinceStart  : " << time_since_start.count() << "s" << std::endl;

	if(robot().proximity_sensors.ir[0] > 0) {
		std::cout << "IR0 : " << robot().proximity_sensors.ir[0] << "\n";
	} else {
		std::cout << COLOR_COUT_BLACK << "IR0 : NONE" << COLOR_COUT_RESET << "\n";
	}

	if(robot().proximity_sensors.ir[1] > 0) {
		std::cout << "IR1 : " << robot().proximity_sensors.ir[1] << "\n";
	} else {
		std::cout << COLOR_COUT_BLACK << "IR1 : NONE" << COLOR_COUT_RESET << "\n";
	}

	if(robot().proximity_sensors.ir[2] > 0) {
		std::cout << "IR2 : " << robot().proximity_sensors.ir[2] << "\n";
	} else {
		std::cout << COLOR_COUT_BLACK << "IR2 : NONE" << COLOR_COUT_RESET << "\n";
	}

	if(robot().proximity_sensors.ir[3] > 0) {
		std::cout << "IR3 : " << robot().proximity_sensors.ir[3] << "\n";
	} else {
		std::cout << COLOR_COUT_BLACK << "IR3 : NONE" << COLOR_COUT_RESET << "\n";
	}

	if(robot().proximity_sensors.ir[4] > 0) {
		std::cout << "IR4 : " << robot().proximity_sensors.ir[4] << "\n";
	} else {
		std::cout << COLOR_COUT_BLACK << "IR4 : NONE" << COLOR_COUT_RESET << "\n";
	}

	if(robot().proximity_sensors.ir[5] > 0) {
		std::cout << "IR5 : " << robot().proximity_sensors.ir[5] << "\n";
	} else {
		std::cout << COLOR_COUT_BLACK << "IR5 : NONE" << COLOR_COUT_RESET << "\n";
	}

	if(robot().proximity_sensors.ir[6] > 0) {
		std::cout << "IR6 : " << robot().proximity_sensors.ir[6] << "\n";
	} else {
		std::cout << COLOR_COUT_BLACK << "IR6 : NONE" << COLOR_COUT_RESET << "\n";
	}

	if(robot().proximity_sensors.ir[7] > 0) {
		std::cout << "IR7 : " << robot().proximity_sensors.ir[7] << "\n";
	} else {
		std::cout << COLOR_COUT_BLACK << "IR7 : NONE" << COLOR_COUT_RESET << "\n";
	}


    log().addIn(log().file_epuck_pose[0], robot().current_pose.x);
    log().addIn(log().file_epuck_pose[1], robot().current_pose.y);
    log().addIn(log().file_epuck_pose[2], robot().current_pose.th);

    log().addIn(log().file_ir[0], robot().proximity_sensors.ir[0]);
    log().addIn(log().file_ir[1], robot().proximity_sensors.ir[1]);
    log().addIn(log().file_ir[2], robot().proximity_sensors.ir[2]);
    log().addIn(log().file_ir[3], robot().proximity_sensors.ir[3]);
    log().addIn(log().file_ir[4], robot().proximity_sensors.ir[4]);
    log().addIn(log().file_ir[5], robot().proximity_sensors.ir[5]);
    log().addIn(log().file_ir[6], robot().proximity_sensors.ir[6]);
    log().addIn(log().file_ir[7], robot().proximity_sensors.ir[7]);

    log().addIn(log().file_epuck_left_wheel_position, robot().wheels_state.left_position);
    log().addIn(log().file_epuck_right_wheel_position, robot().wheels_state.right_position);

    log().addIn(log().file_epuck_left_wheel_velocity, robot().wheels_state.left_velocity);
    log().addIn(log().file_epuck_right_wheel_velocity, robot().wheels_state.right_velocity);

}

void EPuckV2Driver::proxDataRawValuesToMeters() {
	for (int i = 0;i<8;i++) {
		if(robot().proximity_sensors.ir[i]> 0) {   
			robot().proximity_sensors.ir[i] = (0.5/sqrt(robot().proximity_sensors.ir[i]))+ROBOT_RADIUS; // Transform the analog value to a distance value in meters (given from field tests).

		} else { // Sometimes the values could be negative due to the calibration, it means there is no obstacles.
			robot().proximity_sensors.ir[i] = -1;
		}
	}
}


void EPuckV2Driver::positionDataCorrectionOdometryAndWheelsSpeed() {

	if((left_steps_raw_prev_>0) && (motor_steps_[0]<0) && (abs(motor_steps_[0]-left_steps_raw_prev_)>30000)) {     // Overflow detected (positive).
		overflow_count_left_++;
	}
	if((left_steps_raw_prev_<0) && (motor_steps_[0]>0) && (abs(motor_steps_[0]-left_steps_raw_prev_)>30000)) {     // Overflow detected (negative).
		overflow_count_left_--;
	}
	motor_position_data_correct_[0] = (overflow_count_left_*65536) + motor_steps_[0];
	
	if((right_steps_raw_prev_>0) && (motor_steps_[1]<0) && (abs(motor_steps_[1]-right_steps_raw_prev_)>30000)) {     // Overflow detected (positive).
		overflow_count_right_++;
	}
	if((right_steps_raw_prev_<0) && (motor_steps_[1]>0) && (abs(motor_steps_[1]-right_steps_raw_prev_)>30000)) {     // Overflow detected (negative).
		overflow_count_right_--;
	}
	motor_position_data_correct_[1]  = (overflow_count_right_*65536) + motor_steps_[1];        
	
	left_steps_raw_prev_ = motor_steps_[0];
	right_steps_raw_prev_ = motor_steps_[1];
	
	if(DEBUG_ODOMETRY)std::cout << "[" << robot().ip << "] " << "left, right raw: " << motor_steps_[0] << ", " << motor_steps_[1] << std::endl;
	if(DEBUG_ODOMETRY)std::cout << "[" << robot().ip << "] " << "left, right raw corrected: " << robot().wheels_state.left_position  << ", " << robot().wheels_state.right_position  << std::endl;

	// Compute odometry.
	left_steps_diff_ = motor_position_data_correct_[0]*MOT_STEP_DIST - left_steps_prev_; // Expressed in meters.
	right_steps_diff_ = motor_position_data_correct_[1]*MOT_STEP_DIST - right_steps_prev_;   // Expressed in meters.
	if(DEBUG_ODOMETRY)std::cout << "[" << robot().ip << "] " << "left, right steps diff: " << left_steps_diff_ << ", " << right_steps_diff_ << std::endl;
	
	delta_theta_ = (right_steps_diff_ - left_steps_diff_)/WHEEL_DISTANCE;   // Expressed in radiant.
	delta_steps_ = (right_steps_diff_ + left_steps_diff_)/2;        // Expressed in meters.
	if(DEBUG_ODOMETRY)std::cout << "[" << robot().ip << "] " << "delta theta, steps: " << delta_theta_ << ", " << delta_steps_ << std::endl;

	x_pos_ += delta_steps_*cos(theta_ + delta_theta_/2);   // Expressed in meters.
	y_pos_ += delta_steps_*sin(theta_ + delta_theta_/2);   // Expressed in meters.
	theta_ += delta_theta_;    // Expressed in radiant.

	

	if(DEBUG_ODOMETRY)std::cout << "[" << robot().ip << "] " << "x, y, theta: " << x_pos_ << ", " << y_pos_ << ", " << theta_ << std::endl;
	
	left_steps_prev_ = motor_position_data_correct_[0]*MOT_STEP_DIST;     // Expressed in meters.
	right_steps_prev_ = motor_position_data_correct_[1]*MOT_STEP_DIST;    // Expressed in meters.

	robot().current_pose.x = x_pos_;
	robot().current_pose.y = y_pos_;
	robot().current_pose.th = intervalAdjustment(theta_);
	robot().wheels_state.left_position= intervalAdjustment(std::fmod(motor_position_data_correct_[0],1000)*(2*M_PI/1000));
	robot().wheels_state.right_position= intervalAdjustment(std::fmod(motor_position_data_correct_[1],1000)*(2*M_PI/1000));


	cur_time = std::chrono::high_resolution_clock::now();

	time_iteration = cur_time - prev_time ;

	time_ten_iter_ += time_iteration.count();
	left_steps_diff_avg_ += left_steps_diff_;
	right_steps_diff_avg_ += right_steps_diff_;

	if (cnt_iter%10 == 0){
		robot().wheels_state.left_velocity = (left_steps_diff_avg_/time_ten_iter_)/robot().parameters.wheel_radius; 
		robot().wheels_state.right_velocity = (right_steps_diff_avg_/time_ten_iter_)/robot().parameters.wheel_radius;
		left_steps_diff_avg_ = 0;
		right_steps_diff_avg_ = 0;
		time_ten_iter_ = 0;
	}

	prev_time = std::chrono::high_resolution_clock::now();


}

double EPuckV2Driver::intervalAdjustment(double val){
    if (val < 0) {
        if (val > -M_PI){
            val = std::fabs(val) + M_PI;
        } else {
            val = std::fabs(val) - M_PI;
        } 
    }
    val = std::fmod(val, 2*M_PI);

	if (val > M_PI)
        val = -val + M_PI;
	
	return val;
}
