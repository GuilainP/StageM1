#pragma once

#include "Robot.hpp"
#include "datafile.hpp"
#include "controlFunctions.hpp"

#include <sstream>
#include <math.h>
#include <time.h>
#include <sys/time.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <opencv2/opencv.hpp>

#define COLOR_COUT_RESET "\033[0m"

#define COLOR_COUT_BLACK "\033[0;30m"
#define COLOR_COUT_RED "\033[0;31m"
#define COLOR_COUT_GREEN "\033[0;32m"
#define COLOR_COUT_YELLOW "\033[0;33m"
#define COLOR_COUT_BLUE "\033[0;34m"
#define COLOR_COUT_MAGENTA "\033[0;35m"
#define COLOR_COUT_CYAN "\033[0;36m"
#define COLOR_COUT_WHITE "\033[0;37m"

#define COLOR_COUT_BLACK_BRIGHT "\033[1;30m"
#define COLOR_COUT_RED_BRIGHT "\033[1;31m"
#define COLOR_COUT_GREEN_BRIGHT "\033[1;32m"
#define COLOR_COUT_YELLOW_BRIGHT "\033[1;33m"
#define COLOR_COUT_BLUE_BRIGHT "\033[1;34m"
#define COLOR_COUT_MAGENTA_BRIGHT "\033[1;35m"
#define COLOR_COUT_CYAN_BRIGHT "\033[1;36m"
#define COLOR_COUT_WHITE_BRIGHT "\033[1;37m"

class RobotDriver {
public:
    RobotDriver(Robot& robot, char** arg) : robot_(robot), argv(arg) {
    }
    virtual ~RobotDriver() = default;

    virtual bool init() = 0;
    virtual void read() = 0;
    virtual void sendCmd() = 0;
    virtual void getVisionSensor(Robot& robot) = 0;
    cv::Mat showAndSaveRobotImage(unsigned char* imgData, const int& cnt_it);

    Robot& robot();
    Logger& log();

protected:
    int cnt_iter;
    char** argv;

private:
    Robot& robot_;
    Logger log_; 
};

class EPuckV1Driver : public RobotDriver {
public:
    EPuckV1Driver(Robot& robot, char** arg);
    virtual ~EPuckV1Driver();

    bool init() override;
    void read() override;
    void sendCmd() override;
    void getVisionSensor(Robot& robot) override ; // Empty (not needed in EPuckV1Driver)

private:
    void initCamera(const std::string& epuck_ip);
    void initSocketOpening(const std::string& epuck_ip);
    void openCameraSocket();
    void openSensorReceivingSocket();
    void setWheelCommands(struct timeval startTime, const int& speed_L, const int& speed_R, char MotorCmd[15]);
    void saveData(Logger& log);
    void* cameraReceptionThread(void* arg);
    void splitSensorMeasures();
    void receiveSensorMeasures();
    void sendMotorAndLEDCommandToRobot(const char MotorCmd[15]);
    void closeSocket(int NOM_SOCKET);
    void openCommandSendingSocket(const std::string& epuck_ip);
    void incorrectArguments(const int& argc);

    /* Variables of main thread */

    typedef int SOCKET;
    SOCKET camera_socket, command_sending_socket, sensor_receiving_socket;
    char all_sensors[100]; // Buffer de reception des capteurs
    int encoder_left, encoder_right, prev_encoder_left, prev_encoder_right;
    int prox_sensors[10];
    bool stop_threads;

    enum Controller { setWheelCmd, setVel, setRobVel, followWall, visServo };
    Controller c;

    struct timeval startTime, curTime, prevTime;
    double timeSinceStart;
    char MotorCommand[15];      // command for the two motors
    cv::Mat robImg;
    Pose prevPoseFromEnc, curPoseFromEnc, prevPoseFromVis, curPoseFromVis, initPose;

    int speedLeft, speedRight;
    float vel, omega;

    /* Variables for threads and sockets */
    typedef struct sockaddr_in SOCKADDR_IN;
    SOCKADDR_IN sockaddrin_init, local_sockaddrin_init, sockaddrin_camera,
        sockaddrin_reception_capteurs, local_sockaddrin_envoie_commandes,
        sockaddrin_envoie_commandes;
    SOCKET sock_init;
    typedef struct sockaddr SOCKADDR;

    // Camera variables
    bool camera_active;
    struct imageData {
        int id_img;
        int id_block;
        unsigned long date;
        unsigned char msg[230400];
    } img_data;

    pthread_t IDCameraReceptionThread; 

    static void *cameraThreadFunc(void * p) {
        EPuckV1Driver *a = (EPuckV1Driver *)p; // cast *p to EPuckV1Driver class type
        a->cameraReceptionThread(nullptr);     // access EPuckV1Driver's non-static member CameraReceptionThread()
        return NULL;                           // do some work and return
    }
};

class EPuckV2Driver : public RobotDriver {
public:
    EPuckV2Driver(Robot& robot, char** arg);
    virtual ~EPuckV2Driver();

    bool init() override;
    void read() override;
    void sendCmd() override;
    
    void getVisionSensor(Robot& robot) override ;
    
    


private:
    void printSensors();
    void closeConnection();
    void proxDataRawValuesToMeters();
    void positionDataCorrection();
    void odometry();

    unsigned char image_[160*120*2];

    // Communication variables
    struct sockaddr_in robot_addr_;
    int fd_;
    unsigned char command_[21];
    unsigned char header_, sensor_[104];
    int bytes_sent, bytes_recv ;
    bool camera_enabled, ground_sensors_enabled;
    uint8_t expected_recv_packets;
    bool new_image_received_;

    // EPuckV2 data variables

    int16_t acc_data_[3];
    int16_t motor_steps_[2];

    double left_steps_diff_, right_steps_diff_;
    double left_steps_prev_, right_steps_prev_;
    signed long int left_steps_raw_prev_, right_steps_raw_prev_;
    signed long int motor_position_data_correct_[2];
    double delta_steps_, delta_theta_;

    int overflow_count_left_, overflow_count_right_ ;
    int16_t gyro_offset_[3] ; // Used if making an initial calibration of the gyro.

        /*** ADDITIONNAL DATA FOR EPUCK V2 */

    // Sensors data variables

    float acceleration, orientation, inclination;/**< acceleration data*/

    int16_t gyro_raw[3];
    float magnetic_field[3];
    uint8_t temperature;
    int light_avg;		/**< light sensor data*/
    
    uint16_t distance_cm;
    uint16_t mic_volume[4];	/**< microphone data*/
    uint16_t battery_raw;
    uint8_t micro_sd_state;
    uint8_t ir_check, ir_address, ir_data;
    uint8_t selector;
    int16_t ground_prox[3], ground_ambient[3];
    uint8_t button_state;

};

class EPuckVREPDriver : public RobotDriver {
public:
    EPuckVREPDriver(Robot& robot, char** arg);
    virtual ~EPuckVREPDriver();

    bool init() override;
    void read() override;
    void sendCmd() override;
    void getVisionSensor(Robot& robot) override ;

    void setVelocity(double, double);


private:

    void printSensors();
    void dataToRobot();
    
    int client_id_, ping_time_;
    int epuck_handle_;
    int right_joint_handle_, left_joint_handle_;
    int prox_sensors_handle_[8];
    int vision_handle_;
    int res_[2];
    float epuck_position_[3];
    float left_joint_position_, right_joint_position_;
    float left_joint_velocity_[3], right_joint_velocity_[3];
    float euler_angles_[3];
    uint8_t detection_state_ir_[8];
    float detected_point_ir_[8][3];
    uint8_t* sim_image_;
    int ajouter_;
    
    double minl_,maxl_,minr_,maxr_;
};
