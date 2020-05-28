#pragma once

#include "Robot.hpp"
#include "datafile.hpp"

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
    RobotDriver(Robot& robot) : robot_(robot) {
    }
    virtual ~RobotDriver() = default;

    virtual bool Init() = 0;
    virtual void Read() = 0;
    virtual void Send() = 0;
    virtual void getVisionSensor(Robot& robot) = 0;

    Robot& robot();
    Logger& log();

private:
    Robot& robot_;
    Logger log_; 
};

class EPuckV1Driver : public RobotDriver {
public:
    EPuckV1Driver(Robot& robot);
    virtual ~EPuckV1Driver() = default;

    bool Init() override;
    void Read() override;
    void Send() override;
    void getVisionSensor(Robot& robot) override ;

private:
    std::string robotIP;
    std::string robotID;
};

class EPuckV2Driver : public RobotDriver {
public:
    EPuckV2Driver(Robot& robot);
    virtual ~EPuckV2Driver();

    bool Init() override;
    void Read() override;
    void Send() override;
    
    void getVisionSensor(Robot& robot) override ;
    void PrintSensors();
    void closeConnection();
    void proxDataRawValuesToMeters();
    void positionDataCorrection();
    void odometry();


private:

    unsigned char image_[160*120*2];
    int ajouter_;

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


};

class EPuckVREPDriver : public RobotDriver {
public:
    EPuckVREPDriver(Robot& robot);
    virtual ~EPuckVREPDriver();

    bool Init() override;
    void Read() override;
    void Send() override;
    void getVisionSensor(Robot& robot) override ;

    void setVelocity(double, double);
    void PrintSensors();
    void dataToRobot();


private:
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
