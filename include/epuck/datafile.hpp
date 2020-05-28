#pragma once

#include <fstream>
#include <string>

class Logger {
public:
    Logger();
    ~Logger();

    void folderGeneration(); // folder name (current time)
    void fileGeneration();
    void addIn(std::ofstream& file , double value, std::string endline=""); // add data to file (end line)
    void CloseAll();

    std::string folder_;

    std::ofstream file_epuck_pose[3]; // x, y, th
    std::ofstream file_ir[8];
    std::ofstream file_epuck_left_wheel_position;
    std::ofstream file_epuck_right_wheel_position;
    std::ofstream file_epuck_left_wheel_velocity;
    std::ofstream file_epuck_right_wheel_velocity;

    // FOR EPUCKV1 UNTOUCHED
    std::ofstream file_v;
    std::ofstream file_w;
    std::ofstream file_dx;
    std::ofstream file_dy;
    std::ofstream file_dth;
    std::ofstream file_x;
    std::ofstream file_y;
    std::ofstream file_th;
    std::ofstream file_time;

    std::ofstream file_x_vision;
    std::ofstream file_y_vision;
    std::ofstream file_th_vision;

    std::ofstream file_distances;

    std::ofstream file_eg;
    std::ofstream file_ed;
    std::ofstream file_p0;
    std::ofstream file_p1;
    std::ofstream file_p2;
    std::ofstream file_p3;
    std::ofstream file_p4;
    std::ofstream file_p5;
    std::ofstream file_p6;
    std::ofstream file_p7;
    std::ofstream file_p8;
    std::ofstream file_p9;

    
};
