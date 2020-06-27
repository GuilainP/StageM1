#pragma once

#include <fstream>
#include <string>

class Logger {
public:
    Logger();
    ~Logger();

    void folderGeneration(); // folder name (current time)
    void fileGeneration();
    void addIn(std::ofstream& file , double value, std::string endline="\n"); // add data to file (end line)
    void closeAll();

    std::string folder_;

    std::ofstream file_epuck_pose[3]; // x, y, th
    std::ofstream file_ir[8];
    std::ofstream file_epuck_left_wheel_position;
    std::ofstream file_epuck_right_wheel_position;
    std::ofstream file_epuck_left_wheel_velocity;
    std::ofstream file_epuck_right_wheel_velocity;

    std::ofstream file_distances;

    std::ofstream file_v;
    std::ofstream file_w;
    std::ofstream file_dx;
    std::ofstream file_dy;
    std::ofstream file_dth;
    std::ofstream file_time;

    std::ofstream file_x_vision;
    std::ofstream file_y_vision;
    std::ofstream file_th_vision;

};
