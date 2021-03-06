#include "datafile.hpp"

#include <sys/stat.h>
#include <ctime>
#include <iostream>
#include <chrono>
#include <thread>

Logger::Logger() {
    folderGeneration();
    std::this_thread::sleep_for(std::chrono::microseconds(100));
    fileGeneration();
}

Logger::~Logger() { 
    closeAll();
}

void Logger::closeAll() {

    file_epuck_pose[0].close();
    file_epuck_pose[1].close();
    file_epuck_pose[2].close();

    file_epuck_left_wheel_position.close();
    file_epuck_right_wheel_position.close();
    file_epuck_left_wheel_velocity.close();
    file_epuck_right_wheel_velocity.close();

    file_ir[0].close();
    file_ir[1].close();
    file_ir[2].close();
    file_ir[3].close();
    file_ir[4].close();
    file_ir[5].close();
    file_ir[6].close();
    file_ir[7].close();

    // EPUCKV1 AREA //

    // FROM CONTROLFUNCTIONS.CPP

    file_dx.close();
    file_dy.close();
    file_v.close();
    file_w.close();

    file_time.close();

    file_x_vision.close();
    file_y_vision.close();
    file_th_vision.close();

    file_distances.close();

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    std::cout << "Files closed" << std::endl;
}

void Logger::folderGeneration() {

    time_t current_time;
    struct tm* ti;
    time(&current_time);
    ti = localtime(&current_time);
    std::string timeString = asctime(ti);

    std::string month, day, hour, minute, sec;
    ti->tm_mon < 10 ? month = "0" + std::to_string(ti->tm_mon + 1) : month = std::to_string(ti->tm_mon +1);
    ti->tm_mday < 10 ? day = "0" + std::to_string(ti->tm_mday) : day = std::to_string(ti->tm_mday);
    ti->tm_hour < 10 ? hour = "0" + std::to_string(ti->tm_hour) : hour = std::to_string(ti->tm_hour);
    ti->tm_min < 10 ? minute = "0" + std::to_string(ti->tm_min) : minute = std::to_string(ti->tm_min);
    ti->tm_sec < 10 ? sec = "0" + std::to_string(ti->tm_sec) : sec = std::to_string(ti->tm_sec);

    // YYYY_MM_DD-hh:mm:ss
    timeString = std::to_string(ti->tm_year+1900) + '_' + 
                                month + '_' + 
                                day + '-' + 
                                hour + ':' + 
                                minute + ':' +
                                sec ;
    std::cout << timeString << std::endl;

    folder_ = "../logs/" + timeString;

    std::string dateFolder = "mkdir " + folder_;

    std::cout << dateFolder << std::endl;

    char cstr[dateFolder.size() + 1];
    dateFolder.copy(cstr, dateFolder.size() + 1);
    cstr[dateFolder.size()] = '\0';

    system(cstr);

    std::string otherFolder = "mkdir " + folder_ +
                              "/data"
                              " && mkdir " +
                              folder_ + "/image";

    cstr[otherFolder.size() + 1];
    otherFolder.copy(cstr, otherFolder.size() + 1);
    cstr[otherFolder.size()] = '\0';

    system(cstr);
}

void Logger::fileGeneration() {

    file_epuck_pose[0].open(folder_ + "/data/x.txt");
    file_epuck_pose[1].open(folder_ + "/data/y.txt");
    file_epuck_pose[2].open(folder_ + "/data/th.txt");

    file_ir[0].open(folder_ + "/data/ir0.txt");
    file_ir[1].open(folder_ + "/data/ir1.txt");
    file_ir[2].open(folder_ + "/data/ir2.txt");
    file_ir[3].open(folder_ + "/data/ir3.txt");
    file_ir[4].open(folder_ + "/data/ir4.txt");
    file_ir[5].open(folder_ + "/data/ir5.txt");
    file_ir[6].open(folder_ + "/data/ir6.txt");
    file_ir[7].open(folder_ + "/data/ir7.txt");

    file_epuck_left_wheel_position.open(folder_ + "/data/left_position.txt");
    file_epuck_right_wheel_position.open(folder_ + "/data/right_position.txt");

    file_epuck_left_wheel_velocity.open(folder_ + "/data/left_velocity.txt");
    file_epuck_right_wheel_velocity.open(folder_ + "/data/right_velocity.txt");

    // EPUCKV1 AREA //

    // FROM CONTROLFUNCTION.CPP

    file_v.open(folder_ + "/data/v.txt");
    file_w.open(folder_ + "/data/w.txt"); 
    file_dx.open(folder_ + "/data/dx.txt");
    file_dy.open(folder_ + "/data/dy.txt"); 
    file_time.open(folder_ + "/data/time.txt");

    file_x_vision.open(folder_ + "/data/xVis.txt");
    file_y_vision.open(folder_ + "/data/yVis.txt");
    file_th_vision.open(folder_ + "/data/thetaVis.txt");

    file_distances.open(folder_ + "/data/distances.txt");

}

void Logger::addIn(std::ofstream& file, double value, std::string endline) {
    if (file.is_open()) {
        file << value << endline ;
    }
}