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
    file_ePuckPose[0].close();
    file_ePuckPose[1].close();
    file_ePuckPose[2].close();

    file_ePuckLeftWheelPosition.close();
    file_ePuckRightWheelPosition.close();
    file_ePuckLeftWheelVelocity.close();
    file_ePuckRightWheelVelocity.close();

    file_IR[0].close();
    file_IR[1].close();
    file_IR[2].close();
    file_IR[3].close();
    file_IR[4].close();
    file_IR[5].close();
    file_IR[6].close();
    file_IR[7].close();
}
void Logger::folderGeneration() {

    time_t current_time;
    struct tm* ti;
    time(&current_time);
    ti = localtime(&current_time);
    std::string timeString = asctime(ti);

    std::string month;
    if(ti->tm_mon < 10){
        month = "0" + std::to_string(ti->tm_mon + 1) ;
    }

    // YYYY_MM_DD-hh:mm:ss
    timeString = std::to_string(ti->tm_year+1900) + '_' + 
           month + '_' + 
           std::to_string(ti->tm_mday) + '-' + 
           std::to_string(ti->tm_hour) + ':' + 
           std::to_string(ti->tm_min) + ':' +
           std::to_string(ti->tm_sec) ;
           
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
    file_ePuckPose[0].open(folder_ + "/data/x.txt");
    file_ePuckPose[1].open(folder_ + "/data/y.txt");
    file_ePuckPose[2].open(folder_ + "/data/th.txt");

    file_IR[0].open(folder_ + "/data/ir0.txt");
    file_IR[1].open(folder_ + "/data/ir1.txt");
    file_IR[2].open(folder_ + "/data/ir2.txt");
    file_IR[3].open(folder_ + "/data/ir3.txt");
    file_IR[4].open(folder_ + "/data/ir4.txt");
    file_IR[5].open(folder_ + "/data/ir5.txt");
    file_IR[6].open(folder_ + "/data/ir6.txt");
    file_IR[7].open(folder_ + "/data/ir7.txt");

    file_ePuckLeftWheelPosition.open(folder_ + "/data/left_position.txt");
    file_ePuckRightWheelPosition.open(folder_ + "/data/right_position.txt");

    file_ePuckLeftWheelVelocity.open(folder_ + "/data/left_velocity.txt");
    file_ePuckRightWheelVelocity.open(folder_ + "/data/right_velocity.txt");
}

void Logger::addIn(std::ofstream& file, double value) {
    if (file.is_open()) {
        file << value << "\n";
    }
}
