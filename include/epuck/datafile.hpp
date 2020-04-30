#pragma once

#include <fstream>
#include <string>

class Logger {
public:
    Logger();
    ~Logger();

    void folderGeneration(); // older name (current time)
    void fileGeneration();
    void addIn(std::ofstream&, double); // add data to file

    std::string folder_;
    std::ofstream file_ePuckPose[3]; // x, y, th
    std::ofstream file_IR[8];
    std::ofstream file_ePuckLeftWheelPosition;
    std::ofstream file_ePuckRightWheelPosition;
    std::ofstream file_ePuckLeftWheelVelocity;
    std::ofstream file_ePuckRightWheelVelocity;


};
