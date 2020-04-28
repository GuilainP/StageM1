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
    std::ofstream file_ePuckPosition[2];
    std::ofstream file_IR[8];
};
