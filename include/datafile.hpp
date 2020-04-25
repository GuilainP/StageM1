#pragma once 

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <memory>
#include <ctime>
#include <cmath>
#include <bits/stdc++.h>
#include <unistd.h>


class Logger{
    public:
        Logger(){
            folderGeneration();
            std::cout << "done fld" << std::endl;
            usleep(100);
            fileGeneration();
            std::cout << "done file" << std::endl;
            }

        ~Logger();

        void folderGeneration(); // older name (current time)
        void fileGeneration();
        void addIn(std::ofstream&, double); // add data to file

        std::string folder_;
        std::ofstream file_ePuckPosition[2];
        std::ofstream file_IR[8];
};
