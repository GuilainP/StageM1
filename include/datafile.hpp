#pragma once 

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <memory>
#include <ctime>
#include <cmath>
#include <bits/stdc++.h>


class Logger{
    public:
        Logger(){
            file_.open(fileName());
            system("mkdir ../logs/BONJOUR");
            
            }

        ~Logger(){file_.close();}

        std::string fileName(); // file name (current time)
        void addIn(float x, float y ); // add data to file
        std::ofstream file_;
        std::ofstream file_ir[8];
};
