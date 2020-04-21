#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <memory>
#include <ctime>
#include <cmath>


class Logger{
    public:
        Logger(){file_.open(fileName());}
        ~Logger(){file_.close();}

        std::string fileName(); // file name (current time)
        void addIn(float x, float y ); // add data to file
        std::ofstream file_;
};
