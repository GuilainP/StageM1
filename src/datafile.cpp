#include "datafile.hpp"
#include <sys/stat.h>

Logger::~Logger(){
    file_ePuckPosition[0].close();
    file_ePuckPosition[1].close();
    file_IR[0].close();
    file_IR[1].close();
    file_IR[2].close();
    file_IR[3].close();
    file_IR[4].close();
    file_IR[5].close();
    file_IR[6].close();
    file_IR[7].close();
}
void Logger::folderGeneration(){

    time_t current_time;
    struct tm* ti;
    time (&current_time);
    ti=localtime(&current_time);
    std::string tStr = asctime(ti);

    std::string time_hhmmss({tStr[11],tStr[12],
        tStr[14],tStr[15],
        tStr[17],tStr[18]});
    std::string date_ddmmyyyy({
        tStr[8],tStr[9],
        tStr[4],tStr[5],tStr[6],
        tStr[20], tStr[21], tStr[22], tStr[23]});

    folder_ = "../logs/" + date_ddmmyyyy + "_" + time_hhmmss ;

    std::string dateFolder = "mkdir " + folder_;

    std::cout << dateFolder << std::endl;

    char cstr[dateFolder.size() +1];
    dateFolder.copy(cstr, dateFolder.size() +1);
    cstr[dateFolder.size()] = '\0';

    system(cstr);

    std::string otherFolder = "mkdir " + folder_ + "/prox"
    " && mkdir " + folder_ + "/image";

    cstr[otherFolder.size() +1];
    otherFolder.copy(cstr, otherFolder.size() +1);
    cstr[otherFolder.size()] = '\0';

    system(cstr);
}

void Logger::fileGeneration(){
    file_ePuckPosition[0].open(folder_+ "/prox/x.txt");
    file_ePuckPosition[1].open(folder_+ "/prox/y.txt");
    file_IR[0].open(folder_ + "/prox/ir0.txt");
    file_IR[1].open(folder_ + "/prox/ir1.txt");
    file_IR[2].open(folder_ + "/prox/ir2.txt");
    file_IR[3].open(folder_ + "/prox/ir3.txt");
    file_IR[4].open(folder_ + "/prox/ir4.txt");
    file_IR[5].open(folder_ + "/prox/ir5.txt");
    file_IR[6].open(folder_ + "/prox/ir6.txt");
    file_IR[7].open(folder_ + "/prox/ir7.txt");

}

void Logger::addIn(std::ofstream& file, double value){
    if(file.is_open()){
        file << value <<"\n";
    }
}

