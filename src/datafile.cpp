#include "datafile.hpp"
#include <sys/stat.h>

std::string Logger::fileName(){

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

    return (date_ddmmyyyy + "_" + time_hhmmss + ".txt");

}

void Logger::addIn(float x, float y ){
    if(file_.is_open()){
        file_ << x << "    " << y << "\n";
    }
}

/*
std::string createLogFolder() {
    char folder_name[128];
    time_t current_time;
    struct tm date;

    time(&current_time);
    date = *localtime(&current_time);

    // YYYY_MM_DD-hh:mm:ss
    strftime(folder_name, 128, "../logs/%Y_%m_%d-%X", &date);
    struct stat st = {0};

    std::string log_folder = folder_name;
    if (stat(log_folder.c_str(), &st) == -1) {
        mkdir(log_folder.c_str(), 0777);
    }

    std::string data_log_folder_;
    data_log_folder_ = log_folder + "/data/";
    if (stat(data_log_folder_.c_str(), &st) == -1) {
        mkdir(data_log_folder_.c_str(), 0777);
    }

    images_log_folder = log_folder + "/images/";
    if (stat(images_log_folder.c_str(), &st) == -1) {
        mkdir(images_log_folder.c_str(), 0777);
    }
    return(data_log_folder_);
}
*/