#include "datafile.hpp"

std::string fileName(){

    time_t current_time;
    struct tm* ti;
    time (&current_time);
    ti=localtime(&current_time);
    std::string tStr = asctime(ti);

    std::string time_hhmmss({tStr[11],tStr[12],tStr[13],
        tStr[14],tStr[15],tStr[16],
        tStr[17],tStr[18]});
    std::string date_ddmmyyyy({
        tStr[8],tStr[9],
        tStr[4],tStr[5],tStr[6],
        tStr[20], tStr[21], tStr[22], tStr[23]});

    return time_hhmmss + "_" + date_ddmmyyyy + ".txt";
}