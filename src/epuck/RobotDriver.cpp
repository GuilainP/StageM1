#include "RobotDriver.hpp"

Robot& RobotDriver::robot() {
    return robot_;
}

Logger& RobotDriver::log() {
    return log_;
}

/**** Show image from the robot camera and save it on the disk ****/
cv::Mat RobotDriver::showAndSaveRobotImage(unsigned char* imgData, const int& cnt_it) {
    cv::Mat rawImg =  cv::Mat(240, 320, CV_8UC3, imgData);
    cv::Mat img;
    cv::cvtColor(rawImg, img, cv::COLOR_RGB2BGRA); 
    //show the image
    cv::namedWindow( "Robot image", cv::WINDOW_AUTOSIZE ); // Create a window for display.
    cv::moveWindow( "Robot image", 0,0); // Move window 
    cv::imshow( "Robot image", img );                // Show the image inside it.
    cv::waitKey(1);
    //save the image on disk
    cv::imwrite(log().folder_ + "/image/image" +  std::string( 4 - std::to_string(cnt_it).length(), '0').append( std::to_string(cnt_it)) + ".png", img);
    return(img);
}
