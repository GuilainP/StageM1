#include "Robot.hpp"

Robot::Robot(){

}

Robot::~Robot(){

}

double Robot::getValueX(){
    return x_;
}
double Robot::getValueY(){
    return y_;
}
double Robot::getValueTh(){
    return theta_;
}

void Robot::setValue(double x, double y, double theta){
    x_ = x;
    y_ = y;
    theta_ = theta;
}