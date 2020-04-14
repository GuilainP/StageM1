#include "Robot.hpp"

Robot::Robot(){

}

Robot::~Robot(){

}

double Robot::getValueX(){
    return x;
}
double Robot::getValueY(){
    return y;
}
double Robot::getValueZ(){
    return z;
}

void Robot::setValue(double x, double y, double z){
    this->x = x;
    this->y = y;
    this->z = z;
}