#include "Robot.hpp"

Robot::Robot(){

}

Robot::~Robot(){

}

float Robot::getValueX(){
    return x;
}
float Robot::getValueY(){
    return y;
}
float Robot::getValueTh(){
    return theta;
}

void Robot::setValue(float x, float y, float theta){
    this->x = x;
    this->y = y;
    this->theta = theta;
}
