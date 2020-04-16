#pragma one

#include <iostream>
#include <string>
#include <vector>
#include <memory>


class Robot {
    public:
        Robot();
        ~Robot();

        double getValueX();
        double getValueY();
        double getValueTh();
        void setValue(double x, double y, double theta);
    private:
        double x_, y_, theta_;
        
};