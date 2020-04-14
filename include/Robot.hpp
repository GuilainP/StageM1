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
        double getValueZ();
        void setValue(double x, double y, double z);
    private:
        double x, y ,z;
        
};