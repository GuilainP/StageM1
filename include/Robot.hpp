#pragma one

#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include <cmath>

struct robotParameters {
    const float theta[10] = {-0.2268, -0.8371, -1.5708, -2.2391, 2.2391, 1.5708,  0.8371,  0.2268,  0.35,    -0.35};
    const float robRadius = 0.033;
    const float wheel_radius = 0.021;
    const float axisLength = 0.052;
    const float distMax = 0.035;
};


class Robot {
    public:
        Robot();
        ~Robot();

        float getValueX();
        float getValueY();
        float getValueTh();
        void setValue(float x, float y, float theta);
    private:
        robotParameters rb;
        float x, y, theta;
        
};