#pragma one

#include "Robot.hpp"

class RobotDriver{
    public:
        RobotDriver();
        ~RobotDriver();

        virtual void printName() = 0;
//        virtual void init() = 0;
//        virtual void read() = 0;
//        virtual void moveRobot(Robot) = 0;
//        virtual void send() = 0;
    private:
};


class EPuckV1Driver : public RobotDriver {
    public:
        EPuckV1Driver(Robot);
        ~EPuckV1Driver();

        void printName() override;
    private:
        std::string name;
        Robot robot;
}; 

class EPuckV2Driver : public RobotDriver {
    public:
        EPuckV2Driver(Robot);
        ~EPuckV2Driver();

        void printName() override;
    private:
        std::string name;
        Robot robot;
}; 

class EPuckVREPDriver : public RobotDriver {
    public:
        EPuckVREPDriver(Robot);
        ~EPuckVREPDriver();

        void printName() override;
    private:
        std::string name;
        Robot robot;
}; 

