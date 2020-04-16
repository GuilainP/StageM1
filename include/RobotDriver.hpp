#pragma one

#include "Robot.hpp"

class RobotDriver{
    public:
        RobotDriver(Robot& robot) : robot_(robot) {}
        virtual ~RobotDriver() = default;

    virtual void init() = 0;
    virtual void read() = 0;
    // virtual void moveRobot(Robot) = 0;
    virtual void send() = 0;
    private:
        Robot& robot_;
        std::string name_;
};

class EPuckV1Driver : public RobotDriver {
    public:
        EPuckV1Driver(Robot& robot) : RobotDriver(robot) {}
        virtual ~EPuckV1Driver() = default;

        void init() override ;// TODO}
        void read() override ;// TODO}
        void send() override ;// TODO}

    private:
        std::string IP;
        std::string port;
};

class EPuckV2Driver : public RobotDriver {
    public:
        EPuckV2Driver(Robot& robot) : RobotDriver(robot) {}
        virtual ~EPuckV2Driver() = default;

        void init() override ;// TODO}
        void read() override ;// TODO}
        void send() override ;// TODO}

    private:
        std::string IP;
        std::string port;
};

class EPuckVREPDriver : public RobotDriver {
    public:
        EPuckVREPDriver(Robot& robot) : RobotDriver(robot) {}
        virtual ~EPuckVREPDriver() = default;

        void init() override ;// TODO}
        void read() override ;// TODO}
        void send() override ;// TODO}

    private:
        std::string Portname;
};