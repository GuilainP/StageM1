#include <iostream>
#include <string>
#include <vector>
#include <memory>

class RobotDriver{
    public:
        RobotDriver(){};
        ~RobotDriver(){};
        virtual std::string getName() = 0;
        void printName(){
            std::cout << "Ici" << std::endl;
        }
    protected:
    private:

};


class EpuckV1Driver : RobotDriver {
    public:
        EpuckV1Driver()
            : name("Unknown") {};
        EpuckV1Driver(const std::string& name)
            : name(name) {};
        ~EpuckV1Driver(){};
        std::string getName() override {
            return name;
        }
    protected:

    private:
        const std::string name;
};

class EpuckV2Driver : RobotDriver {
    public:
        EpuckV2Driver(){};
        EpuckV2Driver(const std::string& name)
            : name(name) {};
        ~EpuckV2Driver(){};
        std::string getName() override {
            return name;
        }
    protected:

    private:
        const std::string name;
};

class EpuckVrepDriver : RobotDriver {
    public:
        EpuckVrepDriver()
            : name("Unknown") {};
        EpuckVrepDriver(const std::string& name)
            : name(name) {};
        ~EpuckVrepDriver(){};
        
        std::string getName() override {
            return name;
        }
    protected:

    private:
        const std::string name;
};
