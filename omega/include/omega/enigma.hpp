#ifndef ENIGMA_HPP
#define ENIGMA_HPP
#include <ros/ros.h>

class Enigma{
    public:
    Enigma(ros::NodeHandlePtr nh): nh_(nh){
    }

    virtual void run() = 0;
    virtual std::string name() = 0;

    protected:
    ros::NodeHandlePtr nh_;
};
#endif /* ENIGMA_HPP */
