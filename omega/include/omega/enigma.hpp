#ifndef ENIGMA_HPP
#define ENIGMA_HPP
#include <ros/ros.h>
#include <std_msgs/Bool.h>

class Enigma{
    public:
    Enigma(ros::NodeHandlePtr nh): nh_(nh), isRunStopEnabled_(false){
        runStopPub_ = nh_->subscribe("/pr2_ethercat/motors_halted", 1, &Enigma::onRunStop, this);
    };

    void onRunStop(const std_msgs::BoolConstPtr& msg){
        bool motorHalted = msg->data;
        if (isRunStopEnabled_ && motorHalted){
            std::cout << "Désactivation du run/stop détectée." << std::endl;
            throw std::runtime_error("Arrêt du système Omega");
        }
        isRunStopEnabled_ = !motorHalted;
    }

    virtual void run() = 0;
    virtual std::string name() = 0;

    protected:
    bool waitForRunStop(double timeout){
        for (unsigned int i=0; i<timeout*100; i++){
            if (isRunStopEnabled_){
                break;
            }
            ros::Duration(0.01).sleep();
            ros::spinOnce();
        }
        return isRunStopEnabled_;
    }

    ros::NodeHandlePtr nh_;
    ros::Subscriber runStopPub_;
    bool isRunStopEnabled_;
};
#endif /* ENIGMA_HPP */
