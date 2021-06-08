#ifndef THETA_HPP
#define THETA_HPP
#include "omega/enigma.hpp"
#include <actionlib/client/simple_action_client.h>
#include <pr2_controllers_msgs/PointHeadAction.h>

class Theta: public Enigma{
    public:
    Theta(ros::NodeHandlePtr nh);

    void run() override;
    std::string name() override;

    private:
    actionlib::SimpleActionClient<pr2_controllers_msgs::PointHeadAction> headact_;
};

#endif /* THETA_HPP */
