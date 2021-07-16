#ifndef THETA_HPP
#define THETA_HPP
#include "omega/enigma.hpp"
#include <actionlib/client/simple_action_client.h>
#include <pr2_controllers_msgs/PointHeadAction.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>

class Theta: public Enigma{
    public:
    Theta(ros::NodeHandlePtr nh);

    void run() override;
    std::string name() override;

    private:
    void fillArmCommand(const std::string& side, pr2_controllers_msgs::JointTrajectoryGoal& goal);

    actionlib::SimpleActionClient<pr2_controllers_msgs::PointHeadAction> headact_;
    actionlib::SimpleActionClient<pr2_controllers_msgs::JointTrajectoryAction> leftArmAct_;
    actionlib::SimpleActionClient<pr2_controllers_msgs::JointTrajectoryAction> rightArmAct_;
    actionlib::SimpleActionClient<pr2_controllers_msgs::JointTrajectoryAction> torsoLiftAct_;

    static const double STD_DURATION;
};

#endif /* THETA_HPP */
