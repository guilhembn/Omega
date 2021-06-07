#include <ros/ros.h>
#include <pr2_controllers_msgs/PointHeadAction.h>
#include <actionlib/client/simple_action_client.h>
#include "omega/enigma.hpp"
#define BASE_TIME 0.1

class Delta: public Enigma{
    public:
    Delta(ros::NodeHandlePtr nh);

    void run() override;
    std::string name() override;

    private:
    void sendLong();
    void sendShort();
    void breakSymbol();
    void breakLetter();
    void breakWord();
    void look();
    ros::ServiceClient cameraDynReconfigurePub_;
    actionlib::SimpleActionClient<pr2_controllers_msgs::PointHeadAction> headact_;
    ros::Duration s, l, sp, le, wo;
};