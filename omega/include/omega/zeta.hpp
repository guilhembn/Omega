#ifndef ZETA_HPP
#define ZETA_HPP
#include "omega/enigma.hpp"
#include <image_transport/image_transport.h>
#include <opencv2/aruco.hpp>
#include <actionlib/client/simple_action_client.h>
#include <pr2_controllers_msgs/PointHeadAction.h>
#include <std_msgs/Bool.h>

class Zeta: public Enigma{
    public:
    Zeta(ros::NodeHandlePtr nh);

    void run() override;
    std::string name() override;

    void imageCb(const sensor_msgs::ImageConstPtr& msg);

    private:
    float getAngle(const std::vector<cv::Point2f>& markerCorner) const;
    static float centerAngle(float angle);
    void say(const std::string& speech);
    std::string demangleString(const std::vector<uint8_t>& str);
    void look();    

    actionlib::SimpleActionClient<pr2_controllers_msgs::PointHeadAction> headact_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber imgSub_;
    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    bool capture_;
    char lastAnswer_;
    ros::Publisher sayPub_;

    static const std::vector<std::vector<uint8_t>> questions;
    static const std::vector<std::vector<std::vector<uint8_t>>> answers;
    static const std::vector<size_t> cor;
    static const std::vector<uint8_t> sec;

};

#endif /* ZETA_HPP */
