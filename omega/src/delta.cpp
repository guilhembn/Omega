#include "omega/delta.hpp"
#include <dynamic_reconfigure/Reconfigure.h>

#define BASE_TIME 0.1

Delta::Delta(ros::NodeHandlePtr nh): Enigma(nh), s(BASE_TIME), l(BASE_TIME*3), sp(BASE_TIME), le(BASE_TIME*2), wo(BASE_TIME*6)
        , headact_("/head_traj_controller/point_head_action"){
    cameraDynReconfigurePub_ = nh_->serviceClient<dynamic_reconfigure::Reconfigure>("/camera_synchronizer_node/set_parameters");
    headact_.waitForServer();
}

std::string Delta::name(){
    return "Delta";
}

void Delta::run() {
    look();
    breakWord();
    breakWord();
    sendLong();
    sendShort();
    sendLong();
    sendShort();
    breakLetter();
    sendLong();
    sendLong();
    sendLong();
    breakLetter();
    sendLong();
    sendShort();
    sendShort();
    breakLetter();
    sendShort();
    breakWord();
}

void Delta::sendShort(){
    dynamic_reconfigure::Reconfigure srv;
    dynamic_reconfigure::Config cfg;
    dynamic_reconfigure::IntParameter p;
    cfg.ints.resize(1);
    p.name = "projector_mode";
    p.value = 3;
    cfg.ints[0] = p;
    srv.request.config = cfg;
    cameraDynReconfigurePub_.call(srv);
    s.sleep();
    breakSymbol();
}

void Delta::sendLong(){
    dynamic_reconfigure::Reconfigure srv;
    dynamic_reconfigure::Config cfg;
    dynamic_reconfigure::IntParameter p;
    cfg.ints.resize(1);
    p.name = "projector_mode";
    p.value = 3;
    cfg.ints[0] = p;
    srv.request.config = cfg;
    cameraDynReconfigurePub_.call(srv);
    l.sleep();
    breakSymbol();
}

void Delta::breakSymbol(){
    dynamic_reconfigure::Reconfigure srv;
    dynamic_reconfigure::Config cfg;
    dynamic_reconfigure::IntParameter p;
    cfg.ints.resize(1);
    p.name = "projector_mode";
    p.value = 1;
    cfg.ints[0] = p;
    srv.request.config = cfg;
    cameraDynReconfigurePub_.call(srv);
    sp.sleep();
}

void Delta::breakLetter(){
    dynamic_reconfigure::Reconfigure srv;
    dynamic_reconfigure::Config cfg;
    dynamic_reconfigure::IntParameter p;
    cfg.ints.resize(1);
    p.name = "projector_mode";
    p.value = 1;
    cfg.ints[0] = p;
    srv.request.config = cfg;
    cameraDynReconfigurePub_.call(srv);
    le.sleep();
}

void Delta::breakWord(){
    dynamic_reconfigure::Reconfigure srv;
    dynamic_reconfigure::Config cfg;
    dynamic_reconfigure::IntParameter p;
    cfg.ints.resize(1);
    p.name = "projector_mode";
    p.value = 1;
    cfg.ints[0] = p;
    srv.request.config = cfg;
    cameraDynReconfigurePub_.call(srv);
    wo.sleep();
}
void Delta::look(){
    pr2_controllers_msgs::PointHeadGoal g;
    geometry_msgs::PointStamped point;
    point.header.frame_id = "base_footprint";
    point.header.stamp = ros::Time::now();
    point.point.x = 1.5;
    point.point.y = 0.0;
    point.point.z = 0.3;
    g.target = point;
    g.pointing_frame = "high_def_frame";
    g.pointing_axis.x = 1;
    g.pointing_axis.y = 0;
    g.pointing_axis.z = 0;
    g.min_duration = ros::Duration(5.0);
    g.max_velocity = 1.0;
    headact_.sendGoal(g);
    headact_.waitForResult();
}

