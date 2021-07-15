#include "omega/delta.hpp"
#include <dynamic_reconfigure/Reconfigure.h>
#include <stdexcept>

#define BASE_TIME 0.1

Delta::Delta(ros::NodeHandlePtr nh): Enigma(nh), s(BASE_TIME), l(BASE_TIME*3), sp(BASE_TIME), le(BASE_TIME*2), wo(BASE_TIME*6)
        , headact_("/head_traj_controller/point_head_action"){
    cameraDynReconfigurePub_ = nh_->serviceClient<dynamic_reconfigure::Reconfigure>("/camera_synchronizer_node/set_parameters");
    headact_.waitForServer(ros::Duration(5.0));
    if (!headact_.isServerConnected()){
        std::cout << "Quelque chose ne va pas... Lancez vous bien cet executable sur le PR2 ? Le PR2 est-il prêt (i.e. robot.launch lancé) ?" << std::endl;
        throw std::runtime_error("Arrêt du système Omega.");
    }
}

std::string Delta::name(){
    return "Delta";
}

void Delta::run() {
    if (!waitForRunStop(5.0)){
        if (!isRunStopEnabled_){
            std::cout << "Merci d'activer le 'run/stop' du robot." << std::endl;
            throw std::runtime_error("Arrêt du système Omega");
        }
    }
    look();
    breakWord();
    breakWord();
    sendShort();
    sendLong();
    sendShort();
    sendShort();
    breakLetter();
    sendShort();
    sendLong();
    breakWord();

    sendShort();
    sendLong();
    sendShort();
    breakLetter();
    sendLong();
    sendLong();
    sendLong();
    breakLetter();
    sendShort();
    sendShort();
    sendLong();
    breakLetter();
    sendLong();
    breakLetter();
    sendShort();
    breakWord();

    sendShort();
    breakLetter();
    sendShort();
    sendShort();
    sendShort();
    breakLetter();
    sendLong();
    breakWord();

    sendShort();
    breakLetter();
    sendLong();
    sendShort();
    breakLetter();
    sendLong();
    sendShort();
    sendLong();
    sendShort();
    breakLetter();
    sendLong();
    sendLong();
    sendLong();
    breakLetter();
    sendShort();
    sendLong();
    sendShort();
    breakLetter();
    sendShort();
    breakWord();

    sendShort();
    sendLong();
    sendShort();
    sendShort();
    breakLetter();
    sendLong();
    sendLong();
    sendLong();
    breakLetter();
    sendLong();
    sendShort();
    breakLetter();
    sendLong();
    sendLong();
    sendShort();
    breakLetter();
    sendShort();
    sendShort();
    sendLong();
    breakLetter();
    sendShort();
    breakWord();

    breakWord();
    breakWord();

    sendShort();
    sendShort();
    sendShort();
    sendShort();
    breakLetter();
    sendLong();
    sendLong();
    sendLong();
    breakLetter();
    sendLong();
    sendLong();
    breakLetter();
    sendShort();
    breakLetter();
    sendShort();
    sendLong();
    sendLong();
    sendShort();
    breakLetter();
    sendShort();
    sendLong();
    breakLetter();
    sendLong();
    sendLong();
    sendShort();
    breakLetter();
    sendShort();
    breakLetter();
    sendShort();
    sendShort();
    sendShort();
    breakWord();

    sendShort();
    sendLong();
    sendShort();
    sendShort();
    breakLetter();
    sendShort();
    sendLong();
    breakLetter();
    sendShort();
    sendLong();
    breakLetter();
    sendShort();
    sendShort();
    sendShort();
    breakWord();

    sendShort();
    sendShort();
    sendLong();
    sendShort();
    breakLetter();
    sendShort();
    sendLong();
    sendShort();
    breakWord();

    sendLong();
    sendLong();
    sendShort();
    breakLetter();
    sendLong();
    sendShort();
    sendShort();
    sendShort();
    breakLetter();
    sendShort();
    sendShort();
    sendLong();
    breakLetter();
    sendShort();
    sendShort();
    breakLetter();
    sendShort();
    sendShort();
    sendShort();
    breakLetter();
    sendShort();
    sendLong();
    breakLetter();
    sendLong();
    sendShort();
    breakWord();

    sendShort();
    sendLong();
    sendLong();
    breakLetter();
    sendShort();
    sendShort();
    sendShort();
    sendShort();
    breakLetter();
    sendLong();
    sendLong();
    sendLong();
    breakLetter();
    sendShort();
    sendLong();
    breakLetter();
    sendLong();
    sendLong();
    breakLetter();
    sendShort();
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

