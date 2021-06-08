#include "omega/theta.hpp"
#include <stdexcept>

Theta::Theta(ros::NodeHandlePtr nh): Enigma(nh), headact_("/head_traj_controller/point_head_action"){
    headact_.waitForServer(ros::Duration(5.0));
    if (!headact_.isServerConnected()){
        std::cout << "Quelque chose ne va pas... Lancez vous bien cet executable sur le PR2 ? Le PR2 est-il prêt (i.e. robot.launch lancé) ?" << std::endl;
        throw std::runtime_error("Arrêt du système Omega.");
    }
}

std::string Theta::name(){
    return "Theta";
}

void Theta::run() {
    
}