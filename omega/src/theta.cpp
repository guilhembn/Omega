#include "omega/theta.hpp"
#include <stdexcept>

const double Theta::STD_DURATION = 3.0;

Theta::Theta(ros::NodeHandlePtr nh): Enigma(nh), headact_("/head_traj_controller/point_head_action"), 
                rightArmAct_("/r_arm_controller/joint_trajectory_action"), leftArmAct_("/l_arm_controller/joint_trajectory_action"),
                torsoLiftAct_("/torso_controller/joint_trajectory_action"){
    headact_.waitForServer(ros::Duration(5.0));
    if (!headact_.isServerConnected()){
        std::cout << "Quelque chose ne va pas... Lancez vous bien cet executable sur le PR2 ? Le PR2 est-il prêt (i.e. robot.launch lancé) ?" << std::endl;
        throw std::runtime_error("Arrêt du système Omega.");
    }
    rightArmAct_.waitForServer(ros::Duration(2.5));
    leftArmAct_.waitForServer(ros::Duration(2.5));
    if (!rightArmAct_.isServerConnected() || !leftArmAct_.isServerConnected()){
        std::cout << "Quelque chose ne va pas... Lancez vous bien cet executable sur le PR2 ? Le PR2 est-il prêt (i.e. robot.launch lancé) ?" << std::endl;
        throw std::runtime_error("Arrêt du système Omega.");
    }
}

std::string Theta::name(){
    return "Theta";
}

void Theta::run() {
    if (!waitForRunStop(5.0)){
        if (!isRunStopEnabled_){
            std::cout << "Merci d'activer le 'run/stop' du robot." << std::endl;
            throw std::runtime_error("Arrêt du système Omega");
        }
    }
    std::cout << "Vérifiez que rien ne se trouve autour du robot. Les bras vont bouger." << std::endl;
    std::cout << "L'environment autour du robot est-il dégagé ? (y/n): ";
    std::string answer;
    std::cin >> answer;
    if (answer != "y"){
        std::cout << "Dégagez l'espace à proximité du robot.";
        throw std::runtime_error("Arrêt du système Omega.");
    }
    double tfs = 0.0;
    pr2_controllers_msgs::JointTrajectoryGoal leftGoal, rightGoal;
    fillArmCommand("left", leftGoal);
    fillArmCommand("right", rightGoal);
    leftGoal.trajectory.points.resize(1);
    tfs += STD_DURATION;
    leftGoal.trajectory.points[0].positions.push_back(0.0);
    leftGoal.trajectory.points[0].positions.push_back(1.109);
    leftGoal.trajectory.points[0].positions.push_back(0.0);
    leftGoal.trajectory.points[0].positions.push_back(-2.234);
    leftGoal.trajectory.points[0].positions.push_back(0.0);
    leftGoal.trajectory.points[0].positions.push_back(-1.658);
    leftGoal.trajectory.points[0].positions.push_back(0.0);
    rightGoal.trajectory.points.resize(1);
    rightGoal.trajectory.points[0].positions.push_back(0.0);
    rightGoal.trajectory.points[0].positions.push_back(1.109);
    rightGoal.trajectory.points[0].positions.push_back(0.0);
    rightGoal.trajectory.points[0].positions.push_back(-2.234);
    rightGoal.trajectory.points[0].positions.push_back(0.0);
    rightGoal.trajectory.points[0].positions.push_back(-1.658);
    rightGoal.trajectory.points[0].positions.push_back(0.0);
    for (size_t j = 0; j < leftGoal.trajectory.points[0].positions.size(); j++) {
        leftGoal.trajectory.points[0].velocities.push_back(0.0);
        rightGoal.trajectory.points[0].velocities.push_back(0.0);
    }
    leftGoal.trajectory.points[0].time_from_start = ros::Duration(tfs);
    rightGoal.trajectory.points[0].time_from_start = ros::Duration(tfs);

    pr2_controllers_msgs::JointTrajectoryGoal torsoLiftGoal;
    torsoLiftGoal.trajectory.header.stamp = ros::Time::now();
    torsoLiftGoal.trajectory.joint_names.push_back("torso_lift_joint");
    torsoLiftGoal.trajectory.points.push_back({});
    torsoLiftGoal.trajectory.points[0].positions.push_back(0.3);
    torsoLiftGoal.trajectory.points[0].time_from_start = ros::Duration(1.0);
    look();
    leftArmAct_.sendGoal(leftGoal);
    rightArmAct_.sendGoal(rightGoal);
    torsoLiftAct_.sendGoal(torsoLiftGoal);
    leftArmAct_.waitForResult();
    rightArmAct_.waitForResult();
    torsoLiftAct_.waitForResult();
    leftArmAct_.getResult();
    rightArmAct_.getResult();
    torsoLiftAct_.getResult();

    ros::Duration(2.0).sleep();

    leftGoal.trajectory.points.clear();
    rightGoal.trajectory.points.clear();

    leftGoal.trajectory.points.push_back({});
    rightGoal.trajectory.points.push_back({});
    tfs += STD_DURATION;
    int n = leftGoal.trajectory.points.size() - 1;
    leftGoal.trajectory.points[n].positions.push_back(1.57);
    leftGoal.trajectory.points[n].positions.push_back(-0.5);
    leftGoal.trajectory.points[n].positions.push_back(3.1415);
    leftGoal.trajectory.points[n].positions.push_back(-1.57);
    leftGoal.trajectory.points[n].positions.push_back(0.0);
    leftGoal.trajectory.points[n].positions.push_back(0.0);
    leftGoal.trajectory.points[n].positions.push_back(0.0);
    rightGoal.trajectory.points[n].positions.push_back(-1.57);
    rightGoal.trajectory.points[n].positions.push_back(0.5);
    rightGoal.trajectory.points[n].positions.push_back(3.1415);
    rightGoal.trajectory.points[n].positions.push_back(-1.57);
    rightGoal.trajectory.points[n].positions.push_back(0.0);
    rightGoal.trajectory.points[n].positions.push_back(0.0);
    rightGoal.trajectory.points[n].positions.push_back(0.0);
    leftGoal.trajectory.points[n].time_from_start = ros::Duration(tfs);
    rightGoal.trajectory.points[n].time_from_start = ros::Duration(tfs);

    leftGoal.trajectory.points.push_back(leftGoal.trajectory.points[n]);
    rightGoal.trajectory.points.push_back(rightGoal.trajectory.points[n]);
    tfs += STD_DURATION;
    n = leftGoal.trajectory.points.size() - 1;
    leftGoal.trajectory.points[n].time_from_start = ros::Duration(tfs);
    rightGoal.trajectory.points[n].time_from_start = ros::Duration(tfs);

    leftGoal.trajectory.points.push_back({});
    rightGoal.trajectory.points.push_back({});
    tfs += STD_DURATION;
    n = leftGoal.trajectory.points.size() - 1;
    leftGoal.trajectory.points[n].positions.push_back(1.57);
    leftGoal.trajectory.points[n].positions.push_back(0.5);
    leftGoal.trajectory.points[n].positions.push_back(3.14);
    leftGoal.trajectory.points[n].positions.push_back(-1.57);
    leftGoal.trajectory.points[n].positions.push_back(0.0);
    leftGoal.trajectory.points[n].positions.push_back(0.0);
    leftGoal.trajectory.points[n].positions.push_back(0.0);
    rightGoal.trajectory.points[n].positions.push_back(-1.57);
    rightGoal.trajectory.points[n].positions.push_back(-0.5);
    rightGoal.trajectory.points[n].positions.push_back(0.0);
    rightGoal.trajectory.points[n].positions.push_back(-1.57);
    rightGoal.trajectory.points[n].positions.push_back(0.0);
    rightGoal.trajectory.points[n].positions.push_back(0.0);
    rightGoal.trajectory.points[n].positions.push_back(0.0);
    leftGoal.trajectory.points[n].time_from_start = ros::Duration(tfs);
    rightGoal.trajectory.points[n].time_from_start = ros::Duration(tfs);
    leftGoal.trajectory.points.push_back(leftGoal.trajectory.points[n]);
    rightGoal.trajectory.points.push_back(rightGoal.trajectory.points[n]);
    tfs += 2 * STD_DURATION;
    n = leftGoal.trajectory.points.size() - 1;
    leftGoal.trajectory.points[n].time_from_start = ros::Duration(tfs);
    rightGoal.trajectory.points[n].time_from_start = ros::Duration(tfs);

    leftGoal.trajectory.points.push_back({});
    rightGoal.trajectory.points.push_back({});
    tfs += STD_DURATION;
    n = leftGoal.trajectory.points.size() - 1;
    leftGoal.trajectory.points[n].positions.push_back(1.57);
    leftGoal.trajectory.points[n].positions.push_back(0.0);
    leftGoal.trajectory.points[n].positions.push_back(0.0);
    leftGoal.trajectory.points[n].positions.push_back(0.0);
    leftGoal.trajectory.points[n].positions.push_back(0.0);
    leftGoal.trajectory.points[n].positions.push_back(0.0);
    leftGoal.trajectory.points[n].positions.push_back(0.0);
    rightGoal.trajectory.points[n].positions.push_back(-1.57);
    rightGoal.trajectory.points[n].positions.push_back(0.0);
    rightGoal.trajectory.points[n].positions.push_back(0.0);
    rightGoal.trajectory.points[n].positions.push_back(-1.57);
    rightGoal.trajectory.points[n].positions.push_back(0.0);
    rightGoal.trajectory.points[n].positions.push_back(0.0);
    rightGoal.trajectory.points[n].positions.push_back(0.0);
    leftGoal.trajectory.points[n].time_from_start = ros::Duration(tfs);
    rightGoal.trajectory.points[n].time_from_start = ros::Duration(tfs); 
    leftGoal.trajectory.points.push_back(leftGoal.trajectory.points[n]);
    rightGoal.trajectory.points.push_back(rightGoal.trajectory.points[n]);
    tfs += STD_DURATION;
    n = leftGoal.trajectory.points.size() - 1;
    leftGoal.trajectory.points[n].time_from_start = ros::Duration(tfs);
    rightGoal.trajectory.points[n].time_from_start = ros::Duration(tfs);


    leftGoal.trajectory.points.push_back({});
    rightGoal.trajectory.points.push_back({});
    tfs += STD_DURATION;
    n = leftGoal.trajectory.points.size() - 1;
    leftGoal.trajectory.points[n].positions.push_back(1.57);
    leftGoal.trajectory.points[n].positions.push_back(0.5);
    leftGoal.trajectory.points[n].positions.push_back(3.1415);
    leftGoal.trajectory.points[n].positions.push_back(-1.57);
    leftGoal.trajectory.points[n].positions.push_back(0.0);
    leftGoal.trajectory.points[n].positions.push_back(0.0);
    leftGoal.trajectory.points[n].positions.push_back(0.0);
    rightGoal.trajectory.points[n].positions.push_back(-1.57);
    rightGoal.trajectory.points[n].positions.push_back(-0.5);
    rightGoal.trajectory.points[n].positions.push_back(3.1416);
    rightGoal.trajectory.points[n].positions.push_back(-1.57);
    rightGoal.trajectory.points[n].positions.push_back(0.0);
    rightGoal.trajectory.points[n].positions.push_back(0.0);
    rightGoal.trajectory.points[n].positions.push_back(0.0);
    leftGoal.trajectory.points[n].time_from_start = ros::Duration(tfs);
    rightGoal.trajectory.points[n].time_from_start = ros::Duration(tfs);
    leftGoal.trajectory.points.push_back(leftGoal.trajectory.points[n]);
    rightGoal.trajectory.points.push_back(rightGoal.trajectory.points[n]);
    tfs += STD_DURATION;
    n = leftGoal.trajectory.points.size() - 1;
    leftGoal.trajectory.points[n].time_from_start = ros::Duration(tfs);
    rightGoal.trajectory.points[n].time_from_start = ros::Duration(tfs);

    leftGoal.trajectory.points.push_back({});
    rightGoal.trajectory.points.push_back({});
    tfs += STD_DURATION;
    n = leftGoal.trajectory.points.size() - 1;
    leftGoal.trajectory.points[n].positions.push_back(1.57);
    leftGoal.trajectory.points[n].positions.push_back(0.5);
    leftGoal.trajectory.points[n].positions.push_back(3.14);
    leftGoal.trajectory.points[n].positions.push_back(-1.57);
    leftGoal.trajectory.points[n].positions.push_back(0.0);
    leftGoal.trajectory.points[n].positions.push_back(0.0);
    leftGoal.trajectory.points[n].positions.push_back(0.0);
    rightGoal.trajectory.points[n].positions.push_back(-1.57);
    rightGoal.trajectory.points[n].positions.push_back(-0.5);
    rightGoal.trajectory.points[n].positions.push_back(0.0);
    rightGoal.trajectory.points[n].positions.push_back(-1.57);
    rightGoal.trajectory.points[n].positions.push_back(0.0);
    rightGoal.trajectory.points[n].positions.push_back(0.0);
    rightGoal.trajectory.points[n].positions.push_back(0.0);
    leftGoal.trajectory.points[n].time_from_start = ros::Duration(tfs);
    rightGoal.trajectory.points[n].time_from_start = ros::Duration(tfs);
    leftGoal.trajectory.points.push_back(leftGoal.trajectory.points[n]);
    rightGoal.trajectory.points.push_back(rightGoal.trajectory.points[n]);
    tfs += STD_DURATION;
    n = leftGoal.trajectory.points.size() - 1;
    leftGoal.trajectory.points[n].time_from_start = ros::Duration(tfs);
    rightGoal.trajectory.points[n].time_from_start = ros::Duration(tfs);

    leftGoal.trajectory.points.push_back({});
    rightGoal.trajectory.points.push_back({});
    tfs += STD_DURATION;
    n = leftGoal.trajectory.points.size() - 1;
    leftGoal.trajectory.points[n].positions.push_back(1.57);
    leftGoal.trajectory.points[n].positions.push_back(0.0);
    leftGoal.trajectory.points[n].positions.push_back(0.0);
    leftGoal.trajectory.points[n].positions.push_back(0.0);
    leftGoal.trajectory.points[n].positions.push_back(0.0);
    leftGoal.trajectory.points[n].positions.push_back(0.0);
    leftGoal.trajectory.points[n].positions.push_back(0.0);
    rightGoal.trajectory.points[n].positions.push_back(-1.57);
    rightGoal.trajectory.points[n].positions.push_back(0.0);
    rightGoal.trajectory.points[n].positions.push_back(0.0);
    rightGoal.trajectory.points[n].positions.push_back(-1.57);
    rightGoal.trajectory.points[n].positions.push_back(0.0);
    rightGoal.trajectory.points[n].positions.push_back(0.0);
    rightGoal.trajectory.points[n].positions.push_back(0.0);
    leftGoal.trajectory.points[n].time_from_start = ros::Duration(tfs);
    rightGoal.trajectory.points[n].time_from_start = ros::Duration(tfs); 
    leftGoal.trajectory.points.push_back(leftGoal.trajectory.points[n]);
    rightGoal.trajectory.points.push_back(rightGoal.trajectory.points[n]);
    tfs += STD_DURATION;
    n = leftGoal.trajectory.points.size() - 1;
    leftGoal.trajectory.points[n].time_from_start = ros::Duration(tfs);
    rightGoal.trajectory.points[n].time_from_start = ros::Duration(tfs);

    
    leftGoal.trajectory.points.push_back({});
    rightGoal.trajectory.points.push_back({});
    tfs += STD_DURATION;
    n = leftGoal.trajectory.points.size() - 1;
    leftGoal.trajectory.points[n].positions.push_back(1.57);
    leftGoal.trajectory.points[n].positions.push_back(0.0);
    leftGoal.trajectory.points[n].positions.push_back(3.1415);
    leftGoal.trajectory.points[n].positions.push_back(-1.57);
    leftGoal.trajectory.points[n].positions.push_back(0.0);
    leftGoal.trajectory.points[n].positions.push_back(0.0);
    leftGoal.trajectory.points[n].positions.push_back(0.0);
    rightGoal.trajectory.points[n].positions.push_back(-1.57);
    rightGoal.trajectory.points[n].positions.push_back(0.0);
    rightGoal.trajectory.points[n].positions.push_back(0.0);
    rightGoal.trajectory.points[n].positions.push_back(-1.57);
    rightGoal.trajectory.points[n].positions.push_back(0.0);
    rightGoal.trajectory.points[n].positions.push_back(0.0);
    rightGoal.trajectory.points[n].positions.push_back(0.0);
    leftGoal.trajectory.points[n].time_from_start = ros::Duration(tfs);
    rightGoal.trajectory.points[n].time_from_start = ros::Duration(tfs); 
    leftGoal.trajectory.points.push_back(leftGoal.trajectory.points[n]);
    rightGoal.trajectory.points.push_back(rightGoal.trajectory.points[n]);
    tfs += STD_DURATION;
    n = leftGoal.trajectory.points.size() - 1;
    leftGoal.trajectory.points[n].time_from_start = ros::Duration(tfs);
    rightGoal.trajectory.points[n].time_from_start = ros::Duration(tfs);
        
    leftGoal.trajectory.points.push_back({});
    rightGoal.trajectory.points.push_back({});
    tfs += STD_DURATION;
    n = leftGoal.trajectory.points.size() - 1;
    leftGoal.trajectory.points[n].positions.push_back(1.57);
    leftGoal.trajectory.points[n].positions.push_back(0.0);
    leftGoal.trajectory.points[n].positions.push_back(0.0);
    leftGoal.trajectory.points[n].positions.push_back(-1.57);
    leftGoal.trajectory.points[n].positions.push_back(0.0);
    leftGoal.trajectory.points[n].positions.push_back(0.0);
    leftGoal.trajectory.points[n].positions.push_back(0.0);
    rightGoal.trajectory.points[n].positions.push_back(-1.57);
    rightGoal.trajectory.points[n].positions.push_back(0.0);
    rightGoal.trajectory.points[n].positions.push_back(3.1416);
    rightGoal.trajectory.points[n].positions.push_back(-1.57);
    rightGoal.trajectory.points[n].positions.push_back(0.0);
    rightGoal.trajectory.points[n].positions.push_back(0.0);
    rightGoal.trajectory.points[n].positions.push_back(0.0);
    leftGoal.trajectory.points[n].time_from_start = ros::Duration(tfs);
    rightGoal.trajectory.points[n].time_from_start = ros::Duration(tfs); 
    leftGoal.trajectory.points.push_back(leftGoal.trajectory.points[n]);
    rightGoal.trajectory.points.push_back(rightGoal.trajectory.points[n]);
    tfs += STD_DURATION;
    n = leftGoal.trajectory.points.size() - 1;
    leftGoal.trajectory.points[n].time_from_start = ros::Duration(tfs);
    rightGoal.trajectory.points[n].time_from_start = ros::Duration(tfs);

    leftGoal.trajectory.points.push_back({});
    rightGoal.trajectory.points.push_back({});
    tfs += STD_DURATION;
    n = leftGoal.trajectory.points.size() - 1;
    leftGoal.trajectory.points[n].positions.push_back(1.57);
    leftGoal.trajectory.points[n].positions.push_back(0.0);
    leftGoal.trajectory.points[n].positions.push_back(3.1415);
    leftGoal.trajectory.points[n].positions.push_back(-1.57);
    leftGoal.trajectory.points[n].positions.push_back(0.0);
    leftGoal.trajectory.points[n].positions.push_back(0.0);
    leftGoal.trajectory.points[n].positions.push_back(0.0);
    rightGoal.trajectory.points[n].positions.push_back(-1.57);
    rightGoal.trajectory.points[n].positions.push_back(0.0);
    rightGoal.trajectory.points[n].positions.push_back(0.0);
    rightGoal.trajectory.points[n].positions.push_back(-1.57);
    rightGoal.trajectory.points[n].positions.push_back(0.0);
    rightGoal.trajectory.points[n].positions.push_back(0.0);
    rightGoal.trajectory.points[n].positions.push_back(0.0);
    leftGoal.trajectory.points[n].time_from_start = ros::Duration(tfs);
    rightGoal.trajectory.points[n].time_from_start = ros::Duration(tfs); 
    leftGoal.trajectory.points.push_back(leftGoal.trajectory.points[n]);
    rightGoal.trajectory.points.push_back(rightGoal.trajectory.points[n]);
    tfs += STD_DURATION;
    n = leftGoal.trajectory.points.size() - 1;
    leftGoal.trajectory.points[n].time_from_start = ros::Duration(tfs);
    rightGoal.trajectory.points[n].time_from_start = ros::Duration(tfs);

    leftGoal.trajectory.points.push_back({});
    rightGoal.trajectory.points.push_back({});
    tfs += STD_DURATION;
    n = leftGoal.trajectory.points.size() - 1;
    leftGoal.trajectory.points[n].positions.push_back(1.57);
    leftGoal.trajectory.points[n].positions.push_back(-0.5);
    leftGoal.trajectory.points[n].positions.push_back(0.0);
    leftGoal.trajectory.points[n].positions.push_back(0.0);
    leftGoal.trajectory.points[n].positions.push_back(0.0);
    leftGoal.trajectory.points[n].positions.push_back(0.0);
    leftGoal.trajectory.points[n].positions.push_back(0.0);
    rightGoal.trajectory.points[n].positions.push_back(-1.57);
    rightGoal.trajectory.points[n].positions.push_back(0.5);
    rightGoal.trajectory.points[n].positions.push_back(0.0);
    rightGoal.trajectory.points[n].positions.push_back(0.0);
    rightGoal.trajectory.points[n].positions.push_back(0.0);
    rightGoal.trajectory.points[n].positions.push_back(0.0);
    rightGoal.trajectory.points[n].positions.push_back(0.0);
    leftGoal.trajectory.points[n].time_from_start = ros::Duration(tfs);
    rightGoal.trajectory.points[n].time_from_start = ros::Duration(tfs); 
    leftGoal.trajectory.points.push_back(leftGoal.trajectory.points[n]);
    rightGoal.trajectory.points.push_back(rightGoal.trajectory.points[n]);
    tfs += STD_DURATION;
    n = leftGoal.trajectory.points.size() - 1;
    leftGoal.trajectory.points[n].time_from_start = ros::Duration(tfs);
    rightGoal.trajectory.points[n].time_from_start = ros::Duration(tfs);

    leftGoal.trajectory.points.push_back({});
    rightGoal.trajectory.points.push_back({});
    tfs += STD_DURATION;
    n = leftGoal.trajectory.points.size() - 1;
    leftGoal.trajectory.points[n].positions.push_back(1.57);
    leftGoal.trajectory.points[n].positions.push_back(0.0);
    leftGoal.trajectory.points[n].positions.push_back(0.0);
    leftGoal.trajectory.points[n].positions.push_back(-1.57);
    leftGoal.trajectory.points[n].positions.push_back(0.0);
    leftGoal.trajectory.points[n].positions.push_back(0.0);
    leftGoal.trajectory.points[n].positions.push_back(0.0);
    rightGoal.trajectory.points[n].positions.push_back(-1.57);
    rightGoal.trajectory.points[n].positions.push_back(0.0);
    rightGoal.trajectory.points[n].positions.push_back(0.0);
    rightGoal.trajectory.points[n].positions.push_back(-1.57);
    rightGoal.trajectory.points[n].positions.push_back(0.0);
    rightGoal.trajectory.points[n].positions.push_back(0.0);
    rightGoal.trajectory.points[n].positions.push_back(0.0);
    leftGoal.trajectory.points[n].time_from_start = ros::Duration(tfs);
    rightGoal.trajectory.points[n].time_from_start = ros::Duration(tfs); 
    leftGoal.trajectory.points.push_back(leftGoal.trajectory.points[n]);
    rightGoal.trajectory.points.push_back(rightGoal.trajectory.points[n]);
    tfs += STD_DURATION;
    n = leftGoal.trajectory.points.size() - 1;
    leftGoal.trajectory.points[n].time_from_start = ros::Duration(tfs);
    rightGoal.trajectory.points[n].time_from_start = ros::Duration(tfs);


    leftGoal.trajectory.points.push_back({});
    rightGoal.trajectory.points.push_back({});
    tfs += STD_DURATION;
    n = leftGoal.trajectory.points.size() - 1;
    leftGoal.trajectory.points[n].positions.push_back(1.57);
    leftGoal.trajectory.points[n].positions.push_back(0.0);
    leftGoal.trajectory.points[n].positions.push_back(3.1415);
    leftGoal.trajectory.points[n].positions.push_back(-1.57);
    leftGoal.trajectory.points[n].positions.push_back(0.0);
    leftGoal.trajectory.points[n].positions.push_back(0.0);
    leftGoal.trajectory.points[n].positions.push_back(0.0);
    rightGoal.trajectory.points[n].positions.push_back(-1.57);
    rightGoal.trajectory.points[n].positions.push_back(0.0);
    rightGoal.trajectory.points[n].positions.push_back(3.1416);
    rightGoal.trajectory.points[n].positions.push_back(-1.57);
    rightGoal.trajectory.points[n].positions.push_back(0.0);
    rightGoal.trajectory.points[n].positions.push_back(0.0);
    rightGoal.trajectory.points[n].positions.push_back(0.0);
    leftGoal.trajectory.points[n].time_from_start = ros::Duration(tfs);
    rightGoal.trajectory.points[n].time_from_start = ros::Duration(tfs); 
    leftGoal.trajectory.points.push_back(leftGoal.trajectory.points[n]);
    rightGoal.trajectory.points.push_back(rightGoal.trajectory.points[n]);
    tfs += STD_DURATION;
    n = leftGoal.trajectory.points.size() - 1;
    leftGoal.trajectory.points[n].time_from_start = ros::Duration(tfs);
    rightGoal.trajectory.points[n].time_from_start = ros::Duration(tfs);

    leftGoal.trajectory.points.push_back({});
    rightGoal.trajectory.points.push_back({});
    tfs += STD_DURATION;
    n = leftGoal.trajectory.points.size() - 1;
    leftGoal.trajectory.points[n].positions.push_back(0.0);
    leftGoal.trajectory.points[n].positions.push_back(0.0);
    leftGoal.trajectory.points[n].positions.push_back(0.0);
    leftGoal.trajectory.points[n].positions.push_back(-1.57);
    leftGoal.trajectory.points[n].positions.push_back(0.0);
    leftGoal.trajectory.points[n].positions.push_back(0.0);
    leftGoal.trajectory.points[n].positions.push_back(0.0);
    rightGoal.trajectory.points[n].positions.push_back(0.0);
    rightGoal.trajectory.points[n].positions.push_back(1.3);
    rightGoal.trajectory.points[n].positions.push_back(1.57);
    rightGoal.trajectory.points[n].positions.push_back(-1.57);
    rightGoal.trajectory.points[n].positions.push_back(0.0);
    rightGoal.trajectory.points[n].positions.push_back(0.0);
    rightGoal.trajectory.points[n].positions.push_back(0.0);
    leftGoal.trajectory.points[n].time_from_start = ros::Duration(tfs);
    rightGoal.trajectory.points[n].time_from_start = ros::Duration(tfs); 
    leftGoal.trajectory.points.push_back(leftGoal.trajectory.points[n]);
    rightGoal.trajectory.points.push_back(rightGoal.trajectory.points[n]);
    tfs += STD_DURATION;
    n = leftGoal.trajectory.points.size() - 1;
    leftGoal.trajectory.points[n].time_from_start = ros::Duration(tfs);
    rightGoal.trajectory.points[n].time_from_start = ros::Duration(tfs);

    leftGoal.trajectory.points.push_back({});
    rightGoal.trajectory.points.push_back({});
    tfs += STD_DURATION;
    n = leftGoal.trajectory.points.size() - 1;
    leftGoal.trajectory.points[n].positions.push_back(1.57);
    leftGoal.trajectory.points[n].positions.push_back(0.0);
    leftGoal.trajectory.points[n].positions.push_back(0.0);
    leftGoal.trajectory.points[n].positions.push_back(-1.57);
    leftGoal.trajectory.points[n].positions.push_back(0.0);
    leftGoal.trajectory.points[n].positions.push_back(0.0);
    leftGoal.trajectory.points[n].positions.push_back(0.0);
    rightGoal.trajectory.points[n].positions.push_back(-1.57);
    rightGoal.trajectory.points[n].positions.push_back(0.0);
    rightGoal.trajectory.points[n].positions.push_back(3.1416);
    rightGoal.trajectory.points[n].positions.push_back(-1.57);
    rightGoal.trajectory.points[n].positions.push_back(0.0);
    rightGoal.trajectory.points[n].positions.push_back(0.0);
    rightGoal.trajectory.points[n].positions.push_back(0.0);
    leftGoal.trajectory.points[n].time_from_start = ros::Duration(tfs);
    rightGoal.trajectory.points[n].time_from_start = ros::Duration(tfs); 
    leftGoal.trajectory.points.push_back(leftGoal.trajectory.points[n]);
    rightGoal.trajectory.points.push_back(rightGoal.trajectory.points[n]);
    tfs += STD_DURATION;
    n = leftGoal.trajectory.points.size() - 1;
    leftGoal.trajectory.points[n].time_from_start = ros::Duration(tfs);
    rightGoal.trajectory.points[n].time_from_start = ros::Duration(tfs);

    leftGoal.trajectory.points.push_back({});
    rightGoal.trajectory.points.push_back({});
    tfs += STD_DURATION;
    n = leftGoal.trajectory.points.size() - 1;
    leftGoal.trajectory.points[n].positions.push_back(0.0);
    leftGoal.trajectory.points[n].positions.push_back(-0.5);
    leftGoal.trajectory.points[n].positions.push_back(-1.57);
    leftGoal.trajectory.points[n].positions.push_back(-1.57);
    leftGoal.trajectory.points[n].positions.push_back(0.0);
    leftGoal.trajectory.points[n].positions.push_back(0.0);
    leftGoal.trajectory.points[n].positions.push_back(0.0);
    rightGoal.trajectory.points[n].positions.push_back(0.0);
    rightGoal.trajectory.points[n].positions.push_back(1.3);
    rightGoal.trajectory.points[n].positions.push_back(0.0);
    rightGoal.trajectory.points[n].positions.push_back(-0.5);
    rightGoal.trajectory.points[n].positions.push_back(0.0);
    rightGoal.trajectory.points[n].positions.push_back(0.0);
    rightGoal.trajectory.points[n].positions.push_back(0.0);
    leftGoal.trajectory.points[n].time_from_start = ros::Duration(tfs);
    rightGoal.trajectory.points[n].time_from_start = ros::Duration(tfs); 
    leftGoal.trajectory.points.push_back(leftGoal.trajectory.points[n]);
    rightGoal.trajectory.points.push_back(rightGoal.trajectory.points[n]);
    tfs += STD_DURATION;
    n = leftGoal.trajectory.points.size() - 1;
    leftGoal.trajectory.points[n].time_from_start = ros::Duration(tfs);
    rightGoal.trajectory.points[n].time_from_start = ros::Duration(tfs);


    leftGoal.trajectory.points.push_back({});
    rightGoal.trajectory.points.push_back({});
    tfs += STD_DURATION;
    n = leftGoal.trajectory.points.size() - 1;
    leftGoal.trajectory.points[n].positions.push_back(1.57);
    leftGoal.trajectory.points[n].positions.push_back(0.5);
    leftGoal.trajectory.points[n].positions.push_back(3.14);
    leftGoal.trajectory.points[n].positions.push_back(-1.57);
    leftGoal.trajectory.points[n].positions.push_back(0.0);
    leftGoal.trajectory.points[n].positions.push_back(0.0);
    leftGoal.trajectory.points[n].positions.push_back(0.0);
    rightGoal.trajectory.points[n].positions.push_back(-1.57);
    rightGoal.trajectory.points[n].positions.push_back(-0.5);
    rightGoal.trajectory.points[n].positions.push_back(0.0);
    rightGoal.trajectory.points[n].positions.push_back(-1.57);
    rightGoal.trajectory.points[n].positions.push_back(0.0);
    rightGoal.trajectory.points[n].positions.push_back(0.0);
    rightGoal.trajectory.points[n].positions.push_back(0.0);
    leftGoal.trajectory.points[n].time_from_start = ros::Duration(tfs);
    rightGoal.trajectory.points[n].time_from_start = ros::Duration(tfs);
    leftGoal.trajectory.points.push_back(leftGoal.trajectory.points[n]);
    rightGoal.trajectory.points.push_back(rightGoal.trajectory.points[n]);
    tfs += STD_DURATION;
    n = leftGoal.trajectory.points.size() - 1;
    leftGoal.trajectory.points[n].time_from_start = ros::Duration(tfs);
    rightGoal.trajectory.points[n].time_from_start = ros::Duration(tfs);

    leftGoal.trajectory.points.push_back({});
    rightGoal.trajectory.points.push_back({});
    tfs += STD_DURATION;
    n = leftGoal.trajectory.points.size() - 1;
    leftGoal.trajectory.points[n].positions.push_back(1.57);
    leftGoal.trajectory.points[n].positions.push_back(0.0);
    leftGoal.trajectory.points[n].positions.push_back(3.1415);
    leftGoal.trajectory.points[n].positions.push_back(-1.57);
    leftGoal.trajectory.points[n].positions.push_back(0.0);
    leftGoal.trajectory.points[n].positions.push_back(0.0);
    leftGoal.trajectory.points[n].positions.push_back(0.0);
    rightGoal.trajectory.points[n].positions.push_back(-1.57);
    rightGoal.trajectory.points[n].positions.push_back(0.0);
    rightGoal.trajectory.points[n].positions.push_back(0.0);
    rightGoal.trajectory.points[n].positions.push_back(-1.57);
    rightGoal.trajectory.points[n].positions.push_back(0.0);
    rightGoal.trajectory.points[n].positions.push_back(0.0);
    rightGoal.trajectory.points[n].positions.push_back(0.0);
    leftGoal.trajectory.points[n].time_from_start = ros::Duration(tfs);
    rightGoal.trajectory.points[n].time_from_start = ros::Duration(tfs); 
    leftGoal.trajectory.points.push_back(leftGoal.trajectory.points[n]);
    rightGoal.trajectory.points.push_back(rightGoal.trajectory.points[n]);
    tfs += STD_DURATION;
    n = leftGoal.trajectory.points.size() - 1;
    leftGoal.trajectory.points[n].time_from_start = ros::Duration(tfs);
    rightGoal.trajectory.points[n].time_from_start = ros::Duration(tfs);

    leftGoal.trajectory.points.push_back({});
    rightGoal.trajectory.points.push_back({});
    tfs += STD_DURATION;
    n = leftGoal.trajectory.points.size() - 1;
    leftGoal.trajectory.points[n].positions.push_back(0.0);
    leftGoal.trajectory.points[n].positions.push_back(1.3);
    leftGoal.trajectory.points[n].positions.push_back(-1.57);
    leftGoal.trajectory.points[n].positions.push_back(-1.57);
    leftGoal.trajectory.points[n].positions.push_back(0.0);
    leftGoal.trajectory.points[n].positions.push_back(0.0);
    leftGoal.trajectory.points[n].positions.push_back(0.0);
    rightGoal.trajectory.points[n].positions.push_back(0.0);
    rightGoal.trajectory.points[n].positions.push_back(-0.5);
    rightGoal.trajectory.points[n].positions.push_back(1.57);
    rightGoal.trajectory.points[n].positions.push_back(-1.57);
    rightGoal.trajectory.points[n].positions.push_back(0.0);
    rightGoal.trajectory.points[n].positions.push_back(0.0);
    rightGoal.trajectory.points[n].positions.push_back(0.0);
    leftGoal.trajectory.points[n].time_from_start = ros::Duration(tfs);
    rightGoal.trajectory.points[n].time_from_start = ros::Duration(tfs); 
    leftGoal.trajectory.points.push_back(leftGoal.trajectory.points[n]);
    rightGoal.trajectory.points.push_back(rightGoal.trajectory.points[n]);
    tfs += STD_DURATION;
    n = leftGoal.trajectory.points.size() - 1;
    leftGoal.trajectory.points[n].time_from_start = ros::Duration(tfs);
    rightGoal.trajectory.points[n].time_from_start = ros::Duration(tfs);

    leftGoal.trajectory.points.push_back({});
    rightGoal.trajectory.points.push_back({});
    tfs += STD_DURATION;
    n = leftGoal.trajectory.points.size() - 1;
    leftGoal.trajectory.points[n].positions.push_back(1.57);
    leftGoal.trajectory.points[n].positions.push_back(-0.5);
    leftGoal.trajectory.points[n].positions.push_back(0.0);
    leftGoal.trajectory.points[n].positions.push_back(0.0);
    leftGoal.trajectory.points[n].positions.push_back(0.0);
    leftGoal.trajectory.points[n].positions.push_back(0.0);
    leftGoal.trajectory.points[n].positions.push_back(0.0);
    rightGoal.trajectory.points[n].positions.push_back(-1.57);
    rightGoal.trajectory.points[n].positions.push_back(0.5);
    rightGoal.trajectory.points[n].positions.push_back(0.0);
    rightGoal.trajectory.points[n].positions.push_back(-1.57);
    rightGoal.trajectory.points[n].positions.push_back(0.0);
    rightGoal.trajectory.points[n].positions.push_back(0.0);
    rightGoal.trajectory.points[n].positions.push_back(0.0);
    leftGoal.trajectory.points[n].time_from_start = ros::Duration(tfs);
    rightGoal.trajectory.points[n].time_from_start = ros::Duration(tfs); 
    leftGoal.trajectory.points.push_back(leftGoal.trajectory.points[n]);
    rightGoal.trajectory.points.push_back(rightGoal.trajectory.points[n]);
    tfs += STD_DURATION;
    n = leftGoal.trajectory.points.size() - 1;
    leftGoal.trajectory.points[n].time_from_start = ros::Duration(tfs);
    rightGoal.trajectory.points[n].time_from_start = ros::Duration(tfs);



    leftGoal.trajectory.points.push_back({});
    rightGoal.trajectory.points.push_back({});
    tfs += STD_DURATION;
    n = leftGoal.trajectory.points.size() - 1;
    leftGoal.trajectory.points[n].positions.push_back(0.0);
    leftGoal.trajectory.points[n].positions.push_back(1.109);
    leftGoal.trajectory.points[n].positions.push_back(0.0);
    leftGoal.trajectory.points[n].positions.push_back(-2.234);
    leftGoal.trajectory.points[n].positions.push_back(0.0);
    leftGoal.trajectory.points[n].positions.push_back(-1.658);
    leftGoal.trajectory.points[n].positions.push_back(0.0);
    rightGoal.trajectory.points[n].positions.push_back(0.0);
    rightGoal.trajectory.points[n].positions.push_back(1.109);
    rightGoal.trajectory.points[n].positions.push_back(0.0);
    rightGoal.trajectory.points[n].positions.push_back(-2.234);
    rightGoal.trajectory.points[n].positions.push_back(0.0);
    rightGoal.trajectory.points[n].positions.push_back(-1.658);
    rightGoal.trajectory.points[n].positions.push_back(0.0);
    leftGoal.trajectory.points[n].time_from_start = ros::Duration(tfs);
    rightGoal.trajectory.points[n].time_from_start = ros::Duration(tfs); 
    leftGoal.trajectory.points.push_back(leftGoal.trajectory.points[n]);
    rightGoal.trajectory.points.push_back(rightGoal.trajectory.points[n]);
    tfs += STD_DURATION;
    n = leftGoal.trajectory.points.size() - 1;
    leftGoal.trajectory.points[n].time_from_start = ros::Duration(tfs);
    rightGoal.trajectory.points[n].time_from_start = ros::Duration(tfs);

    for (size_t i = 0; i < leftGoal.trajectory.points.size(); i++){
        for (size_t j = 0; j < leftGoal.trajectory.points[i].positions.size(); j++) {
            leftGoal.trajectory.points[i].velocities.push_back(0.0);
            rightGoal.trajectory.points[i].velocities.push_back(0.0);
        }
    }
    leftArmAct_.sendGoal(leftGoal);
    rightArmAct_.sendGoal(rightGoal);
    leftArmAct_.waitForResult();
    rightArmAct_.waitForResult();
}

void Theta::fillArmCommand(const std::string& side, pr2_controllers_msgs::JointTrajectoryGoal& goal){
    goal.trajectory.header.frame_id = "base_footprint";
    goal.trajectory.header.stamp = ros::Time::now();
    if (side == "left"){
        goal.trajectory.joint_names.emplace_back("l_shoulder_pan_joint");
        goal.trajectory.joint_names.emplace_back("l_shoulder_lift_joint");
        goal.trajectory.joint_names.emplace_back("l_upper_arm_roll_joint");
        goal.trajectory.joint_names.emplace_back("l_elbow_flex_joint");
        goal.trajectory.joint_names.emplace_back("l_forearm_roll_joint");
        goal.trajectory.joint_names.emplace_back("l_wrist_flex_joint");
        goal.trajectory.joint_names.emplace_back("l_wrist_roll_joint");
    }else{
        goal.trajectory.joint_names.emplace_back("r_shoulder_pan_joint");
        goal.trajectory.joint_names.emplace_back("r_shoulder_lift_joint");
        goal.trajectory.joint_names.emplace_back("r_upper_arm_roll_joint");
        goal.trajectory.joint_names.emplace_back("r_elbow_flex_joint");
        goal.trajectory.joint_names.emplace_back("r_forearm_roll_joint");
        goal.trajectory.joint_names.emplace_back("r_wrist_flex_joint");
        goal.trajectory.joint_names.emplace_back("r_wrist_roll_joint");
    }
}

void Theta::look(){
    pr2_controllers_msgs::PointHeadGoal g;
    geometry_msgs::PointStamped point;
    point.header.frame_id = "base_footprint";
    point.header.stamp = ros::Time::now();
    point.point.x = 1.5;
    point.point.y = 0.0;
    point.point.z = 1.6;
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
