#include "omega/zeta.hpp"
#include <sensor_msgs/image_encodings.h>
#include <math.h> 
#include <cv_bridge/cv_bridge.h>
#include <stdexcept>


Zeta::Zeta(ros::NodeHandlePtr nh): Enigma(nh), headact_("/head_traj_controller/point_head_action"), it_(*nh_), capture_(false), lastAnswer_(' '){
    imgSub_ = it_.subscribe("/wide_stereo/left/image_rect", 1, &Zeta::imageCb, this);
    dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    headact_.waitForServer(ros::Duration(5.0));
    if (!headact_.isServerConnected()){
        std::cout << "Quelque chose ne va pas... Lancez vous bien cet executable sur le PR2 ? Le PR2 est-il prêt (i.e. robot.launch lancé) ?" << std::endl;
        throw std::runtime_error("Arrêt du système Omega.");
    }
}

std::string Zeta::name(){
    return "Zeta";
}

void Zeta::run(){
    char L[4] = {'A', 'B', 'C', 'D'};
    std::vector<std::tuple<std::string, std::vector<std::string>, char>> qs{
        std::make_tuple("Répondez A", std::vector<std::string>({"Oui", "Non", "Toujours pas", "Nope..."}), 'A'), 
        std::make_tuple("Répondez B", std::vector<std::string>({"Non", "Oui", "Nope", "Niet"}), 'B'), 
        std::make_tuple("Répondez C", std::vector<std::string>({"Noon", "No", "Oui", "Neg"}), 'C'),
        std::make_tuple("Répondez D", std::vector<std::string>({"Non", "Non", "Non", "Oui"}), 'D')};
            };
    for (const auto &p: qs){
        capture_ = false;
        lastAnswer_ = ' ';
        std::string question = std::get<0>(p);
        std::cout << question << std::endl;
        say(question);
        for (size_t i=0; i < std::get<1>(p).size(); i++){
            std::cout << L[i] << " - " << std::get<1>(p)[i] <<std::endl;
            say(std::get<1>(p)[i]);
            ros::Duration(0.75).sleep();
        }
        for (unsigned int i=5; i>0; i--){
            std::cout << "\rValidation de la réponse dans " << i << " secondes" << std::flush;
            ros::Duration(1.0).sleep();
        }
        std::cout << std::endl;
        capture_ = true;
        for (unsigned int i=0; i<100; i++){
            if (lastAnswer_ != ' '){
                break;
            }
            ros::Duration(0.01).sleep();
            ros::spinOnce();
        }
        capture_ = false;
        if (lastAnswer_ == ' '){
            std::cout << "No answer detected" << std::endl;
            return;
        }
        assert(lastAnswer_ == 'A' || lastAnswer_ == 'B' || lastAnswer_ == 'C' || lastAnswer_ == 'D');
        std::cout << "Votre réponse : " << lastAnswer_ << std::endl;
        if (lastAnswer_ != std::get<2>(p)){
            std::cout << "Mauvaise réponse ! Fin de la procédure." << std::endl;
            say("Mauvaise réponse ! Fin de la procédure.");
            return;
        }else{
            std::cout << "Réponse valide." << std::endl;
            say("Réponse valide.");
        }
    }
    say("Félicitations")

}

void Zeta::imageCb(const sensor_msgs::ImageConstPtr& msg){
    if (!capture_){
        return;
    }
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
    cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
    cv::aruco::detectMarkers(cv_ptr->image, dictionary_, markerCorners, markerIds, parameters, rejectedCandidates);
    int index = -1;
    for (size_t i=0; i < markerIds.size(); i++){
        int markerId = markerIds[i];
                std::cout << "Marker id" << markerId << std::endl;
        if (markerId == 13){
            index = i;
            break;
        }
    }
    if (index == -1){
        return;
    }
    float angle = getAngle(markerCorners[index]);
    if (angle <= 3*M_PI/4 && angle > M_PI/4){
        std::cout << "B" << std::endl;
        lastAnswer_ = 'B';
    }else if (angle <= M_PI/4 && angle > -M_PI/4){
        std::cout << "A" << std::endl;
        lastAnswer_ = 'A';
    }else if (angle <= -M_PI/4 && angle > -3*M_PI/4){
        std::cout << "D" << std::endl;
        lastAnswer_ = 'D';
    }else{
        std::cout << "C" << std::endl;
        lastAnswer_ = 'C';
    }
}



float Zeta::getAngle(const std::vector<cv::Point2f>& markerCorners) const{
    cv::Point2f topBary = (markerCorners[0] + markerCorners[1]) / 2.0f;
    cv::Point2f bottomBary = (markerCorners[2] + markerCorners[3]) / 2.0f;
    cv::Point2f orientVec = topBary - bottomBary;
    return centerAngle(atan2f(orientVec.y, orientVec.x) + 1.57079632679);  // Switch done on purpose.
}

float Zeta::centerAngle(float angle){
    while (angle < -M_PI){
        angle += 2*M_PI;
    }
    while (angle >= M_PI){
        angle -= 2*M_PI;
    }
    return angle;
}

void Zeta::say(const std::string& speech){
    std_msgs::String msg;
    msg.data = speech;
    sayPub_.publish(msg);
    ros::Duration(0.45 * (std::count(speech.cbegin(), speech.cend(), ' ') + 1)).sleep();
}