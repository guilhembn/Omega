#include "omega/zeta.hpp"
#include <sensor_msgs/image_encodings.h>
#include <math.h> 


Zeta::Zeta(ros::NodeHandlePtr nh): Enigma(nh), headact_("/head_traj_controller/point_head_action"), it_(nh_), capture_(false), lastAnswer_(''){
    imgSub_ = it_.subscribe("/to/do/image_compressed", 1, &Zeta::imageCb, this);
    dictionnary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
}

std::string Zeta::name(){
    return "Zeta";
}

void Zeta::run(){
    char L[4] = {'A', 'B', 'C', 'D'};
    std::vector<std::tuple<std::string, std::vector<std::string>, char>> qs = {
        {"Répondez A", {"Oui", "Non", "Toujours pas", "Nope..."} 'A'}, 
        {"Répondez B", {"Non", "Oui", "Nope", "Niet"}, 'B'}, 
        {"Répondez C", {"Noon", "No", "Oui", "Neg"}, 'C'},
        {"Répondez D", {"Non", "Non", "Non", "Oui"}, 'D'}};
    for (const auto &p: qs){
        capture_ = false;
        lastAnswer_ = '';
        std::cout << std::get<0>(p) << std::endl;
        for (size_t i=0; i < std::get<1>(p).size(); i++){
            std::cout << L[i] << " - " << std::get<1>(p)[i] <<std::endl;
        }
        for (unsigned int i=5; i>0; i--){
            std::cout << "\rValidation de la réponse dans " << i << " secondes" << std::flush;
            ros::Duration(1.0).sleep();
        }
        std::cout << std::endl;
        capture_ = true;
        for (unsigned int i=0; i<10; i++){
            if (lastAnswer_ != ''){
                break;
            }
            ros::Duration(0.5).sleep();
        }
        capture_ = false;
        if (lastAnswer_ == ''){
            std::cout << "No answer detected" << std::endl;
            return;
        }
        assert(lastAnswer_ == 'A' || lastAnswer_ == 'B' || lastAnswer_ == 'C' || lastAnswer_ == 'D');
        std::cout << "Votre réponse : " << lastAnswer_ << std::endl;
        if (lastAnswer_ != std::get<2>(p)){
            std::cout << "Mauvaise réponse !" << std::endl;
            return;
        }else{
            std::cout << "Bien joué !";
        }
    }

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
    cv::aruco::detectMarkers(inputImage, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);
    int index = -1;
    for (size_t i=0; i < markerIds.size(); i++){
        int markerId = markerIds[i];
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
    return centerAngle(atan2f(orientVec.x, orientVec.y));  // Switch done on purpose.
}

static float Zeta::centerAngle(float angle){
    while (angle < -M_PI){
        angle += 2*M_PI;
    }
    while (angle >= M_PI){
        angle -= 2*M_PI;
    }
    return angle;
}