#include "omega/zeta.hpp"
#include <sensor_msgs/image_encodings.h>
#include <math.h> 
#include <cv_bridge/cv_bridge.h>
#include <stdexcept>
#include <std_msgs/String.h>

Zeta::Zeta(ros::NodeHandlePtr nh): Enigma(nh), headact_("/head_traj_controller/point_head_action"), it_(*nh_), capture_(false), lastAnswer_(' '){
    imgSub_ = it_.subscribe("/wide_stereo/left/image_rect", 1, &Zeta::imageCb, this);
    dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    sayPub_ = nh->advertise<std_msgs::String>("/say", 5);
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
        std::make_tuple("Combien de fonctionnalités de plus a Oro comparé à ontologenius ?", 
            std::vector<std::string>({"5 fois plus", "10 fois plus", "20 fois plus", "100 fois plus"}), 'B'), 
        std::make_tuple("Combien y avait-il de perles RIS répertoriées au 8 mai 2021 ?", 
            std::vector<std::string>({"42", "51", "69", "101"}), 'C'), 
        std::make_tuple("Sur combien de version de moveXD Jules a-t-il travaillé ?", 
            std::vector<std::string>({"2", "3", "4", "5"}), 'B'),
        std::make_tuple("Qui a dit “Je préfère être capitaliste que roux” ?", 
            std::vector<std::string>({"David", "Jules", "Guillaume", "Amandine"}), 'A'),
        std::make_tuple("Quel membre de RIS a donné son nom à une technique de tarot ?", 
            std::vector<std::string>({"Amélie", "Amandine", "Guillaume", "Rafa"}), 'D'),    
        std::make_tuple("Durant la calibration (après l’allumage) du PR2 les grippers se calibrent avant la tête.", 
            std::vector<std::string>({"Vrai", "Faux"}), 'A'),  
        std::make_tuple("Quelle peluche n’a jamais été présente dans l'open space le plus à l'Est ?", 
            std::vector<std::string>({"Un neurone", "Une carotte", "Une baleine", "Un loup"}), 'D'),    
        std::make_tuple("Durant le séminaire RIS 2018, la table d’Amandine est arrivée en deuxième position de la table ayant mangé le plus de raclette.", 
            std::vector<std::string>({"Vrai", "Faux"}), 'B'), 
        std::make_tuple("Combien de versions expliquant la couleur verte de la bière bue à Tampere le serveur a-t-il donné ?", 
            std::vector<std::string>({"Une", "Deux", "Trois", "Quatre"}), 'B'), 
        std::make_tuple("Que faut-il faire d’après Rachid ?", 
            std::vector<std::string>({"Préciser et rajouter", "Argumenter et agrémenter", "Affiner et enrichir",
                "Travailler incrémentalement, être modeste et ne pas viser trop loin trop vite"}), 'C'), 
            };
    for (const auto &p: qs){
        capture_ = false;
        lastAnswer_ = ' ';
        std::string question = std::get<0>(p);
        std::cout << question << std::endl;
        say(question);
        for (size_t i=0; i < std::get<1>(p).size(); i++){
            std::cout << L[i] << " - " << std::get<1>(p)[i] <<std::endl;
            say(L[i] + ", " + std::get<1>(p)[i]);
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
            std::cout << "Pas de réponse détectée, fin de la procédure." << std::endl;
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
    say("Félicitations");

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
    ros::Duration(0.55 * (std::count(speech.cbegin(), speech.cend(), ' ') + 1)).sleep();
}