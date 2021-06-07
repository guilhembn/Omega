#include <ros/ros.h>
#include "omega/enigma.hpp"
#include "omega/delta.hpp"

class OmegaCore{
    public:
    OmegaCore(ros::NodeHandlePtr& nh): nh_(nh), e_(nullptr){
    }

    ~OmegaCore(){
        delete e_;
    }

    void run(){
        std::string pass;
        std::cout << "Mot de passe : ";
        std::cin >> pass;
        std::size_t h1 = std::hash<std::string>{}(pass);
        if (h1 == 7404464429118221970){
            e_ = new Delta(nh_);
        }
        if (e_ != nullptr){
            std::cout << "Mot de passe accepté." << std::endl;
            std::cout << "Lancement de la procédure '" << e_->name() << "'..." << std::endl;
            e_->run();
        }else{
            std::cout << "Mot de passe incorrect." << std::endl;
        }
    }

    private:
    ros::NodeHandlePtr nh_;
    Enigma* e_;

};

int main(int argc, char** argv){
    ros::init(argc, argv, "Omega");
    ros::NodeHandlePtr nh = boost::make_shared<ros::NodeHandle>("~");
    OmegaCore oc(nh);
    oc.run();
    return 0;
}