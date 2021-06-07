#include <ros/ros.h>
#include "omega/enigma.hpp"
#include "omega/delta.hpp"
#include "omega/zeta.hpp"

#include <openssl/md5.h>

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
        unsigned char h[MD5_DIGEST_LENGTH];
        MD5(pass.c_str(), pass.length(), h);
        if (memcmp(h, "64a4e8faed1a1aa0bf8bf0fc84938d25") == 0){
            e_ = new Delta(nh_);
        }else if(memcmp(h, "307ebb72d3e246ddae773de23b487ab6") == 0){
            e_ = new Zeta(nh_);
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