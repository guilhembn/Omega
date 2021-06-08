#include <ros/ros.h>
#include "omega/enigma.hpp"
#include "omega/delta.hpp"
#include "omega/zeta.hpp"

#include <openssl/sha.h>

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
        unsigned char h[SHA_DIGEST_LENGTH];
        unsigned char d[SHA_DIGEST_LENGTH] = {0x64, 0xfa, 0xf5, 0xd0, 0xb1, 0xdc, 0x31, 0x1f, 0xd0, 0xf9, 0x4a, 0xf6, 0x4f, 0x6c, 0x29, 0x6a, 0x03, 0x04, 0x55, 0x71};
        unsigned char s[SHA_DIGEST_LENGTH] = {0x75, 0x88, 0x7b, 0xc1, 0x37, 0xbd, 0x86, 0xe8, 0x2f, 0xfa, 0x6f, 0xad, 0x40, 0xb2, 0x99, 0x1c, 0xf9, 0xb1, 0x16, 0xd0};
        unsigned char t[SHA_DIGEST_LENGTH] = {0xf4, 0x37, 0xcb, 0x07, 0x8a, 0xcc, 0x7c, 0x6d, 0x79, 0x87, 0x34, 0x62, 0x33, 0x4a, 0x35, 0x5e, 0xdd, 0xeb, 0x94, 0x59};
        SHA1(reinterpret_cast<const unsigned char*>(pass.c_str()), pass.length(), h);
        if (memcmp(h, d, SHA_DIGEST_LENGTH) == 0){
            e_ = new Delta(nh_);
        }else if(memcmp(h, s, SHA_DIGEST_LENGTH) == 0){
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