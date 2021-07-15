#include <ros/ros.h>
#include "omega/enigma.hpp"
#include "omega/delta.hpp"
#include "omega/zeta.hpp"
#include "omega/theta.hpp"

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
        unsigned char d[SHA_DIGEST_LENGTH] = {0x2e, 0x89, 0x90, 0xa8, 0xf8, 0xd3, 0x14, 0x1e, 0x33, 0x36, 0x39, 0x60, 0x7c, 0x1f, 0x9d, 0xa0, 0x70, 0x11, 0xa9, 0x6d};
        unsigned char s[SHA_DIGEST_LENGTH] = {0x8f, 0x0f, 0xdd, 0x04, 0x48, 0x74, 0xef, 0xda, 0x2a, 0xa1, 0xf7, 0xd7, 0xef, 0xd3, 0xa0, 0xb0, 0x2a, 0xed, 0x96, 0x94};
        unsigned char t[SHA_DIGEST_LENGTH] = {0xfa, 0x35, 0x6d, 0x3e, 0x57, 0x0e, 0x38, 0x74, 0xc4, 0x90, 0xa4, 0x27, 0x83, 0x4a, 0xa6, 0x12, 0xc5, 0x43, 0x61, 0xf5};
        unsigned char T[SHA_DIGEST_LENGTH] = {0xd5, 0x0c, 0x78, 0xc0, 0x0a, 0x0c, 0x23, 0x0d, 0xf2, 0x1f, 0x8e, 0x8a, 0xb2, 0x93, 0x32, 0x04, 0xe0, 0xb8, 0xf9, 0x72};
        SHA1(reinterpret_cast<const unsigned char*>(pass.c_str()), pass.length(), h);
        std::cout << std::endl;
        if (memcmp(h, d, SHA_DIGEST_LENGTH) == 0){
            e_ = new Delta(nh_);
        }else if(memcmp(h, s, SHA_DIGEST_LENGTH) == 0){
            e_ = new Zeta(nh_);
        }else if(memcmp(h, t, SHA_DIGEST_LENGTH) == 0 || memcmp(h, T, SHA_DIGEST_LENGTH) == 0){
            e_ = new Theta(nh_);
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