// class definition
#include "nature/control/keyboard_controller.h"
// c++ includes
#include <iostream>

namespace nature {
namespace control{

static const float green[3] = {0.0f, 255.0f, 0.0f};

KeyboardController::KeyboardController(){
    image_.assign(512, 512, 1, 3, 0.);
    throttle_ = 0.0f;
    steering_ = 0.0f;
    braking_ = 0.0f;
    d_throt_ = 0.005f;
    d_steer_ = 0.005f;
    d_brake_ = 0.01f;
    
}

void KeyboardController::Update(){
    disp_.set_title("Keyboard Controller");
    disp_ = image_;

    bool throt = disp_.is_keyARROWUP();
    bool brake = disp_.is_keyARROWDOWN();
    bool left = disp_.is_keyARROWLEFT();
    bool right = disp_.is_keyARROWRIGHT();

    if (brake){
        braking_ += d_brake_;
        throttle_ = 0.0f;
    }
    else if (throt){
        throttle_ += d_throt_;
        braking_ = 0.0f;
    }
    if (left){
        steering_ += d_steer_;
    }
    else if (right){
        steering_ -= d_steer_;
    }


    throttle_ = std::max(0.0f,std::min(1.0f, throttle_));
    braking_ = std::max(0.0f,std::min(1.0f, braking_));
    steering_ = std::max(-1.0f,std::min(1.0f, steering_));

    std::cout<<throttle_<<" "<<steering_<<" "<<braking_<<std::endl;
}

} // namespace control
} // namespace nature