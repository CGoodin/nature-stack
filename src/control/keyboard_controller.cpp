// class definition
#include "nature/control/keyboard_controller.h"
// c++ includes
#include <iostream>
#include <cstdlib>
// system includes
#include <ncurses.h>

namespace nature {
namespace control{

int KeyboardController::kbhit(void){
    int ch = getch();
    if (ch != ERR) {
        ungetch(ch);
        return 1;
    } 
    else {
        return 0;
    }
}

KeyboardController::KeyboardController(){
    throttle_ = 0.0f;
    steering_ = 0.0f;
    braking_ = 0.0f;
    d_throt_ = 0.005f;
    d_steer_ = 0.0025f;
    d_brake_ = 0.01f;
    steering_decay_rate_ = 0.8f;
    initscr(); //Start curses mode
    clear();
    noecho();
    cbreak();
    nodelay(stdscr, TRUE);
    scrollok(stdscr, TRUE);
    printw("Drive with the W-A-S-D keys");
}

KeyboardController::~KeyboardController(){
    endwin(); //End curses mode	
}

void KeyboardController::Update(){

    bool throt = false; 
    bool brake = false; 
    bool left = false; 
    bool right = false; 

    // w = 119
    // a = 97
    // s = 115
    // d = 100
    if (kbhit()) {
        int pressed = getch();
        if (pressed == 119 ) throt = true;
        if (pressed == 115 ) brake = true;
        if (pressed == 97 ) left = true;
        if (pressed == 100 ) right = true;
        std::string status_str = "Throttle="+std::to_string(throttle_)+", braking="+std::to_string(braking_)+", steering="+std::to_string(steering_)+"\n";
        printw(status_str.c_str());
        refresh();
    } 
    else {
        refresh();
    }  

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
    else{
        steering_ *= steering_decay_rate_;
    }

    throttle_ = std::max(0.0f,std::min(1.0f, throttle_));
    braking_ = std::max(0.0f,std::min(1.0f, braking_));
    steering_ = std::max(-1.0f,std::min(1.0f, steering_));
}

} // namespace control
} // namespace nature