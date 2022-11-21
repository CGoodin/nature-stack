/**
* \class KeyboardController
*
* A controller to get driving commands from the keyboard and visualize state
*
* \author Chris Goodin
*
* \date 11/18/22
*/
#ifndef KEYBOARD_CONTROLLER_H
#define KEYBOARD_CONTROLLER_H

namespace nature {
namespace control{

class KeyboardController{
public:
    KeyboardController();

    ~KeyboardController();

    void Update();

    float GetThrottle(){ return throttle_; }

    float GetBraking(){ return braking_; }

    float GetSteering(){ return steering_; }

    void SetCurrentThrottle(float throt){ throttle_ = throt; }

    void SetCurrentBraking(float brake){ braking_ = brake; }

    void SetCurrentSteering(float steer){ steering_ = steer; }

private:
    int kbhit(void);
    float throttle_;
    float braking_;
    float steering_;
    float d_throt_;
    float d_steer_;
    float d_brake_;
    float steering_decay_rate_;
};

} // namespace nature
} // control

#endif //