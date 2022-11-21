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
// nature includes
#include <nature/CImg.h>

namespace nature {
namespace control{

class KeyboardController{
public:
    KeyboardController();

    void Update();

private:
    float throttle_;
    float braking_;
    float steering_;
    float d_throt_;
    float d_steer_;
    float d_brake_;

    cimg_library::CImg<float> image_;
	cimg_library::CImgDisplay disp_;
};

} // namespace nature
} // control

#endif //