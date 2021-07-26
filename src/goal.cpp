#include "goal.h"
// Goal class
Goal::Goal(float xc, float yc, float radius, bool dynamics = false) : Circle(xc, yc, radius){
    this->dynamics = dynamics;
}
std::string Goal::tostring(){
    return "Goal\n" + Circle::tostring();
}