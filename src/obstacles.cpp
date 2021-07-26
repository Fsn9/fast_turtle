#include "obstacles.h"
// Round Obstacle
RoundObstacle::RoundObstacle(float xc, float yc, float radius, bool dynamics = false) : Circle(xc, yc, radius){
	this->dynamics = dynamics;
}
std::string RoundObstacle::tostring(){
    return "(RoundObstacle) " + Circle::tostring() + ") and dynamics is " + bool_to_string(this->dynamics) + "\n";
}