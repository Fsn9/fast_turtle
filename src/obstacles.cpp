#include "obstacles.h"
// Round Obstacle
RoundObstacle::RoundObstacle(float xc, float yc, float radius) : Circle(xc, yc, radius){
	this->dynamics = dynamics;
}
std::string RoundObstacle::tostring(){
    return "(RoundObstacle) " + Circle::tostring() + ")\n";
}

// Wall Obstacle
WallObstacle::WallObstacle(float length, float xc, float yc, float angle) : Square(length, xc, yc, angle){
	this->dynamics = dynamics;
}
std::string WallObstacle::tostring(){
    return "(WallObstacle) " + Square::tostring() + ")\n";
}