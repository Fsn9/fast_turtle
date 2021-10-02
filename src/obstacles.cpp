#include "obstacles.h"
// Round Obstacle
RoundObstacle::RoundObstacle(float xc, float yc, float radius) : Circle(xc, yc, radius){
	this->dynamics = dynamics;
}
std::string RoundObstacle::tostring(){
    return "(RoundObstacle) " + Circle::tostring() + ")\n";
}

FoodItem::FoodItem(float xc, float yc, float radius) : Circle(xc, yc, radius){
    this->dynamics = dynamics;
}

std::string FoodItem::tostring(){
    return "(FoodItem) " + Circle::tostring() + ")\n";
}