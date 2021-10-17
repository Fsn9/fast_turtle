#include "obstacles.h"

RoundObstacle::RoundObstacle(float xc, float yc, float radius) : Circle(xc, yc, radius){
	this->dynamics = dynamics;
}

std::string RoundObstacle::tostring(){
    return "(RoundObstacle) " + Circle::tostring() + ")\n";
}

WallObstacle::WallObstacle(float x1, float y1, float x2, float y2) : LineSegment(x1, y1, x2, y2){
	this->dynamics = dynamics;
}

std::string WallObstacle::tostring(){
    return "(WallObstacle) " + LineSegment::tostring() + ")\n";
}

FoodItem::FoodItem(float xc, float yc, float radius) : Circle(xc, yc, radius){
    this->dynamics = dynamics;
}

std::string FoodItem::tostring(){
    return "(FoodItem) " + Circle::tostring() + ")\n";
}