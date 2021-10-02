#include "obstacles.h"

RoundObstacle::RoundObstacle(float xc, float yc, float radius) : Circle(xc, yc, radius){
	this->dynamics = dynamics;
}

std::string RoundObstacle::tostring(){
    return "(RoundObstacle) " + Circle::tostring() + ")\n";
}

WallObstacle::WallObstacle(float length, float xc, float yc, float angle) : Square(length, xc, yc, angle){
	this->dynamics = dynamics;
}

std::string WallObstacle::tostring(){
    return "(WallObstacle) " + Square::tostring() + ")\n";
}

FoodItem::FoodItem(float xc, float yc, float radius) : Circle(xc, yc, radius){
    this->dynamics = dynamics;
}

std::string FoodItem::tostring(){
    return "(FoodItem) " + Circle::tostring() + ")\n";
}