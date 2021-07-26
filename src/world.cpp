#include "world.h"
/* World */
// World class
World::World(float length, float xc, float yc): Square(length, xc, yc){
	// Parameter file upload
	this->lines.reserve(4);
	this->round_obstacles.reserve(30);
	this->burgers.reserve(4);
	
	// Goal
}

void World::add_obstacle(float x, float y, float radius, std::string type_, bool dynamics){
	if (type_.compare("round") == 0) this->round_obstacles.push_back(RoundObstacle(x,y,radius,dynamics));
	else if (type_.compare("squared") == 0) {}
	else{
		std::cout << "Not valid type";
	}
	return;
}

void World::add_turtlebot_burger(float x, float y, float theta, float radius, float dt, std::string name){
	this->burgers.push_back(TurtlebotBurger(x,y,theta,radius,dt,name));
}

std::vector<RoundObstacle> World::get_round_obstacles(){
	return this->round_obstacles;
}

RoundObstacle* World::get_round_obstacle(int idx){
	return &this->round_obstacles[idx];
}

std::string World::tostring(){
	std::string repr_ = "";
    repr_ += "--Squared World--\n" + Square::tostring();
	for(int idx = 0; idx < this->round_obstacles.size(); idx++){
		repr_ += this->round_obstacles[idx].tostring();
	}
	for(int idx = 0; idx < this->burgers.size(); idx++){
		repr_ += this->burgers[idx].tostring();
	}
	return repr_ + "\n";
}

std::vector<TurtlebotBurger> World::get_burgers(){
	return this->burgers;
}

TurtlebotBurger* World::get_burger(int idx){
	return &this->burgers[idx];
}

