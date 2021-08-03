#include "world.h"
/* World */
// World class
World::World(float length, float xc, float yc): Square(length, xc, yc){
	// TO DO: Get values below from json or xml file
	this->lines.reserve(4); // Max edges
	this->round_obstacles.reserve(MAX_ROUND_OBSTACLES); // Max round obstacles
	this->burgers.reserve(MAX_BURGERS); // Max burgers
	this->n_burgers = 0; // counter of burgers
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
	std::vector<TurtlebotBurger>::iterator it; 
	it = this->burgers.begin() + this->n_burgers;
  	it = this->burgers.insert(it, TurtlebotBurger(x,y,theta,radius,dt,name));
	this->n_burgers += 1;
}

int World::get_n_burgers(){
	return this->n_burgers;
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

