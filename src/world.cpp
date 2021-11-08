#include "world.h"
#include <ros/ros.h>

World::World(float length, float xc, float yc, float angle): Square(length, xc, yc, angle){
	this->lines.reserve(4); // Max edges
	this->round_obstacles.reserve(MAX_ROUND_OBSTACLES); // Max round obstacles
	this->wall_obstacles.reserve(MAX_WALL_OBSTACLES); // Max wall obstacles
	this->burgers.reserve(MAX_BURGERS); // Max burgers
	this->n_burgers = 0; // counter of burgers
	this->food_items.reserve(MAX_FOODS); // Max food
	this->simple_drones.reserve(MAX_SIMPLE_DRONES);
	this->n_simple_drones = 0; // counter of burgers
}

void World::add_obstacle(float x, float y, float radius, std::string type_){
	if (type_.compare("round") == 0) this->round_obstacles.push_back(RoundObstacle(x,y,radius));
	else if (type_.compare("squared") == 0) {}
	else{
		std::cout << "Not valid type";
	}
	return;
}

void World::add_wall(float x1, float y1, float x2, float y2){
	this->wall_obstacles.push_back(WallObstacle(x1, y1, x2, y2));
	return;
}


void World::add_turtlebot_burger(float x, float y, float theta, float radius, std::string name, float controller_period){
	std::vector<TurtlebotBurger>::iterator it; 
	it = this->burgers.begin() + this->n_burgers;
	it = this->burgers.insert(it, TurtlebotBurger(x,y,theta,radius,name,controller_period));
	this->n_burgers += 1;
}

void World::add_simple_drone(float x, float y, float height, float radius, std::string name, float controller_period){
	std::vector<SimpleDrone>::iterator it; 
	it = this->simple_drones.begin() + this->n_simple_drones;
	it = this->simple_drones.insert(it, SimpleDrone(x,y,height,radius,name,controller_period));
	this->n_simple_drones += 1;
}

void World::add_food_item(float xc, float yc, float radius){
	this->food_items.push_back(FoodItem(xc, yc, radius));
	return;
}

int World::get_n_burgers(){
	return this->n_burgers;
}

int World::get_n_simple_drones(){
	return this->n_simple_drones;
}

std::vector<RoundObstacle> World::get_round_obstacles(){
	return this->round_obstacles;
}

RoundObstacle* World::get_round_obstacle(int idx){
	return &this->round_obstacles[idx];
}


std::vector<WallObstacle> World::get_wall_obstacles(){
	return this->wall_obstacles;
}

WallObstacle* World::get_wall_obstacle(int idx){
	return &this->wall_obstacles[idx];
}

std::string World::tostring(){
	std::string repr_ = "";
    repr_ += "--Squared World--\n" + Square::tostring();
	for(int idx = 0; idx < this->round_obstacles.size(); idx++){
		repr_ += this->round_obstacles[idx].tostring();
	}
	for(int idx = 0; idx < this->wall_obstacles.size(); idx++){
		repr_ += this->wall_obstacles[idx].tostring();
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

std::vector<FoodItem> World::get_food_items(){
	return this->food_items;
}

FoodItem* World::get_food_item(int idx){
	return &this->food_items[idx];
}
std::vector<SimpleDrone> World::get_simple_drones(){
	return this->simple_drones;
}

SimpleDrone* World::get_simple_drone(int idx){
	return &this->simple_drones[idx];
}

// For now we work with drones only
std::vector<std::string> World::get_robot_names()
{
	std::vector<std::string> names;
	for(SimpleDrone robot : this->simple_drones)
	{
		names.emplace_back(robot.get_name());
	}
	return names;
}
