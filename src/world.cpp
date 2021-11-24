#include "world.h"

World::World(double length, double xc, double yc, double angle): Square(length, xc, yc, angle){
	this->lines.reserve(4); // Max edges
	this->round_obstacles.reserve(MAX_ROUND_OBSTACLES); // Max round obstacles
	this->wall_obstacles.reserve(MAX_WALL_OBSTACLES); // Max wall obstacles
	this->burgers.reserve(MAX_BURGERS); // Max burgers
	this->n_burgers = 0; // counter of burgers
	this->food_items.reserve(MAX_FOODS); // Max food
	this->simple_drones.reserve(MAX_SIMPLE_DRONES);
	this->n_simple_drones = 0; // counter of burgers
}

void World::add_obstacle(double x, double y, double radius, std::string type_){
	if (type_.compare("round") == 0) this->round_obstacles.push_back(RoundObstacle(x,y,radius));
	else if (type_.compare("squared") == 0) {}
	else{
		std::cout << "Not valid type";
	}
	return;
}

void World::add_wall(double x1, double y1, double x2, double y2){
	this->wall_obstacles.push_back(WallObstacle(x1, y1, x2, y2));
	return;
}


void World::add_turtlebot_burger(double x, double y, double theta, double radius, std::string name, double controller_period){
	std::vector<TurtlebotBurger>::iterator it; 
	it = this->burgers.begin() + this->n_burgers;
	it = this->burgers.insert(it, TurtlebotBurger(x,y,theta,radius,name,controller_period));
	this->n_burgers += 1;
}

void World::add_simple_drone(double x, double y, double height, double radius, std::string name, double controller_period){
	std::vector<std::shared_ptr<SimpleDrone>>::iterator it; 
	it = this->simple_drones.begin() + this->n_simple_drones;
	it = this->simple_drones.insert(it, std::make_shared<SimpleDrone>(x,y,height,radius,name,controller_period));
	this->n_simple_drones += 1;
}

void World::add_food_item(double xc, double yc, double radius){
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
std::vector<std::shared_ptr<SimpleDrone>> World::get_simple_drones(){
	return this->simple_drones;
}

std::shared_ptr<SimpleDrone> World::get_simple_drone(int idx){
	return this->simple_drones[idx];
}

// For now we work with drones only
std::vector<std::string> World::get_robot_names()
{
	std::vector<std::string> names;
	for(std::shared_ptr<SimpleDrone> robot : this->simple_drones)
	{
		names.emplace_back(robot->get_name());
	}
	return names;
}

std::shared_ptr<SimpleDrone> World::get_simple_drone(std::string name)
{
	for(std::shared_ptr<SimpleDrone> drone : simple_drones)
	{
		if(drone->get_name() == name)
		{
			return drone;
		}
	}
	return {};
}

void World::set_simple_drone_position(std::string name, double x, double y)
{
	std::shared_ptr<SimpleDrone> sd = get_simple_drone(name);
	if(sd != nullptr)
	{
		sd->set_xc(x);
		sd->set_yc(y);
	}
	
}

void World::reset_simple_drones()
{
	for(std::shared_ptr<SimpleDrone> drone : simple_drones)
	{
		drone->reset();
	}
}

void World::set_robot_positions(const std::map<std::string, std::pair<double, double>>& positions)
{
	for(std::pair<std::string, std::pair<double, double>> robot : positions)
	{
		set_simple_drone_position(robot.first, std::get<0>(robot.second), std::get<1>(robot.second));
	}
}