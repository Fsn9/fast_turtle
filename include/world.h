#ifndef WORLD_H
#define WORLD_H

#include <iostream>
#include <string.h>
#include <vector>

#include "utils.h"
#include "geometry.h"
#include "goal.h"
#include "obstacles.h"
#include "robot.h"

#define MAX_BURGERS 10u
#define MAX_SIMPLE_DRONES 10u
#define MAX_ROUND_OBSTACLES 30u
#define MAX_WALL_OBSTACLES 30u
#define MAX_FOODS 5u
#define MIN_HEIGHT_DRONES 0.10f

class World : public Square{
	private:
		float dt;
		std::vector<Line> lines;
		std::vector<TurtlebotBurger> burgers;
		std::vector<RoundObstacle> round_obstacles;
		std::vector<WallObstacle> wall_obstacles;
		std::vector<FoodItem> food_items;
		std::vector<SimpleDrone> simple_drones;
		int n_burgers;
		int n_simple_drones;
  public:
    World(float length, float xc, float yc, float angle);
    std::string tostring();
	void add_obstacle(float x, float y, float radius, std::string type_);
	void add_wall(float x1, float y1, float x2, float y2);
	void add_turtlebot_burger(float x, float y, float theta, float radius, std::string name, float controller_period);
	void add_simple_drone(float x, float y, float height, float radius, std::string name, float controller_period);
	void add_food_item(float x, float y, float radius);
	std::vector<RoundObstacle> get_round_obstacles();
	std::vector<WallObstacle> get_wall_obstacles();
	std::vector<TurtlebotBurger> get_burgers();
	std::vector<FoodItem> get_food_items();
	std::vector<SimpleDrone> get_simple_drones();
	RoundObstacle* get_round_obstacle(int idx);
	WallObstacle* get_wall_obstacle(int idx);
	TurtlebotBurger* get_burger(int idx);
	FoodItem* get_food_item(int idx);
	SimpleDrone* get_simple_drone(int idx);
	int get_n_burgers();
	int get_n_simple_drones();
	std::vector<std::string> get_robot_names();
};
#endif // WORLD_H