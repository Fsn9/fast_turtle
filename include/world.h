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
#define MAX_ROUND_OBSTACLES 30u
#define MAX_WALL_OBSTACLES 30u
#define MAX_FOODS 5u

class World : public Square{
	private:
		float dt;
		std::vector<Line> lines;
		std::vector<TurtlebotBurger> burgers;
		std::vector<RoundObstacle> round_obstacles;
		std::vector<WallObstacle> wall_obstacles;
		std::vector<FoodItem> food_items;
		int n_burgers;
  public:
    World(float length, float xc, float yc, float angle);
    std::string tostring();
		void add_obstacle(float x, float y, float radius, std::string type_);
		void add_wall(float x1, float y1, float x2, float y2);
		void add_turtlebot_burger(float x, float y, float theta, float radius, std::string name, float controller_period);
		void add_food_item(float x, float y, float radius);
		std::vector<RoundObstacle> get_round_obstacles();
		std::vector<WallObstacle> get_wall_obstacles();
		std::vector<TurtlebotBurger> get_burgers();
		std::vector<FoodItem> get_food_items();
		TurtlebotBurger* get_burger(int idx);
		RoundObstacle* get_round_obstacle(int idx);
		WallObstacle* get_wall_obstacle(int idx);
		FoodItem* get_food_item(int idx);
		int get_n_burgers();
};
#endif // WORLD_H