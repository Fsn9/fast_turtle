#ifndef WORLD_H
#define WORLD_H

#include <iostream>
#include <string.h>
#include <vector>
#include <map>
#include <memory>

#include "utils.h"
#include "geometry.h"
#include "goal.h"
#include "obstacles.h"
#include "robot.h"

#define MAX_BURGERS 10u
#define MAX_SIMPLE_DRONES 50u
#define MAX_ROUND_OBSTACLES 30u
#define MAX_WALL_OBSTACLES 30u
#define MAX_FOODS 5u
#define MIN_HEIGHT_DRONES 0.10f

class World : public Square{
	private:
		double dt;
		std::vector<Line> lines;
		std::vector<TurtlebotBurger> burgers;
		std::vector<RoundObstacle> round_obstacles;
		std::vector<WallObstacle> wall_obstacles;
		std::vector<FoodItem> food_items;
		std::vector<std::shared_ptr<SimpleDrone>> simple_drones;
		std::vector<std::shared_ptr<Drone3D>> _3d_drones;
		int n_burgers;
		int n_simple_drones;
		int n_3d_drones;
	public:
		World(double length, double xc, double yc, double angle);
		std::string tostring();
		void add_obstacle(double x, double y, double radius, std::string type_);
		void add_wall(double x1, double y1, double x2, double y2);
		void add_turtlebot_burger(double x, double y, double theta, double radius, std::string name, double controller_period);
		void add_simple_drone(double x, double y, double height, double radius, std::string name, double controller_period);
		void add_3d_drone(double x, double y, double z, double height, double radius, std::string name, double controller_period);
		void add_food_item(double x, double y, double radius);
		std::vector<RoundObstacle> get_round_obstacles();
		std::vector<WallObstacle> get_wall_obstacles();
		std::vector<TurtlebotBurger> get_burgers();
		std::vector<FoodItem> get_food_items();
		std::vector<std::shared_ptr<SimpleDrone>> get_simple_drones();
		std::vector<std::shared_ptr<Drone3D>> get_3d_drones();
		RoundObstacle* get_round_obstacle(int idx);
		WallObstacle* get_wall_obstacle(int idx);
		TurtlebotBurger* get_burger(int idx);
		FoodItem* get_food_item(int idx);
		std::shared_ptr<SimpleDrone> get_simple_drone(int idx);
		std::shared_ptr<SimpleDrone> get_simple_drone(std::string name);
		std::shared_ptr<Drone3D> get_3d_drone(int idx);
		std::shared_ptr<Drone3D> get_3d_drone(std::string name);
		void set_simple_drone_position(std::string name, double x, double y);
		void set_3d_drone_position(std::string name, double x, double y, double z);
		void reset_simple_drones();
		void reset_3d_drones();
		int get_n_burgers();
		int get_n_simple_drones();
		int get_n_3d_drones();
		std::vector<std::string> get_robot_names();
		void set_robot_positions(const std::map<std::string, std::pair<double, double>>& position);
};
#endif // WORLD_H