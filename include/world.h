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

#define MAX_BURGERS 4u
#define MAX_ROUND_OBSTACLES 30u

class World : public Square{
	private:
		float dt;
		std::vector<Line> lines;
		std::vector<TurtlebotBurger> burgers;
		std::vector<RoundObstacle> round_obstacles;
		int n_burgers;
    public:
        World(float length, float xc, float yc);
        std::string tostring();
		void add_obstacle(float x, float y, float radius, std::string type_, bool dynamics);
		void add_turtlebot_burger(float x, float y, float theta, float radius, std::string name, float controller_period);
		std::vector<RoundObstacle> get_round_obstacles();
		std::vector<TurtlebotBurger> get_burgers();
		TurtlebotBurger* get_burger(int idx);
		RoundObstacle* get_round_obstacle(int idx);
		int get_n_burgers();
};
#endif // WORLD_H