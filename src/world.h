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

class World : public Square{
	private:
		float dt;
		std::vector<Line> lines;
		std::vector<TurtlebotBurger> burgers;
		//std::vector<Obstacle> squared_obstacles;
		std::vector<RoundObstacle> round_obstacles;
    public:
        World(float length, float xc, float yc);
        std::string tostring();
		void add_obstacle(float x, float y, float radius, std::string type_, bool dynamics);
		void add_turtlebot_burger(float x, float y, float theta, float radius, float dt, std::string name);
		std::vector<RoundObstacle> get_round_obstacles();
		std::vector<TurtlebotBurger> get_burgers();
		TurtlebotBurger* get_burger(int idx);
		RoundObstacle* get_round_obstacle(int idx);
};
#endif