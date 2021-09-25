#ifndef OBSTACLES_H
#define OBSTACLES_H
#include "geometry.h"
#include "utils.h"

class Obstacle{
	protected:
		bool dynamics;
};

class RoundObstacle : public Obstacle, public Circle{
    public:
        RoundObstacle(float xc, float yc, float radius);
        std::string tostring();
};

class FoodItem : public Obstacle, public Circle{
    public:
        FoodItem(float xc, float yc, float radius);
        std::string toString();
        bool visible = true; //If the food was captured
        int robot = -1;  //-1  if the food is free, robot_id if it was captured
};

#endif // OBSTACLES_H