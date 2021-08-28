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
#endif // OBSTACLES_H