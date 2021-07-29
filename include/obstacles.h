#ifndef OBSTACLES_H
#define OBSTACLES_H
#include "geometry.h"
#include "utils.h"

#ifndef OBSTACLE_H
#define OBSTACLE_H
class Obstacle{
	protected:
		bool dynamics;
};
#endif

#ifndef ROUNDOBSTACLE_H
#define ROUNDOBSTACLE_H
class RoundObstacle : public Obstacle, public Circle{
    public:
        RoundObstacle(float xc, float yc, float radius, bool dynamics);
        std::string tostring();
};
#endif
#endif