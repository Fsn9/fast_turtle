#ifndef GOAL_H
#define GOAL_H
#include <string>
#include "geometry.h"

class Goal : public Circle{
    private:
        float dynamics;
    public:
        Goal(float xc, float yc, float radius, bool dynamics);
        std::string tostring();
};
#endif