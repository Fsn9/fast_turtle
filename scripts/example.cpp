// Simulator classes
#include "fast_turtle.h"
#include <ros/ros.h>

int main(int argc, char **argv)
{
    FastTurtle ft;
    float obstacle_radius = 0.15;
    ft.init_world(4, 0, 0,"square");
    ft.add_turtlebot_burger(0, -1, -M_PI_2, 0.09, 1, "michelangelo");
    ft.add_obstacle(0, -2, obstacle_radius, "round", false);
    ft.add_obstacle(0, 2, obstacle_radius, "round", false);
    ft.add_obstacle(-1, -1, obstacle_radius, "round", false);
    ft.add_obstacle(-1, -2, obstacle_radius, "round", false);
    float v = 0.1;
    float w = 0.0;
    int idx_robot = 0;
    Observation observation;

    while(1){
        ft.act(v, w, idx_robot);
        observation = ft.observe(idx_robot);
        observation.print();
    }
    
    return 0;
}
