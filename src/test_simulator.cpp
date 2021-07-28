// Simulator classes
#include "fast_turtle.h"
#include <chrono>

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
    std::chrono::steady_clock::time_point begin; 
    std::chrono::steady_clock::time_point end;
    while(1){
        begin = std::chrono::steady_clock::now();
        ft.act(v,w);
        end = std::chrono::steady_clock::now();
        std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[µs]" << std::endl; 
    }
    
    return 0;
}
