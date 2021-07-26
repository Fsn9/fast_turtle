#include "fast_turtle.h"

FastTurtle::FastTurtle(){}

void FastTurtle::init_world(float length, float xc, float yc, std::string type = "square"){
    this->w = new World(length, xc, yc);
}

void FastTurtle::add_turtlebot_burger(float x, float y, float theta, float radius, float dt, std::string name){
    this->w->add_turtlebot_burger(x,y,theta,radius,dt,name);
}

void FastTurtle::add_obstacle(float x, float y, float radius, std::string type_, bool dynamics){
    this->w->add_obstacle(x,y,radius,type_,dynamics);
}

World* FastTurtle::get_world(){
    return this->w;
}

std::vector<float> FastTurtle::observe_robot_pose(int idx_robot){
    if (idx_robot < 0 || idx_robot > this->w->get_burgers().size() - 1){
        throw std::invalid_argument("invalid robot index of " + 
        std::to_string(idx_robot) +". It needs to be >= 0 or < " + 
        std::to_string(this->w->get_burgers().size()));
    }
    return std::vector<float> {this->w->get_burger(idx_robot)->x(), this->w->get_burger(idx_robot)->y(), this->w->get_burger(idx_robot)->get_theta()};
}

std::vector<float> FastTurtle::observe_robot_lidar(int idx_robot){
    return this->w->get_burger(idx_robot)->get_lidar()->get_lasers();
}

int FastTurtle::act(float v, float w){
    this->w->get_burger(0)->update_lidar_heavy(this->w->get_round_obstacles(), this->w->get_edges());
    return 0;
}



