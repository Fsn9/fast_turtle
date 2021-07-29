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

// Returns lidar measurements from robot with index = idx_robot
std::vector<float> FastTurtle::observe_robot_lidar(int idx_robot){
    if (idx_robot < 0 || idx_robot > this->w->get_burgers().size() - 1){
        throw std::invalid_argument("invalid robot index of " + 
        std::to_string(idx_robot) +". It needs to be >= 0 or < " + 
        std::to_string(this->w->get_burgers().size()));
    }
    return this->w->get_burger(idx_robot)->get_lidar()->get_lasers();
}

// Returns lidar measurements from robot with index = idx_robot plus robot pose data
Observation FastTurtle::observe(int idx_robot){
    return Observation(
        this->observe_robot_pose(idx_robot),
        this->observe_robot_lidar(idx_robot), 
        idx_robot
    );
}

// Acts with twist message in robot with idx_robot
void FastTurtle::act(float v, float w, int idx_robot){
    this->w->get_burger(idx_robot)->move(v,w);
    this->w->get_burger(idx_robot)->get_lidar()->update_lidar_heavy(
        this->w->get_round_obstacles(), 
        this->w->get_edges(), 
        this->get_world()->get_burger(idx_robot)->get_xc(), 
        this->get_world()->get_burger(idx_robot)->get_yc(), 
        this->get_world()->get_burger(idx_robot)->get_theta()
    );
}

std::vector<float> Observation::get_pose(){
    return this->pose;
}

std::vector<float> Observation::get_laser_data(){
    return this->laser_data;
}

void Observation::print(){
    std::cout << "\n[Observation of robot " << this->idx_robot << "]";
    this->print_laser_data();
    this->print_pose();
}

void Observation::print_pose(){
    std::cout << "\nPose: \n";
    for(std::vector<float>::const_iterator i = this->pose.begin(); i!=this->pose.end(); ++i)
        std::cout << *i << ' ';
    std::cout << "\n";
}

void Observation::print_laser_data(){
    std::cout << "\nLaser data: \n";
    for(std::vector<float>::const_iterator i = this->laser_data.begin(); i!=this->laser_data.end(); ++i)
        std::cout << *i << ' ';
    std::cout << "\n";
}

Observation::Observation(std::vector<float> pose, std::vector<float> laser_data, int idx_robot) : pose(pose), laser_data(laser_data), idx_robot(idx_robot){}

Observation::Observation(){}