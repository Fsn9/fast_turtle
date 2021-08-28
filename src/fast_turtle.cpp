#include "fast_turtle.h"

FastTurtle::FastTurtle(){
    std::cout << "--Fast Turtle simulator created --\n";
    this->last_times.reserve(MAX_BURGERS);
}

FastTurtle::FastTurtle(unsigned int simulation_fps){
    std::cout << "--Fast Turtle simulator created --\n";
    this->last_times.reserve(MAX_BURGERS);
    this->simulation_fps = simulation_fps;
    this->simulation_dt = 1.0 / this->simulation_fps;
}

void FastTurtle::init_world(float length, float xc, float yc, std::string type = "square"){
    this->w = new World(length, xc, yc);
}

void FastTurtle::add_turtlebot_burger(float x, float y, float theta, float radius, std::string name, float controller_period){
    if (this->get_world()->get_n_burgers() < MAX_BURGERS){
        this->last_times[this->w->get_n_burgers()] = std::chrono::steady_clock::now();
        this->w->add_turtlebot_burger(x,y,theta,radius,name,controller_period);
    }
    else{
        throw std::invalid_argument("No more robots allowed. Maximum is" + std::to_string(MAX_BURGERS));
    }
}

void FastTurtle::add_obstacle(float x, float y, float radius, std::string type_){
    this->w->add_obstacle(x,y,radius,type_);
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
    if (idx_robot < 0 || idx_robot > this->w->get_n_burgers() - 1){
        throw std::invalid_argument("invalid robot index of " + 
        std::to_string(idx_robot) +". It needs to be >= 0 or < " + 
        std::to_string(this->w->get_burgers().size()));
    }
    // Check current time
    std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();

    // Measure time passed since robot last actuation time
    double elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(now - this->last_times[idx_robot]).count() * 1e-9;

    // If duration is bigger than the controller rate, then controller can act
    if(elapsed > this->w->get_burger(idx_robot)->get_controller_period()){
        // Update last time
        this->last_times[idx_robot] = now;
        // Update last twist values
        this->w->get_burger(idx_robot)->set_new_v_w(v,w);
    }

    // Move robot
    this->w->get_burger(idx_robot)->move(
        this->w->get_burger(idx_robot)->get_last_v(),
        this->w->get_burger(idx_robot)->get_last_w(),
        this->simulation_dt
    );

    // Update robot lidar
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
    std::cout << "\nPose of robot " << this->idx_robot << ": \n";
    for(std::vector<float>::const_iterator i = this->pose.begin(); i!=this->pose.end(); ++i)
        std::cout << *i << ' ';
    std::cout << "\n";
}

void Observation::print_laser_data(){
    std::cout << "\nLaser data of robot " << this->idx_robot << ": \n";
    for(std::vector<float>::const_iterator i = this->laser_data.begin(); i!=this->laser_data.end(); ++i)
        std::cout << *i << ' ';
    std::cout << "\n";
}

Observation::Observation(std::vector<float> pose, std::vector<float> laser_data, int idx_robot) : pose(pose), laser_data(laser_data), idx_robot(idx_robot){}

Observation::Observation(){}