#include "fast_turtle.h"
#include <algorithm>

FastTurtle::FastTurtle(){
    std::cout << "--Fast Turtle simulator created --\n";
    this->last_times_tb_robots.reserve(MAX_BURGERS);
    this->last_times_simple_drones.reserve(MAX_SIMPLE_DRONES);
    std::fill(default_scan_.begin(), default_scan_.begin() + N_LASERS, MAX_DISTANCE);
}

FastTurtle::FastTurtle(unsigned int simulation_fps){
    std::cout << "--Fast Turtle simulator created --\n";
    this->last_times_tb_robots.reserve(MAX_BURGERS);
    this->last_times_simple_drones.reserve(MAX_SIMPLE_DRONES);
    this->simulation_fps = simulation_fps;
    this->simulation_dt = 1.0 / this->simulation_fps;
}

void FastTurtle::init_world(float length, float xc, float yc, std::string type = "square"){
    this->w = new World(length, xc, yc, 0.0);
}

void FastTurtle::add_turtlebot_burger(float x, float y, float theta, float radius, std::string name, float controller_period){
    if (this->get_world()->get_n_burgers() < MAX_BURGERS){
        this->last_times_tb_robots[this->w->get_n_burgers()] = std::chrono::steady_clock::now();
        this->w->add_turtlebot_burger(x,y,theta,radius,name,controller_period);
    }
    else{
        throw std::invalid_argument("No more tb_robots allowed. Maximum is: " + std::to_string(MAX_BURGERS));
    }
}

void FastTurtle::add_simple_drone(float x, float y, float height, float radius, std::string name, float controller_period){
    if (height < MIN_HEIGHT_DRONES){
        throw std::invalid_argument("Entered height is invalid. Minimum height for drones is: " + std::to_string(MIN_HEIGHT_DRONES));
    }
    if (this->get_world()->get_n_simple_drones() < MAX_SIMPLE_DRONES){
        this->last_times_simple_drones[this->w->get_n_simple_drones()] = std::chrono::steady_clock::now();
        this->w->add_simple_drone(x,y,height,radius,name,controller_period);
    }
    else{
        throw std::invalid_argument("No more simple drones allowed. Maximum is: " + std::to_string(MAX_SIMPLE_DRONES));
    }
}

void FastTurtle::add_obstacle(float x, float y, float radius, std::string type_){
    this->w->add_obstacle(x,y,radius,type_);
}

void FastTurtle::add_wall(float x1, float y1, float x2, float y2){
    this->w->add_wall(x1, y1, x2, y2);
    
}

void FastTurtle::add_food_item(float x, float y, float radius){
    this->w->add_food_item(x,y,radius);
}

void FastTurtle::check_collisions(){
    for(int i = 0; i < this->get_world()->get_n_simple_drones(); i++){
        if(this->get_world()->get_simple_drone(i)->is_visible()){
            //check collisions between bots
            for(int j = i+1; j < this->get_world()->get_n_simple_drones(); j++){
                if(this->get_world()->get_simple_drone(j)->is_visible()){
                    if(this->get_world()->get_simple_drone(i)->intersects_circle(this->get_world()->get_simple_drone(j))){
                        this->get_world()->get_simple_drone(i)->set_visibility(false);
                        this->get_world()->get_simple_drone(j)->set_visibility(false);
                        std::cout << " collision bot";
                        break;
                    } 
                }
            }
            //check collisions with obstacles
            if(this->get_world()->get_simple_drone(i)->is_visible()){
                for(int j = 0; j < this->get_world()->get_round_obstacles().size(); j++){
                    if(this->get_world()->get_simple_drone(i)->intersects_circle(this->get_world()->get_round_obstacle(j))){
                        this->get_world()->get_simple_drone(i)->set_visibility(false);
                        std::cout << " collision obstacle";
                        break;
                    }
                }
            }
            //check collisions with walls
            if(this->get_world()->get_simple_drone(i)->is_visible()){
                for(int j = 0; j < this->get_world()->get_wall_obstacles().size(); j++){
                    if(std::get<0>(this->get_world()->get_wall_obstacle(j)->intersects_circle(this->get_world()->get_simple_drone(i)))){
                        float x1 = std::get<1>(this->get_world()->get_wall_obstacle(j)->intersects_circle(this->get_world()->get_simple_drone(i)));
                        float y1 = std::get<2>(this->get_world()->get_wall_obstacle(j)->intersects_circle(this->get_world()->get_simple_drone(i)));
                        float x2 = std::get<3>(this->get_world()->get_wall_obstacle(j)->intersects_circle(this->get_world()->get_simple_drone(i)));
                        float y2 = std::get<4>(this->get_world()->get_wall_obstacle(j)->intersects_circle(this->get_world()->get_simple_drone(i))); //pontos do circulo onde a reta intersecta
                        float xa = this->get_world()->get_wall_obstacle(j)->get_x1();
                        float ya = this->get_world()->get_wall_obstacle(j)->get_y1();
                        float xb = this->get_world()->get_wall_obstacle(j)->get_x2();
                        float yb = this->get_world()->get_wall_obstacle(j)->get_y2(); //extremidades da reta
                        
                        float crossproduct_1 = (y1 - ya) * (xb - xa) - (x1 - xa) * (yb - ya);
                        float crossproduct_2 = (y2 - ya) * (xb - xa) - (x2 - xa) * (yb - ya);

                        float dotproduct_1 = (x1 - xa) * (xb - xa) + (y1 - ya)*(yb - ya);
                        float dotproduct_2 = (x2 - xa) * (xb - xa) + (y2 - ya)*(yb - ya);

                        float squaredlengthba = (xb - xa)*(xb - xa) + (yb - ya)*(yb - ya);
                        if ((crossproduct_1 <= 0.0001  &&  dotproduct_1 > 0  &&  dotproduct_1 <= squaredlengthba) || 
                            (crossproduct_2 <= 0.0001  &&  dotproduct_2 > 0  &&  dotproduct_2 <= squaredlengthba)){
                                this->get_world()->get_simple_drone(i)->set_visibility(false);
                                std::cout << " collision wall "; //x: " << std::get<0>(this->get_world()->get_wall_obstacle(j)->get_midpoint()) << " robot x: " << this->get_world()->get_burger(i)->x();
                                break;
                            }
                      
                    }
                    //std::cout << " no collision wall x: " << std::get<0>(this->get_world()->get_wall_obstacle(j)->get_midpoint()) << " robot x: " << this->get_world()->get_burger(i)->x();
                }
            }
        }

    }
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
void FastTurtle::act_turtlebot_burger(float v, float w, int idx_tbb){
    if (idx_tbb < 0 || idx_tbb > this->w->get_n_burgers() - 1){
        throw std::invalid_argument("invalid tb_robot index of " + 
        std::to_string(idx_tbb) +". It needs to be >= 0 or < " + 
        std::to_string(this->w->get_burgers().size()));
    }
    // Check current time
    std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();

    // Measure time passed since tb_robot last actuation time
    double elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(now - this->last_times_tb_robots[idx_tbb]).count() * 1e-9;

    // If duration is bigger than the controller rate, then controller can act
    if(elapsed > this->w->get_burger(idx_tbb)->get_controller_period()){
        // Update last time
        this->last_times_tb_robots[idx_tbb] = now;
        // Update last twist values
        this->w->get_burger(idx_tbb)->set_new_v_w(v,w);
    }

    // Move tb_robot
    this->w->get_burger(idx_tbb)->move(
        this->w->get_burger(idx_tbb)->get_last_v(),
        this->w->get_burger(idx_tbb)->get_last_w(),
        this->simulation_dt
    );

    // Update tb_robot lidar
    this->w->get_burger(idx_tbb)->get_lidar()->update_lidar_heavy(
        this->w->get_round_obstacles(),
        this->w->get_simple_drones(), 
        this->w->get_edges(), 
        this->w->get_wall_obstacles(),
        this->get_world()->get_burger(idx_tbb)->get_xc(), 
        this->get_world()->get_burger(idx_tbb)->get_yc(), 
        this->get_world()->get_burger(idx_tbb)->get_theta()
    );
}

void FastTurtle::act_simple_drone(float vx, float vy, int idx_sd){
    if (idx_sd < 0 || idx_sd > this->w->get_n_simple_drones() - 1){
        throw std::invalid_argument("invalid burger index of " + 
        std::to_string(idx_sd) +". It needs to be >= 0 or < " + 
        std::to_string(this->w->get_simple_drones().size()));
    }
    // Check current time
    std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();

    // Measure time passed since burger last actuation time
    double elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(now - this->last_times_simple_drones[idx_sd]).count() * 1e-9;

    // If duration is bigger than the controller rate, then controller can act
    if(elapsed > this->w->get_simple_drone(idx_sd)->get_controller_period()){
        // Update last time
        this->last_times_simple_drones[idx_sd] = now;
        // Update last twist valuesact
        this->w->get_simple_drone(idx_sd)->set_new_vx_vy(vx,vy);
    }

    // Move drone
    this->w->get_simple_drone(idx_sd)->move(
        this->w->get_simple_drone(idx_sd)->get_last_vx(),
        this->w->get_simple_drone(idx_sd)->get_last_vy(),
        this->simulation_dt
    );

    // Update simple_drone lidar
    this->w->get_simple_drone(idx_sd)->get_lidar()->update_lidar_heavy(
        this->w->get_round_obstacles(), 
        this->w->get_simple_drones(),
        this->w->get_edges(),
        this->w->get_wall_obstacles(), 
        this->get_world()->get_simple_drone(idx_sd)->get_xc(), 
        this->get_world()->get_simple_drone(idx_sd)->get_yc(), 
        this->get_world()->get_simple_drone(idx_sd)->get_theta()
    );
}

std::vector<float> FastTurtle::get_robot_position(std::string robot_name)
{
    for(SimpleDrone sd : this->w->get_simple_drones())
    {
        if(sd.get_name() == robot_name)
        {
            return std::vector<float>{sd.x(), sd.y()};
        }
    }
    return {};
}

std::vector<float> FastTurtle::get_laser(std::string robot_name)
{
    for(SimpleDrone sd : this->w->get_simple_drones())
    {
        if(sd.get_name() == robot_name)
        {
            return sd.get_lidar()->get_lasers();
        }
    }
    return default_scan_;
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