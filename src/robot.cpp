#include "robot.h"

TurtlebotBurger::TurtlebotBurger(float x, float y, float theta, float radius, std::string name, float controller_period) : Circle(x, y, radius){
    // Attributes
    this->controller_period = controller_period;
    this->theta = theta;
    this->inv_diameter = 1.0 / (2 * this->radius);
    this->diameter = this->radius * 2.0;
    this->name = name;
    this->model = "burger";
    this->last_v = 0.0;
    this->last_w = 0.0;

    // Lidar
    float frequency;
    this->lidar = new Lidar(frequency, new Point2d(x,y));
}

float TurtlebotBurger::x(){
    return this->xc;
}

float TurtlebotBurger::y(){
    return this->yc;
}

float TurtlebotBurger::get_theta(){
    return this->theta;
}

float TurtlebotBurger::get_controller_period(){
    return this->controller_period;
}

std::string TurtlebotBurger::tostring(){
    return "(TurtlebotBurger) Name: " + this->get_name() + " , Model: " + this->get_model() + " , " +
    Circle::tostring() + ", theta: " 
    + std::to_string(this->theta)
    + ", with " + this->lidar->tostring() + "\n";
}

std::tuple<float, float, float> TurtlebotBurger::kinematics(float v, float w, double time_step){
    float v_left = v + w * this->radius;
    float v_right = v - w * this->radius;
    float dd = (v_left + v_right) * 0.5;
    float dth = (v_left - v_right) * this->inv_diameter;
    
    return 
    {
        this->xc + dd * cos(this->theta + dth * 0.5) * time_step,
        this->yc + dd * sin(this->theta + dth * 0.5) * time_step,
        normalize_angle(this->theta + dth * TIME_STEP)
    };
}

void TurtlebotBurger::move(float v, float w, double time_step){
    // Compute position
    std::tuple<float,float,float> new_pose = this->kinematics(v, w, time_step);
    // Update position
    this->xc = std::get<0>(new_pose);
    this->yc = std::get<1>(new_pose);
    this->theta = std::get<2>(new_pose);
    return;
}

Lidar* TurtlebotBurger::get_lidar(){
    return this->lidar;
}

std::vector<float> Lidar::get_lasers(){
    return this->lasers;
}

std::string TurtlebotBurger::get_name(){
    return this->name;
}

std::string TurtlebotBurger::get_model(){
    return this->model;
}

double TurtlebotBurger::get_last_w(){
    return this->last_w;
}

double TurtlebotBurger::get_last_v(){
    return this->last_v;
}

void TurtlebotBurger::set_new_v_w(double v, double w){
    this->last_v = v;
    this->last_w = w;
}

void Lidar::update_lidar_heavy(std::vector<RoundObstacle> round_obstacles, std::vector<Line> edges, float x_robot, float y_robot, float theta_robot){
    std::fill(this->lasers.begin(), this->lasers.end(), MAX_DISTANCE);
    Line laser(0,0,0,0);
    bool in_sight;
    std::tuple<bool, float, float, float, float> intersection_obstacle;
    std::tuple<bool, float, float> intersection_line;
    float min_distance;
    float distance;

    // Go through all rays
    for (int ray=0; ray < this->lasers.size(); ray++)
    {
        std::tuple<float, float, float, float> laser_points = this->get_laser_points(ray, x_robot, y_robot, theta_robot);
        laser.set_points(std::get<0>(laser_points), std::get<1>(laser_points), std::get<2>(laser_points), std::get<3>(laser_points));
        // Go through all obstacles
        for(int o = 0; o < round_obstacles.size(); o++)
        {
            // Check intersections
            intersection_obstacle = round_obstacles[o].intersects_line(laser);
            // If there was intersection
            if (std::get<0>(intersection_obstacle))
            {
                // Choose right pair of points. The intersection function returns two possible pairs.
                std::tuple<float, float> obstacle_points = this->get_nearest_points(
                    x_robot,
                    y_robot,
                    std::get<1>(intersection_obstacle),
                    std::get<2>(intersection_obstacle),
                    std::get<3>(intersection_obstacle),
                    std::get<4>(intersection_obstacle)
                );
                // Check if it is in sight 
                in_sight = this->in_sight(
                    std::get<0>(laser_points),
                    std::get<1>(laser_points),
                    std::get<2>(laser_points),
                    std::get<3>(laser_points),
                    std::get<0>(obstacle_points),
                    std::get<1>(obstacle_points)
                );
                //If it is in sight measure distance
                if (in_sight)
                {
                    distance = distance_between_points(
                        std::get<0>(laser_points),
                        std::get<1>(laser_points),
                        std::get<0>(obstacle_points),
                        std::get<1>(obstacle_points)
                    ) + BURGER_RADIUS;
                    this->lasers[ray] = std::min(distance, this->lasers[ray]);
                }

            }
        }
        // Go through 
        for(int e = 0; e < edges.size(); e++)
        {
            intersection_line = edges[e].intersects_line(laser);
            // If intersects with world edge
            if (std::get<0>(intersection_line))
            {
                in_sight = this->in_sight(
                    std::get<0>(laser_points),
                    std::get<1>(laser_points),
                    std::get<2>(laser_points),
                    std::get<3>(laser_points),
                    std::get<1>(intersection_line),
                    std::get<2>(intersection_line)
                );
                if(in_sight)
                {
                    distance = distance_between_points(
                    std::get<0>(laser_points),
                    std::get<1>(laser_points),
                    std::get<1>(intersection_line),
                    std::get<2>(intersection_line)
                    ) + BURGER_RADIUS;
                    this->lasers[ray] = std::min(distance, this->lasers[ray]);
                }
            }
            
        }
    }
}

// Lidar
Lidar::Lidar(float frequency, Point2d* position){
    this->frequency = frequency;
    this->position = position;
    this->lasers.assign(N_LASERS, MAX_DISTANCE);
    this->min_distance = MIN_DISTANCE;
    this->max_distance = MAX_DISTANCE;
}

std::string Lidar::tostring(){
    return "(Lidar) frequency: " + std::to_string(this->frequency)
    + ", position: " + this->position->tostring() 
    + ", min_distance: " + std::to_string(this->min_distance)
    + ", max_distance: " + std::to_string(this->max_distance)
    +  "\n";
}

void Lidar::display_lasers(){
    std::cout << "Lasers: [ ";
    for(int i = 0; i < this->lasers.size(); i++){
        std::cout << this->lasers[i] << " ";
    }
    std::cout << "]" << std::endl;
}

bool Lidar::in_between(float xi, float xm, float xf){
    return (xi <= xm && xm <= xf) || (xf <= xm && xm <= xi);
}
/*
template <typename T> bool Lidar::in_sight(float x_sight, float y_sight, float x_forward, float y_forward, float x_object, float y_object, T object){
    return true;
}
*/
bool Lidar::in_sight(float x_min, float y_min, float x_max, float y_max, float x_obs, float y_obs){
    // Check after if it is needed to add the last condition
    return this->in_between(x_min, x_obs, x_max) && this->in_between(y_min, y_obs, y_max);
}

// intersection is two pair of points. the right pair is the nearest to the robot.
std::tuple<float, float> Lidar::get_nearest_points(float xr, float yr, float x1, float y1, float x2, float y2){
    float distance = HIGHEST_NUMBER;
    if (distance_between_points(xr,yr,x1,y1) < distance_between_points(xr,yr,x2,y2)) return {x1,y1};
    else return {x2,y2};
}


std::tuple<float,float,float,float> Lidar::get_laser_points(float angle, float x, float y, float theta){
    float angle_rad = angle * TO_RAD;
    float cos_ = cos(angle_rad + theta);
    float sin_ = sin(angle_rad + theta);
    return 
    {
        x + this->min_distance * cos_,
        y + this->min_distance * sin_,
        x + this->max_distance * cos_,
        y + this->max_distance * sin_
    };
}

