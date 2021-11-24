#include "robot.h"

TurtlebotBurger::TurtlebotBurger(double x, double y, double theta, double radius, std::string name, double controller_period) : Circle(x, y, radius){
    // Attributes
    this->controller_period = controller_period;
    this->theta = theta;
    this->inv_diameter = 1.0 / (2 * this->radius);
    this->diameter = this->radius * 2.0;
    this->name = name;
    this->model = "burger";
    this->last_v = 0.0;
    this->last_w = 0.0;
    this->visible = true;

    // Lidar
    double frequency;
    this->lidar = new Lidar(frequency, new Point2d(x,y));
}

double TurtlebotBurger::x(){
    return this->xc;
}

double TurtlebotBurger::y(){
    return this->yc;
}

double TurtlebotBurger::get_theta(){
    return this->theta;
}

double TurtlebotBurger::get_controller_period(){
    return this->controller_period;
}

bool TurtlebotBurger::is_visible(){
    return this->visible;
}

void TurtlebotBurger::set_visibility(bool value){
    this->visible = value;
}

std::string TurtlebotBurger::tostring(){
    return "(TurtlebotBurger) Name: " + this->get_name() + " , Model: " + this->get_model() + " , " +
    Circle::tostring() + ", theta: " 
    + std::to_string(this->theta)
    + ", with " + this->lidar->tostring() + "\n";
}

std::tuple<double, double, double> TurtlebotBurger::kinematics(double v, double w, double time_step){
    double v_left = v + w * this->radius;
    double v_right = v - w * this->radius;
    double dd = (v_left + v_right) * 0.5;
    double dth = (v_left - v_right) * this->inv_diameter;
    
    return 
    {
        this->xc + dd * cos(this->theta + dth * 0.5) * time_step,
        this->yc + dd * sin(this->theta + dth * 0.5) * time_step,
        normalize_angle(this->theta + dth * TIME_STEP)
    };
}

void TurtlebotBurger::move(double v, double w, double time_step){
    // Compute position
    std::tuple<double,double,double> new_pose = this->kinematics(v, w, time_step);
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

SimpleDrone::SimpleDrone(double x, double y, double height, double radius, std::string name, double controller_period) : Circle(x, y, radius)
{
    // Attributes
    this->controller_period = controller_period;
    this->inv_diameter = 1.0 / (2 * this->radius);
    this->diameter = this->radius * 2.0;
    this->height = height;
    this->name = name;
    this->model = "simple";
    this->last_vx = 0.0;
    this->last_vy = 0.0;
    this->visible = true;
    this->theta = 0.0;

    // Lidar
    double frequency;
    this->lidar = new Lidar(frequency, new Point2d(x,y));
}

double SimpleDrone::x(){
    return this->xc;
}

double SimpleDrone::y(){
    return this->yc;
}

double SimpleDrone::get_theta(){
    return this->theta;
}

double SimpleDrone::get_height(){
    return this->height;
}

double SimpleDrone::get_controller_period(){
    return this->controller_period;
}

bool SimpleDrone::is_visible(){
    return this->visible;
}

void SimpleDrone::set_visibility(bool value){
    this->visible = value;
}


std::string SimpleDrone::tostring(){
    return "(SimpleDrone) Name: " + this->get_name() + " , Model: " + this->get_model() + " , " +
    Circle::tostring() + ", height: " 
    + std::to_string(this->height)
    + ", with " + this->lidar->tostring() + "\n";
}

std::tuple<double, double> SimpleDrone::kinematics(double vx, double vy, double time_step){    
    return 
    {
        this->xc + vx * time_step,
        this->yc + vy * time_step
    };
}

void SimpleDrone::move(double vx, double vy, double time_step){
    // Compute position
    std::tuple<double,double> new_pose = this->kinematics(vx, vy, time_step);
    // Update position
    this->xc = std::get<0>(new_pose);
    this->yc = std::get<1>(new_pose);
    return;
}

void SimpleDrone::reset()
{
    this->last_vx = 0.0;
    this->last_vy = 0.0;
    this->visible = true;
}

Lidar* SimpleDrone::get_lidar(){
    return this->lidar;
}

std::string SimpleDrone::get_name(){
    return this->name;
}

std::string SimpleDrone::get_model(){
    return this->model;
}

double SimpleDrone::get_last_vx(){
    return this->last_vx;
}

double SimpleDrone::get_last_vy(){
    return this->last_vy;
}

void SimpleDrone::set_new_vx_vy(double vx, double vy){
    this->last_vx = vx;
    this->last_vy = vy;
}

void Lidar::update_lidar_heavy(std::vector<RoundObstacle> round_obstacles, std::vector<std::shared_ptr<SimpleDrone>> simple_drones,std::vector<LineSegment> edges, std::vector<WallObstacle> walls, double x_robot, double y_robot, double theta_robot){
    std::fill(this->lasers.begin(), this->lasers.end(), MAX_DISTANCE);
    LineSegment laser(0,0,0,0);
    bool in_sight;
    std::tuple<bool, double, double, double, double> intersection_obstacle;
    std::tuple<bool, double, double> intersection_line;
    float min_distance;
    float distance;

    // Go through all rays
    for (int ray=0; ray < this->lasers.size(); ray++)
    {
        std::tuple<double, double, double, double> laser_points = this->get_laser_points(ray, x_robot, y_robot, theta_robot);
        laser.set_points(std::get<0>(laser_points), std::get<1>(laser_points), std::get<2>(laser_points), std::get<3>(laser_points));
        // Obstacles scanning
        for(int o = 0; o < round_obstacles.size(); o++)
        {
            // Check intersections
            intersection_obstacle = round_obstacles[o].intersects_line(laser);
            // If there was intersection
            if (std::get<0>(intersection_obstacle))
            {
                // Choose right pair of points. The intersection function returns two possible pairs.
                std::tuple<double, double> obstacle_points = this->get_nearest_points(
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
        // Other robots scanning. For now only scans drones
        for(int r = 0; r < simple_drones.size(); r++)
        {
            // Check intersections
            intersection_obstacle = simple_drones[r]->intersects_line(laser);
            // If there was intersection
            if (std::get<0>(intersection_obstacle))
            {
                // Choose right pair of points. The intersection function returns two possible pairs.
                std::tuple<double, double> obstacle_points = this->get_nearest_points(
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
        // World edges scanning
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
        // Wall obstacles scanning
        for(int w = 0; w < walls.size(); w++)
        {
            intersection_line = walls[w].intersects_line(laser);
            // If intersects with world walls
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
        this->lasers[ray] = (float)std::max(this->lasers[ray], MIN_DISTANCE);
    }
}

// Lidar
Lidar::Lidar(double frequency, Point2d* position){
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

bool Lidar::in_between(double xi, double xm, double xf){
    return (xi <= xm && xm <= xf) || (xf <= xm && xm <= xi);
}

bool Lidar::in_sight(double x_min, double y_min, double x_max, double y_max, double x_obs, double y_obs){
    // Check after if it is needed to add the last condition
    return this->in_between(x_min, x_obs, x_max) && this->in_between(y_min, y_obs, y_max);
}

// intersection is two pair of points. the right pair is the nearest to the robot.
std::tuple<double, double> Lidar::get_nearest_points(double xr, double yr, double x1, double y1, double x2, double y2){
    double distance = HIGHEST_NUMBER;
    if (distance_between_points(xr,yr,x1,y1) < distance_between_points(xr,yr,x2,y2)) return {x1,y1};
    else return {x2,y2};
}


std::tuple<double,double,double,double> Lidar::get_laser_points(double angle, double x, double y, double theta){
    double angle_rad = angle * TO_RAD;
    double cos_ = cos(angle_rad + theta);
    double sin_ = sin(angle_rad + theta);
    return 
    {
        x + this->min_distance * cos_,
        y + this->min_distance * sin_,
        x + this->max_distance * cos_,
        y + this->max_distance * sin_
    };
}

