#ifndef ROBOT_H
#define ROBOT_H

#include "geometry.h"
#include "obstacles.h"

#include <math.h>
#include <memory>
#include <iostream>
#include <tuple>

#define HIGHEST_NUMBER 1e10f
#define TO_DEG 57.29577f
#define TO_RAD 0.01745f
#define N_LASERS 360u
#define MIN_DISTANCE 0.12f
#define MAX_DISTANCE 3.5f
#define BURGER_RADIUS 0.09f
#define FOOD_RADIUS 0.1f
#define TIME_STEP 0.05f
#define DEFAULT_CONTROLLER_PERIOD 0.1f

class SimpleDrone; // in the future sd and tbb will extend Robot
class Lidar{
    private:
        float frequency, min_distance, max_distance;
        Point2d* position;
        std::vector<float> lasers;
    public:
        Lidar(float frequency, Point2d* position);
        std::vector<float> get_lasers();
        void display_lasers();
        void update_lidar_heavy(std::vector<RoundObstacle> round_obstacles, std::vector<std::shared_ptr<SimpleDrone>> simple_drones ,std::vector<LineSegment> edges, std::vector<WallObstacle> walls, float x_robot, float y_robot, float theta_robot);
        std::string tostring();
        bool in_between(float xi, float xm, float xf);
        std::tuple<float, float> get_nearest_points(float xr, float yr, float x1, float y1, float x2, float y2);
        std::tuple<float,float,float,float> get_laser_points(float, float, float, float);
        bool in_sight(float, float, float, float, float, float);
};

class TurtlebotBurger : public Circle{
    private:
        float theta, inv_diameter, diameter, controller_period;
        Lidar* lidar;
        std::string model;
        std::string name;
        double last_v;
        double last_w;
        bool visible;
    public:
        TurtlebotBurger(float x, float y, float theta, float radius, std::string name, float controller_period);
        std::string tostring();
        std::tuple<float, float, float> kinematics(float v, float w, double time_step);
        float x();
        float y();
        float get_theta();
        float get_controller_period();
        bool is_visible();
        void set_visibility(bool value);    
        double get_last_v();
        double get_last_w();
        void set_new_v_w(double v, double w);
        std::string get_name();
        std::string get_model();
        Lidar* get_lidar();
        void move(float v, float w, double time_step);
};

class SimpleDrone : public Circle{
    private:
        float theta, height, inv_diameter, diameter, controller_period;
        Lidar* lidar;
        std::string model;
        std::string name;
        double last_vx;
        double last_vy;
        bool visible;
    public:
        SimpleDrone(float x, float y, float height, float radius, std::string name, float controller_period);
        std::string tostring();
        std::tuple<float, float> kinematics(float vx, float vy, double time_step);
        float x();
        float y();
        float get_theta();
        float get_height();
        float get_controller_period();
        bool is_visible();
        void set_visibility(bool value);
        double get_last_vx();
        double get_last_vy();
        void set_new_vx_vy(double vx, double vy);
        std::string get_name();
        std::string get_model();
        Lidar* get_lidar();
        void move(float v, float w, double time_step);
        void reset();
};

#endif // ROBOT_H
