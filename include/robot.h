#include "geometry.h"
#include "obstacles.h"
#include <math.h>
#include <iostream>
#include <tuple>

#define HIGHEST_NUMBER 1e10
#define TO_DEG 57.29577
#define TO_RAD 0.01745
#define N_LASERS 360
#define MIN_DISTANCE 0.12
#define MAX_DISTANCE 3.5
#define BURGER_RADIUS 0.09

#ifndef LIDAR_H
#define LIDAR_H
class Lidar{
    private:
        float frequency, min_distance, max_distance;
        Point2d* position;
        std::vector<float> lasers;
    public:
        Lidar(float frequency, Point2d* position);
        std::vector<float> get_lasers();
        void display_lasers();
        void update_lidar_heavy(std::vector<RoundObstacle> round_obstacles, std::vector<Line> edges, float x_robot, float y_robot, float theta_robot);
        std::string tostring();
        bool in_between(float xi, float xm, float xf);
        //template <typename T> bool in_sight(float x_sight, float y_sight, float x_forward, float y_forward, float x_object, float y_object, T object);
        std::tuple<float, float> get_nearest_points(float xr, float yr, float x1, float y1, float x2, float y2);
        std::tuple<float,float,float,float> get_laser_points(float, float, float, float);
        bool in_sight(float, float, float, float, float, float);
};
#endif

#ifndef TURTLEBOTBURGER_H
#define TURTLEBOTBURGER_H
class TurtlebotBurger : public Circle{
    private:
        float dt, theta, inv_diameter, diameter;
        Lidar* lidar;
        std::string model;
        std::string name;
    public:
        TurtlebotBurger(float x, float y, float theta, float radius, float dt, std::string name);
        void set_pose(float x, float y, float theta);
        std::string tostring();
        std::tuple<float, float, float> kinematics(float v, float w);
        float x();
        float y();
        float get_dt();
        float get_theta();
        std::string get_name();
        std::string get_model();
        Lidar* get_lidar();
        void move(float v, float w);
};
#endif
