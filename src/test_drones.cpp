// Simulator library
#include "fast_turtle.h"
// Swarm Robotics
#include "swarm_robotics.h"
// ROS main library
#include <ros/ros.h>
// TF
#include <tf/transform_broadcaster.h>
// Messages
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "sensor_msgs/LaserScan.h"
#include "fast_turtle/RobotData.h"
#include "fast_turtle/RobotDataArray.h"

// Simulation frames per second
#define SIMULATION_FPS 40u

// Values for food distances
#define FOOD_LIMIT_X 0.5
#define FOOD_LIMIT_Y 0.5
#define PROXIMITY 0.3

// Drone structure parameters
#define SIMPLE_DRONE_SUPPORTS_THICKNESS 0.01f

// Initialize fast turtle simulator object
FastTurtle *ft = new FastTurtle(SIMULATION_FPS);
SwarmCompetition* sc = new SwarmCompetition(SIMULATION_FPS);

// Markers
visualization_msgs::Marker world_marker;
visualization_msgs::MarkerArray obstacle_markers;
visualization_msgs::MarkerArray wall_markers;
visualization_msgs::MarkerArray tb_burger_markers;
visualization_msgs::MarkerArray tb_burger_orientation_markers;
visualization_msgs::MarkerArray simple_drone_markers;
visualization_msgs::MarkerArray simple_drone_supports_markers;
visualization_msgs::MarkerArray food_markers;

// Marker Publishers
ros::Publisher world_marker_publisher;
ros::Publisher obstacle_markers_publisher;
ros::Publisher wall_markers_publisher;
ros::Publisher tb_burger_markers_publisher;
ros::Publisher tb_burger_orientation_markers_publisher;
ros::Publisher simple_drone_markers_publisher;
ros::Publisher simple_drone_supports_markers_publisher;
ros::Publisher laser_scan_publisher;

// Publishers
ros::Publisher tb_burgers_publisher;
ros::Publisher simple_drones_publisher;

// Messages
sensor_msgs::LaserScan laser_scan_msg;

// Vector of real time command velocities for all tb_burgers
std::vector<cmd_vel_tbb> cmd_vels_tb_burgers{{0, 0}, {0, 0}, {0, 0}, {0, 0}};
std::vector<cmd_vel_sd> cmd_vels_simple_drones{{0, 0}, {0, 0}, {0, 0}, {0, 0}};

void move_tb_burgers(int idx)
{
    if (cmd_vels_tb_burgers[idx].v > MAX_LIN_VELOCITY_TB_BURGER)
    {
        cmd_vels_tb_burgers[idx].v = MAX_LIN_VELOCITY_TB_BURGER;
    }
    else if(cmd_vels_tb_burgers[idx].v < - MAX_LIN_VELOCITY_TB_BURGER)
    {
        cmd_vels_tb_burgers[idx].v = -MAX_LIN_VELOCITY_TB_BURGER;
    }
    if (cmd_vels_tb_burgers[idx].w > MAX_ANG_VELOCITY_TB_BURGER)
    {
        cmd_vels_tb_burgers[idx].w = MAX_ANG_VELOCITY_TB_BURGER;
    }
    else if(cmd_vels_tb_burgers[idx].w < -MAX_ANG_VELOCITY_TB_BURGER)
    {
        cmd_vels_tb_burgers[idx].w = -MAX_ANG_VELOCITY_TB_BURGER;
    }
    ft->act_turtlebot_burger(cmd_vels_tb_burgers[idx].v, cmd_vels_tb_burgers[idx].w, idx);
}

void move_simple_drones(int idx)
{
    if (cmd_vels_simple_drones[idx].vx > MAX_LIN_VELOCITY_SIMPLE_DRONE)
    {
        cmd_vels_simple_drones[idx].vx = MAX_LIN_VELOCITY_SIMPLE_DRONE;
    }
    else if (cmd_vels_simple_drones[idx].vx < -MAX_LIN_VELOCITY_SIMPLE_DRONE) 
    {
        cmd_vels_simple_drones[idx].vx = -MAX_LIN_VELOCITY_SIMPLE_DRONE;
    }


    if (cmd_vels_simple_drones[idx].vy > MAX_LIN_VELOCITY_SIMPLE_DRONE) 
    {
        cmd_vels_simple_drones[idx].vy = MAX_LIN_VELOCITY_SIMPLE_DRONE;
    }
    else if (cmd_vels_simple_drones[idx].vy < -MAX_LIN_VELOCITY_SIMPLE_DRONE) 
    {
        cmd_vels_simple_drones[idx].vy = -MAX_LIN_VELOCITY_SIMPLE_DRONE;
    }

    ft->act_simple_drone(cmd_vels_simple_drones[idx].vx, cmd_vels_simple_drones[idx].vy, idx);
}

void update_physics()
{
    for(int idx = 0; idx < ft->get_world()->get_n_burgers(); idx++) move_tb_burgers(idx);
    for(int idx = 0; idx < ft->get_world()->get_n_simple_drones(); idx++) move_simple_drones(idx);
}

void listen_cmd_vel_tbb0(const geometry_msgs::Twist& msg)
{
    if (ft->get_world()->get_n_burgers() > 0)
    {
        ROS_INFO("Received commands v: %f and w: %f", msg.linear.x, msg.angular.z);
        cmd_vels_tb_burgers[0].v = msg.linear.x;
        cmd_vels_tb_burgers[0].w = msg.angular.z;
        std::cout << "[Turtlebot Burger 0 pose]: " << ft->get_world()->get_burger(0)->tostring() << "\n";
    }
}

void listen_cmd_vel_tbb1(const geometry_msgs::Twist& msg)
{
    if (ft->get_world()->get_n_burgers() > 1)
    {
        ROS_INFO("Received commands v: %f and w: %f", msg.linear.x, msg.angular.z);
        cmd_vels_tb_burgers[1].v = msg.linear.x;
        cmd_vels_tb_burgers[1].w = msg.angular.z;
        std::cout << "[Turtlebot Burger 1 pose]: " << ft->get_world()->get_burger(1)->tostring() << "\n";
    }
}

void listen_cmd_vel_tbb2(const geometry_msgs::Twist& msg)
{
    if (ft->get_world()->get_n_burgers() > 2)
    {
        ROS_INFO("Received commands v: %f and w: %f", msg.linear.x, msg.angular.z);
        cmd_vels_tb_burgers[2].v = msg.linear.x;
        cmd_vels_tb_burgers[2].w = msg.angular.z;
        std::cout << "[Turtlebot Burger 2 pose]: " << ft->get_world()->get_burger(2)->tostring() << "\n";
    }
}

void listen_cmd_vel_tbb3(const geometry_msgs::Twist& msg)
{
    if (ft->get_world()->get_n_burgers() > 3)
    {
        ROS_INFO("Received commands v: %f and w: %f", msg.linear.x, msg.angular.z);
        cmd_vels_tb_burgers[3].v = msg.linear.x;
        cmd_vels_tb_burgers[3].w = msg.angular.z;
        std::cout << "[Turtlebot Burger 3 pose]: " << ft->get_world()->get_burger(3)->tostring() << "\n";
    }
}

void listen_cmd_vel_sd0(const geometry_msgs::Twist& msg)
{
    if (ft->get_world()->get_n_simple_drones() > 0)
    {
        ROS_INFO("Received commands vx: %f and vy: %f", msg.linear.x, msg.linear.y);
        cmd_vels_simple_drones[0].vx = msg.linear.x;
        cmd_vels_simple_drones[0].vy = msg.linear.y;
        std::cout << "[Simple Drone 0 pose]: " << ft->get_world()->get_simple_drone(0)->tostring() << "\n";
    }
}

void init_graphics_and_data()
{
    // Counter markers
    int j = 0;
    // For int variable
    int i = 0;

    // World marker
    world_marker.header.frame_id = "world";
    world_marker.ns = "simulator_markers";
    world_marker.id = 0;
    world_marker.type = visualization_msgs::Marker::CUBE;
    world_marker.action = visualization_msgs::Marker::ADD;
    world_marker.pose.position.x = 0;
    world_marker.pose.position.y = 0;
    world_marker.pose.position.z = 0;
    world_marker.pose.orientation.x = 0.0;
    world_marker.pose.orientation.y = 0.0;
    world_marker.pose.orientation.z = 0.0;
    world_marker.pose.orientation.w = 1.0;
    world_marker.scale.x = ft->get_world()->get_length();
    world_marker.scale.y = ft->get_world()->get_length();
    world_marker.scale.z = 0.0001;
    world_marker.color.a = 0.5;
    world_marker.color.r = 1.0;
    world_marker.color.g = 0.0;
    world_marker.color.b = 0.0;
    j += 1;

    // Turtlebot Burgers position and orientation
    visualization_msgs::Marker tb_burger_marker;
    geometry_msgs::Point arrow_origin, arrow_end;
    geometry_msgs::Quaternion q;
    visualization_msgs::Marker tb_burger_orientation_marker;
    for (i = 0; i < ft->get_world()->get_n_burgers(); i++)
    {
        tb_burger_marker.header.frame_id = "world";
        tb_burger_marker.ns = "simulation_markers";
        tb_burger_marker.id = j;
        tb_burger_marker.type = visualization_msgs::Marker::CYLINDER;
        tb_burger_marker.action = visualization_msgs::Marker::ADD;
        tb_burger_marker.scale.x = ft->get_world()->get_burger(i)->get_diameter();
        tb_burger_marker.scale.y = ft->get_world()->get_burger(i)->get_diameter();
        tb_burger_marker.scale.z = 0.192;
        tb_burger_marker.color.a = 1.0;
        tb_burger_marker.color.r = 0.0;
        tb_burger_marker.color.g = 0.0;
        tb_burger_marker.color.b = 1.0;
        tb_burger_marker.pose.position.x = ft->get_world()->get_burger(i)->get_xc();
        tb_burger_marker.pose.position.y = ft->get_world()->get_burger(i)->get_yc();
        tb_burger_marker.pose.position.z = tb_burger_marker.scale.z * 0.5;
        tb_burger_marker.pose.orientation.x = 0.0;
        tb_burger_marker.pose.orientation.y = 0.0;
        tb_burger_marker.pose.orientation.z = 0.0;
        tb_burger_marker.pose.orientation.w = 1.0;

        tb_burger_markers.markers.push_back(tb_burger_marker);
        j+=1;

        tb_burger_orientation_marker.header.frame_id = "tb_burger_" 
        + ft->get_world()->get_burger(i)->get_model() + "_" 
        + ft->get_world()->get_burger(i)->get_name();
        tb_burger_orientation_marker.ns = "simulation_markers";
        tb_burger_orientation_marker.id = j;
        tb_burger_orientation_marker.type = visualization_msgs::Marker::ARROW;
        tb_burger_orientation_marker.action = visualization_msgs::Marker::ADD;
        tb_burger_orientation_marker.pose.orientation.x = 0.0;
        tb_burger_orientation_marker.pose.orientation.y = 0.0;
        tb_burger_orientation_marker.pose.orientation.z = 0.0;
        tb_burger_orientation_marker.pose.orientation.w = 1.0;
        tb_burger_orientation_marker.scale.x = 0.02;
        tb_burger_orientation_marker.scale.y = 0.02;
        tb_burger_orientation_marker.scale.z = 0.02;
        tb_burger_orientation_marker.color.a = 1.0;
        tb_burger_orientation_marker.color.r = 0.0;
        tb_burger_orientation_marker.color.g = 0.0;
        tb_burger_orientation_marker.color.b = 0.0;
        arrow_origin.x = 0.0;
        arrow_origin.y = 0.0;
        arrow_origin.z = tb_burger_marker.scale.z;
        arrow_end.x = arrow_origin.x + 0.3;
        arrow_end.y = arrow_origin.y;
        arrow_end.z = arrow_origin.z;
        tb_burger_orientation_marker.points.clear();
        tb_burger_orientation_marker.points.push_back(arrow_origin);
        tb_burger_orientation_marker.points.push_back(arrow_end);
        tb_burger_orientation_markers.markers.push_back(tb_burger_orientation_marker);
        j+=1;   
    }

    // Simple drones position
    visualization_msgs::Marker simple_drone_marker;
    for (i = 0; i < ft->get_world()->get_n_simple_drones(); i++)
    {
        simple_drone_marker.header.frame_id = "world";
        simple_drone_marker.ns = "simulation_markers";
        simple_drone_marker.id = j;
        simple_drone_marker.type = visualization_msgs::Marker::CYLINDER;
        simple_drone_marker.action = visualization_msgs::Marker::ADD;
        simple_drone_marker.scale.x = ft->get_world()->get_simple_drone(i)->get_diameter();
        simple_drone_marker.scale.y = ft->get_world()->get_simple_drone(i)->get_diameter();
        simple_drone_marker.scale.z = 0.05;
        simple_drone_marker.pose.position.x = ft->get_world()->get_simple_drone(i)->get_xc();
        simple_drone_marker.pose.position.y = ft->get_world()->get_simple_drone(i)->get_yc();
        simple_drone_marker.pose.position.z = ft->get_world()->get_simple_drone(i)->get_height() + simple_drone_marker.scale.z * 0.5;
        simple_drone_marker.pose.orientation.x = 0.0;
        simple_drone_marker.pose.orientation.y = 0.0;
        simple_drone_marker.pose.orientation.z = 0.0;
        simple_drone_marker.pose.orientation.w = 1.0;

        simple_drone_marker.color.a = 1.0;
        simple_drone_marker.color.r = 0.0;
        simple_drone_marker.color.g = 0.0;
        simple_drone_marker.color.b = 0.0;
        simple_drone_markers.markers.push_back(simple_drone_marker);
        j+=1;

        visualization_msgs::Marker sd_wing_marker;
        visualization_msgs::Marker sd_support_marker;
        geometry_msgs::Point p;
        // Four supports for wings
        for (int sw = 0; sw < 4; sw++)
        {
            sd_support_marker.header.frame_id = "simple_drone_" + ft->get_world()->get_simple_drone(i)->get_model() + "_" + ft->get_world()->get_simple_drone(i)->get_name();
            sd_support_marker.ns = "simulation_markers";
            sd_support_marker.id = j;
            sd_support_marker.type = visualization_msgs::Marker::LINE_LIST;
            sd_support_marker.action = visualization_msgs::Marker::ADD;
            sd_support_marker.scale.x = SIMPLE_DRONE_SUPPORTS_THICKNESS;
            sd_support_marker.color.a = 1.0;
            sd_support_marker.color.r = 0.0;
            sd_support_marker.color.g = 0.0;
            sd_support_marker.color.b = 0.0;
            sd_support_marker.pose.orientation.x = 0.0;
            sd_support_marker.pose.orientation.y = 0.0;
            sd_support_marker.pose.orientation.z = 0.0;
            sd_support_marker.pose.orientation.w = 1.0;
            p.x = 0.0;
            p.y = 0.0;
            p.z = ft->get_world()->get_simple_drone(i)->get_height();
            sd_support_marker.points.push_back(p);
            p.x = p.x + ft->get_world()->get_simple_drone(i)->get_diameter() * cos(M_PI_4 + sw * M_PI_2);
            p.y = p.y + ft->get_world()->get_simple_drone(i)->get_diameter() * sin(M_PI_4 + sw * M_PI_2);
            sd_support_marker.points.push_back(p);
            simple_drone_supports_markers.markers.push_back(sd_support_marker);
            j+=1;
        }   
    }

    // Cylinder obstacles
    visualization_msgs::Marker obstacle_marker;
    for (i = 0; i < ft->get_world()->get_round_obstacles().size(); i++)
    {
        obstacle_marker.header.frame_id = "world";
        obstacle_marker.ns = "simulation_markers";
        obstacle_marker.id = j;
        obstacle_marker.type = visualization_msgs::Marker::CYLINDER;
        obstacle_marker.action = visualization_msgs::Marker::ADD;
        obstacle_marker.pose.orientation.x = 0.0;
        obstacle_marker.pose.orientation.y = 0.0;
        obstacle_marker.pose.orientation.z = 0.0;
        obstacle_marker.pose.orientation.w = 1.0;
        obstacle_marker.scale.x = ft->get_world()->get_round_obstacle(i)->get_diameter();
        obstacle_marker.scale.y = ft->get_world()->get_round_obstacle(i)->get_diameter();
        obstacle_marker.scale.z = 0.192;
        obstacle_marker.pose.position.x = ft->get_world()->get_round_obstacle(i)->get_xc();
        obstacle_marker.pose.position.y = ft->get_world()->get_round_obstacle(i)->get_yc();
        obstacle_marker.pose.position.z = obstacle_marker.scale.z * 0.5;
        obstacle_marker.color.a = 1.0;
        obstacle_marker.color.r = 1.0;
        obstacle_marker.color.g = 1.0;
        obstacle_marker.color.b = 1.0;
        obstacle_markers.markers.push_back(obstacle_marker);
        j += 1;
    }

    // Walls
    visualization_msgs::Marker wall_marker;
    for(i = 0; i < ft->get_world()->get_wall_obstacles().size(); i++){
        wall_marker.header.frame_id = "world";
        wall_marker.ns = "simulation_markers";
        wall_marker.id = j;
        wall_marker.type = visualization_msgs::Marker::CUBE;
        wall_marker.action = visualization_msgs::Marker::ADD;
        wall_marker.pose.position.x = std::get<0>(ft->get_world()->get_wall_obstacle(i)->get_midpoint());
        wall_marker.pose.position.y = std::get<1>(ft->get_world()->get_wall_obstacle(i)->get_midpoint());
        wall_marker.pose.position.z = 1.0;
        wall_marker.pose.orientation.x = 0.0;
        wall_marker.pose.orientation.y = 0.0;
        //wall_marker.pose.orientation.z = ft->get_world()->get_wall_obstacle(i)->is_vertical() ? 1.0 : 0.0;//1.0; //1 - para estar assim | ; 0 - para esta assim _ ;
        wall_marker.pose.orientation.z = 0.0;
        wall_marker.pose.orientation.w = 1.0;
        wall_marker.scale.x = ft->get_world()->get_wall_obstacle(i)->get_length();
        wall_marker.scale.y = 0.01;
        wall_marker.scale.z = 2.0;
        wall_marker.color.a = 1.0;
        wall_marker.color.r = 0.7;
        wall_marker.color.g = 0.7;
        wall_marker.color.b = 0.7;
        wall_markers.markers.push_back(wall_marker);
        j+=1;
    }

    laser_scan_msg.angle_min = 0;
    laser_scan_msg.angle_max = M_PI * 2;
    laser_scan_msg.angle_increment = 2.0 * M_PI / 360;
    laser_scan_msg.range_min = MIN_DISTANCE;
    laser_scan_msg.range_max = MAX_DISTANCE + 1e-4;
    laser_scan_msg.time_increment = 0;
    laser_scan_msg.scan_time = 0;

    world_marker_publisher.publish(world_marker);
    tb_burger_markers_publisher.publish(tb_burger_markers);
    tb_burger_orientation_markers_publisher.publish(tb_burger_orientation_markers);
    obstacle_markers_publisher.publish(obstacle_markers);
    wall_markers_publisher.publish(wall_markers);
    simple_drone_supports_markers_publisher.publish(simple_drone_supports_markers);
}

void repaint()
{
    // World marker
    world_marker_publisher.publish(world_marker);

    // Turtlebot Burgers position markers
    for (int i = 0; i < ft->get_world()->get_n_burgers(); i++)
    {
        tb_burger_markers.markers[i].pose.position.x = ft->get_world()->get_burger(i)->x();
        tb_burger_markers.markers[i].pose.position.y = ft->get_world()->get_burger(i)->y();
    }

    // Simple drone position markers
    for (int i = 0; i < ft->get_world()->get_n_simple_drones(); i++)
    {
        simple_drone_markers.markers[i].pose.position.x = ft->get_world()->get_simple_drone(i)->x();
        simple_drone_markers.markers[i].pose.position.y = ft->get_world()->get_simple_drone(i)->y();
    }

    // Laser scan update
    laser_scan_msg.ranges = ft->get_world()->get_simple_drone(0)->get_lidar()->get_lasers();
    laser_scan_msg.header.frame_id = "simple_drone_" + ft->get_world()->get_simple_drone(0)->get_model() + "_" + ft->get_world()->get_simple_drone(0)->get_name();
    
    // Turtlebot Burgers position markers
    tb_burger_markers_publisher.publish(tb_burger_markers);

    // Turtlebot Burgers orientation markers
    tb_burger_orientation_markers_publisher.publish(tb_burger_orientation_markers);

    // Simple drone markers
    simple_drone_markers_publisher.publish(simple_drone_markers);

    // Simple drone wings markers
    simple_drone_supports_markers_publisher.publish(simple_drone_supports_markers);

    // Obstacles
    obstacle_markers_publisher.publish(obstacle_markers);

    // Walls
    wall_markers_publisher.publish(wall_markers);

    // Laser scan
    laser_scan_publisher.publish(laser_scan_msg);
}

void publish_data()
{
    tf::Transform transform;
    tf::Quaternion q;
    static tf::TransformBroadcaster tf_br_tb_burger;
    static tf::TransformBroadcaster tf_br_simple_drone;
    static tf::TransformBroadcaster tf_br_map;

    // World
    transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
    q.setRPY(0, 0, 0);
    transform.setRotation(q);
    tf_br_map.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "world"));

    // Turtlebots
    fast_turtle::RobotDataArray tb_burgers_msg;
    fast_turtle::RobotData tb_burger_msg;
    fast_turtle::RobotDataArray simple_drones_msg;
    fast_turtle::RobotData simple_drone_msg;
    for (int i = 0; i < ft->get_world()->get_n_burgers(); i++)
    {
        // Send tf for tb_burger i
        transform.setOrigin(tf::Vector3(ft->get_world()->get_burger(i)->x(), ft->get_world()->get_burger(i)->y(), 0.0));
        q.setRPY(0, 0, ft->get_world()->get_burger(i)->get_theta());
        transform.setRotation(q);
        tf_br_tb_burger.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "tb_burger_" + ft->get_world()->get_burger(i)->get_model() + "_" + ft->get_world()->get_burger(i)->get_name()));
        // Send tb_burger data message to topic "turtlebot_burgers"
        tb_burgers_msg.header.stamp = ros::Time::now();
        tb_burger_msg.pose.position.x = ft->get_world()->get_burger(i)->x();
        tb_burger_msg.pose.position.y = ft->get_world()->get_burger(i)->y();
        tb_burger_msg.header.stamp = tb_burgers_msg.header.stamp;
        tb_burger_msg.name = ft->get_world()->get_burger(i)->get_name();
        tb_burger_msg.model = ft->get_world()->get_burger(i)->get_model();
        tb_burger_msg.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, ft->get_world()->get_burger(i)->get_theta());
        tb_burger_msg.scan.header.frame_id = "tb_burger_" + ft->get_world()->get_burger(i)->get_model() + "_" + ft->get_world()->get_burger(i)->get_name();
        tb_burger_msg.scan.ranges = ft->get_world()->get_burger(i)->get_lidar()->get_lasers();
        tb_burgers_msg.robots.push_back(tb_burger_msg);
    }
    tb_burgers_publisher.publish(tb_burgers_msg);

    // Drones
    for (int i = 0; i < ft->get_world()->get_n_simple_drones(); i++)
    {
        // Send tf for drone i
        transform.setOrigin(tf::Vector3(ft->get_world()->get_simple_drone(i)->x(), ft->get_world()->get_simple_drone(i)->y(), 0.0));
        q.setRPY(0, 0, ft->get_world()->get_simple_drone(i)->get_theta());
        transform.setRotation(q);
        tf_br_simple_drone.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "simple_drone_" + ft->get_world()->get_simple_drone(i)->get_model() + "_" + ft->get_world()->get_simple_drone(i)->get_name()));
        // Send robot data message to topic "drones"
        simple_drones_msg.header.stamp = ros::Time::now();
        simple_drone_msg.pose.position.x = ft->get_world()->get_simple_drone(i)->x();
        simple_drone_msg.pose.position.y = ft->get_world()->get_simple_drone(i)->y();
        simple_drone_msg.header.stamp = simple_drones_msg.header.stamp;
        simple_drone_msg.name = ft->get_world()->get_simple_drone(i)->get_name();
        simple_drone_msg.model = ft->get_world()->get_simple_drone(i)->get_model();
        simple_drone_msg.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, ft->get_world()->get_simple_drone(i)->get_theta());
        simple_drone_msg.scan.header.frame_id = "simple_drone_" + ft->get_world()->get_simple_drone(i)->get_model() + "_" + ft->get_world()->get_simple_drone(i)->get_name();
        simple_drone_msg.scan.ranges = ft->get_world()->get_simple_drone(i)->get_lidar()->get_lasers();
        simple_drones_msg.robots.push_back(simple_drone_msg);
    }
    simple_drones_publisher.publish(simple_drones_msg);
}

int main(int argc, char **argv)
{
    // Init node
    ros::init(argc, argv, "test_drones");
    ROS_INFO("Initializing Test Drones Simulation");

    // Node object
    ros::NodeHandle nh;

    // Marker publishers
    world_marker_publisher = nh.advertise<visualization_msgs::Marker>("world_marker", 0);
    tb_burger_markers_publisher = nh.advertise<visualization_msgs::MarkerArray>("tb_burger_markers", 0);
    tb_burger_orientation_markers_publisher = nh.advertise<visualization_msgs::MarkerArray>("tb_burger_orientation_markers", 0);
    obstacle_markers_publisher = nh.advertise<visualization_msgs::MarkerArray>("obstacle_markers", 0);
    wall_markers_publisher = nh.advertise<visualization_msgs::MarkerArray>("wall_markers", 0);
    simple_drone_markers_publisher = nh.advertise<visualization_msgs::MarkerArray>("simple_drone_markers", 0);
    simple_drone_supports_markers_publisher = nh.advertise<visualization_msgs::MarkerArray>("simple_drone_supports_markers", 0);
    laser_scan_publisher = nh.advertise<sensor_msgs::LaserScan>("scan", 0);
    
    // Subscribers for all robots
    ros::Subscriber sub_tbb0 = nh.subscribe("cmd_vel_tbb0", 1000, listen_cmd_vel_tbb0);
    ros::Subscriber sub_tbb1 = nh.subscribe("cmd_vel_tbb1", 1000, listen_cmd_vel_tbb1);
    ros::Subscriber sub_tbb2 = nh.subscribe("cmd_vel_tbb2", 1000, listen_cmd_vel_tbb2);
    ros::Subscriber sub_tbb3 = nh.subscribe("cmd_vel_tbb3", 1000, listen_cmd_vel_tbb3);
    ros::Subscriber sub_sd0 = nh.subscribe("cmd_vel_sd0", 1000, listen_cmd_vel_sd0);

    // Data publishers
    tb_burgers_publisher = nh.advertise<fast_turtle::RobotDataArray>("turtlebot_burgers", 1000);
    simple_drones_publisher = nh.advertise<fast_turtle::RobotDataArray>("simple_drones", 1000);

    // Initialize world
    ft->init_world(8, 0, 0, "square");

    // Initialize obstacles/objects
    float obstacle_radius = 0.15;
    ft->add_obstacle(0, -2, obstacle_radius, "round");
    ft->add_obstacle(0, 2, obstacle_radius, "round");
    ft->add_obstacle(-1, -1, obstacle_radius, "round");
    ft->add_obstacle(-1, -2, obstacle_radius, "round");
    ft->add_simple_drone(1, 1.5, 0.5, BURGER_RADIUS, "drone0", 0.2);
    ft->add_simple_drone(2, 1, 0.5, BURGER_RADIUS, "drone1", 0.2);
    ft->add_simple_drone(-2, 1, 0.5, BURGER_RADIUS, "drone2", 0.2);
    ft->add_simple_drone(2, -2, 0.5, BURGER_RADIUS, "drone3", 0.2);
    ft->add_wall(2, 0, 2, 2);
    
    // Initialize swarm competition giving the robot names
    sc->init(ft->get_world()->get_robot_names());

    // Add robots into teams
    sc->enlist("drone0", 0);
    sc->enlist("drone1", 0);
    sc->enlist("drone2", 0);
    sc->enlist("drone3", 2);

    sc->start_time("drone0");
    sc->start_time("drone2");
    sc->food_was_captured("drone0");
    sc->the_robot_lost("drone0");
    sc->the_robot_lost("drone1");

    // Send first world data and graphics data
    publish_data();
    init_graphics_and_data();

    // Frames per second
    ros::Rate loop_rate(SIMULATION_FPS);

    // Main cycle
    while (ros::ok())
    {
        // Updates logic
        update_physics();

        // Publishes fresh data
        publish_data();

        // Repaint graphics
        repaint();

        // Spin (so callbacks can work)
        ros::spinOnce();

        // Sleep
        loop_rate.sleep();

        // Update swarm competitions
        sc->step();
    }
}
