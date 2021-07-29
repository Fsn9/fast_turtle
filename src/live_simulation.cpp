// Simulator library
#include "fast_turtle.h"
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

// Global variables
// Initialize fast turtle simulator object
FastTurtle* ft;
// Markers
visualization_msgs::Marker world_marker;
visualization_msgs::Marker robot_orientation_marker;
visualization_msgs::Marker robot_marker;
visualization_msgs::MarkerArray obstacle_markers;
ros::Publisher world_marker_publisher;
ros::Publisher robot_marker_publisher;
ros::Publisher robot_orientation_marker_publisher;
ros::Publisher obstacle_markers_publisher;

// Publishers
sensor_msgs::LaserScan laser_scan_msg;
ros::Publisher robots_publisher;
ros::Publisher laser_scan_publisher;

void listen_cmd_vel(const geometry_msgs::Twist& msg)
{
    ROS_INFO("Received commands v: %f and w: %f", msg.linear.x, msg.angular.z);
    ft->act(msg.linear.x, msg.angular.z, 0);
    std::cout << "[Robot position]: " << ft->get_world()->get_burger(0)->tostring() << "\n";
}

void init_graphics(){
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

    // Robot marker
    robot_marker.header.frame_id = "world"; // change this
    robot_marker.ns = "simulator_markers";
    robot_marker.id = 1;
    robot_marker.type = visualization_msgs::Marker::CYLINDER;
    robot_marker.action = visualization_msgs::Marker::ADD;
    robot_marker.pose.position.x = ft->get_world()->get_burger(0)->x();
    robot_marker.pose.position.y = ft->get_world()->get_burger(0)->y();
    robot_marker.pose.position.z = 0.192 * 0.5;
    robot_marker.pose.orientation.x = 0.0;
    robot_marker.pose.orientation.y = 0.0;
    robot_marker.pose.orientation.z = 0.0;
    robot_marker.pose.orientation.w = 1.0;
    robot_marker.scale.x = ft->get_world()->get_burger(0)->get_diameter();
    robot_marker.scale.y = ft->get_world()->get_burger(0)->get_diameter();
    robot_marker.scale.z = 0.192;
    robot_marker.color.a = 1; // Don't forget to set the alpha!
    robot_marker.color.r = 0.0;
    robot_marker.color.g = 0.0;
    robot_marker.color.b = 1.0;

    // Robot orientation marker
    geometry_msgs::Point arrow_origin, arrow_end;
    geometry_msgs::Quaternion q;
    robot_orientation_marker.header.frame_id = "robot";
    robot_orientation_marker.ns = "simulator_markers";
    robot_orientation_marker.id = 2;
    robot_orientation_marker.type = visualization_msgs::Marker::ARROW;
    robot_orientation_marker.action = visualization_msgs::Marker::ADD;
    robot_orientation_marker.scale.x = 0.02;
    robot_orientation_marker.scale.y = 0.02;
    robot_orientation_marker.scale.z = 0.02;
    robot_orientation_marker.color.a = 1;
    robot_orientation_marker.color.r = 0.0;
    robot_orientation_marker.color.g = 0.0;
    robot_orientation_marker.color.b = 0.0;
    robot_orientation_marker.pose.orientation.x = 0;
    robot_orientation_marker.pose.orientation.y = 0;
    robot_orientation_marker.pose.orientation.z = 0;
    robot_orientation_marker.pose.orientation.w = 1;
    arrow_origin.x = 0.0;
    arrow_origin.y = 0.0;
    arrow_origin.z = robot_marker.scale.z;
    arrow_end.x = arrow_origin.x + 0.3;
    arrow_end.y = arrow_origin.y;
    arrow_end.z = robot_marker.scale.z;
    robot_orientation_marker.points.clear();
    robot_orientation_marker.points.push_back(arrow_origin);
    robot_orientation_marker.points.push_back(arrow_end);

    // Obstacles
    visualization_msgs::Marker obstacle_marker;
    for(int i = 0; i < ft->get_world()->get_round_obstacles().size(); i++){
        obstacle_marker.header.frame_id = "world";
        obstacle_marker.ns = "simulation_markers";
        obstacle_marker.id = 3 + i;
        obstacle_marker.type = visualization_msgs::Marker::CYLINDER;
        obstacle_marker.action = visualization_msgs::Marker::ADD;
        obstacle_marker.pose.position.x = ft->get_world()->get_round_obstacle(i)->get_xc();
        obstacle_marker.pose.position.y = ft->get_world()->get_round_obstacle(i)->get_yc();
        obstacle_marker.pose.position.z = 0.192 * 0.5;
        obstacle_marker.pose.orientation.x = 0.0;
        obstacle_marker.pose.orientation.y = 0.0;
        obstacle_marker.pose.orientation.z = 0.0;
        obstacle_marker.pose.orientation.w = 1.0;
        obstacle_marker.scale.x = ft->get_world()->get_round_obstacle(i)->get_diameter();
        obstacle_marker.scale.y = ft->get_world()->get_round_obstacle(i)->get_diameter();
        obstacle_marker.scale.z = 0.192;
        obstacle_marker.color.a = 1.0;
        obstacle_marker.color.r = 1.0;
        obstacle_marker.color.g = 1.0;
        obstacle_marker.color.b = 1.0;
        obstacle_markers.markers.push_back(obstacle_marker);
    }

    laser_scan_msg.ranges = ft->get_world()->get_burger(0)->get_lidar()->get_lasers();
    laser_scan_msg.angle_min = 0;
    laser_scan_msg.angle_max = M_PI * 2;
    laser_scan_msg.angle_increment = 2.0 * M_PI / 360;
    laser_scan_msg.range_min = MIN_DISTANCE;
    laser_scan_msg.range_max = MAX_DISTANCE+1e-4;
    laser_scan_msg.time_increment = 0;
    //laser_scan_msg.scan_time = ft->get_world()->get_burger(0)->get_dt();
    laser_scan_msg.scan_time = 0;
    laser_scan_msg.header.frame_id = "robot";

    //only if using a MESH_RESOURCE world_marker type:
    //world_marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
    world_marker_publisher.publish(world_marker);
    robot_marker_publisher.publish(robot_marker);
    robot_orientation_marker_publisher.publish(robot_orientation_marker);
    obstacle_markers_publisher.publish(obstacle_markers);
    laser_scan_publisher.publish(laser_scan_msg);
}

void repaint(){
    // World marker
    world_marker_publisher.publish(world_marker);
    
    // Robot marker
    robot_marker.pose.position.x = ft->get_world()->get_burger(0)->x();
    robot_marker.pose.position.y = ft->get_world()->get_burger(0)->y();
    robot_marker_publisher.publish(robot_marker);

    // Robot orientation marker
    robot_orientation_marker_publisher.publish(robot_orientation_marker);

    // Obstacles
    obstacle_markers_publisher.publish(obstacle_markers);

    // Laser Scan
    laser_scan_msg.ranges = ft->get_world()->get_burger(0)->get_lidar()->get_lasers();
    laser_scan_publisher.publish(laser_scan_msg);
}

void send_data(){
    tf::Transform transform;
    tf::Quaternion q;
    static tf::TransformBroadcaster tf_br_robot;
    static tf::TransformBroadcaster tf_br_map;
    // World
    transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
    q.setRPY(0,0,0);
    transform.setRotation(q);
    tf_br_map.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "world"));

    // Robot
    transform.setOrigin(tf::Vector3(ft->get_world()->get_burger(0)->x(), ft->get_world()->get_burger(0)->y(), 0.0));
    q.setRPY(0, 0, ft->get_world()->get_burger(0)->get_theta());
    transform.setRotation(q);
    tf_br_robot.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "robot"));
    fast_turtle::RobotDataArray robots_msg;
    fast_turtle::RobotData robot_msg;
    robots_msg.header.stamp = ros::Time::now();
    robot_msg.pose.position.x = ft->get_world()->get_burger(0)->x();
    robot_msg.pose.position.y = ft->get_world()->get_burger(0)->y();
    robot_msg.header.stamp = robots_msg.header.stamp;
    robot_msg.name = ft->get_world()->get_burger(0)->get_name();
    robot_msg.model = ft->get_world()->get_burger(0)->get_model();
    robot_msg.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, ft->get_world()->get_burger(0)->get_theta());
    robots_msg.robots.push_back(robot_msg);
    robots_publisher.publish(robots_msg);
}

int main(int argc, char** argv)
{
    // Init node
    ros::init(argc, argv, "live_simulation");
    ROS_INFO("Initializing live simulation");

    // Node object
    ros::NodeHandle nh;

    // Initialize markers
    world_marker_publisher = nh.advertise<visualization_msgs::Marker>("world_marker", 0);
    robot_marker_publisher = nh.advertise<visualization_msgs::Marker>("robot_marker",0);
    robot_orientation_marker_publisher = nh.advertise<visualization_msgs::Marker>("robot_orientation_marker",0);
    obstacle_markers_publisher = nh.advertise<visualization_msgs::MarkerArray>("obstacle_markers",0);
    laser_scan_publisher = nh.advertise<sensor_msgs::LaserScan>("laser_scan", 10);

    // Subscriber
    ros::Subscriber sub = nh.subscribe("cmd_vel", 1000, listen_cmd_vel);

    // Publishers
    robots_publisher = nh.advertise<fast_turtle::RobotDataArray>("robots", 1000);

    // Initialize simulator object
    ft = new FastTurtle();
    float obstacle_radius = 0.15;
    ft->init_world(4, 0, 0, "square");
    ft->add_obstacle(0, -2, obstacle_radius, "round", false);
    ft->add_obstacle(0, 2, obstacle_radius, "round", false);
    ft->add_obstacle(-1, -1, obstacle_radius, "round", false);
    ft->add_obstacle(-1, -2, obstacle_radius, "round", false);
    ft->add_turtlebot_burger(0, -1, -M_PI_2, BURGER_RADIUS, 0.1, "michelangelo");

    // Send first world data and graphics data
    send_data();
    init_graphics();

    // Main cycle
    while(ros::ok()){
        // Repaint graphics
        repaint();
        // Publishes fresh data
        send_data();
        // Spin (so callbacks can work)
        ros::spinOnce();
    }

}

