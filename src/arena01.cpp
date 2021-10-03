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

// Simulation frames per second
#define SIMULATION_FPS 40u

// Initialize fast turtle simulator object
FastTurtle* ft = new FastTurtle(SIMULATION_FPS);

// Markers
visualization_msgs::Marker world_marker;
visualization_msgs::MarkerArray obstacle_markers;
visualization_msgs::MarkerArray wall_markers;
visualization_msgs::MarkerArray robot_markers;
visualization_msgs::MarkerArray robot_orientation_markers;

// Marker Publishers
ros::Publisher world_marker_publisher;
ros::Publisher obstacle_markers_publisher;
ros::Publisher wall_markers_publisher;
ros::Publisher robot_markers_publisher;
ros::Publisher robot_orientation_markers_publisher;

// Publishers
ros::Publisher robots_publisher;

// Messages
sensor_msgs::LaserScan laser_scan_msg;

// Vector of real time command velocities for all robots
std::vector<cmd_vel_> cmd_vels{{0,0},{0,0},{0,0},{0,0}};

// Move robot 'idx'
void move_robot(int idx)
{
    if (cmd_vels[idx].v > MAX_LIN_VELOCITY) cmd_vels[idx].v = MAX_LIN_VELOCITY;
    if (cmd_vels[idx].w > MAX_ANG_VELOCITY) cmd_vels[idx].w = MAX_ANG_VELOCITY;
    ft->act(cmd_vels[idx].v, cmd_vels[idx].w, idx);
}

// Updates logic of the simulator
void update_physics()
{
    for(int idx = 0; idx < ft->get_world()->get_n_burgers(); idx++) move_robot(idx);
}

void listen_cmd_vel(const geometry_msgs::Twist& msg)
{
    if (ft->get_world()->get_n_burgers() > 0){
        ROS_INFO("Received commands v: %f and w: %f", msg.linear.x, msg.angular.z);
        cmd_vels[0].v = msg.linear.x;
        cmd_vels[0].w = msg.angular.z;
        std::cout << "[Robot 0 pose]: " << ft->get_world()->get_burger(0)->tostring() << "\n";
    }
}

void listen_cmd_vel1(const geometry_msgs::Twist& msg)
{
    if (ft->get_world()->get_n_burgers() > 1){
        ROS_INFO("Received commands v: %f and w: %f", msg.linear.x, msg.angular.z);
        cmd_vels[1].v = msg.linear.x;
        cmd_vels[1].w = msg.angular.z;
        std::cout << "[Robot 1 pose]: " << ft->get_world()->get_burger(1)->tostring() << "\n";
    }
}

void listen_cmd_vel2(const geometry_msgs::Twist& msg)
{
    if (ft->get_world()->get_n_burgers() > 2){
        ROS_INFO("Received commands v: %f and w: %f", msg.linear.x, msg.angular.z);
        cmd_vels[2].v = msg.linear.x;
        cmd_vels[2].w = msg.angular.z;
        std::cout << "[Robot 2 pose]: " << ft->get_world()->get_burger(2)->tostring() << "\n";
    }
}

void listen_cmd_vel3(const geometry_msgs::Twist& msg)
{
    if (ft->get_world()->get_n_burgers() > 3){
        ROS_INFO("Received commands v: %f and w: %f", msg.linear.x, msg.angular.z);
        cmd_vels[3].v = msg.linear.x;
        cmd_vels[3].w = msg.angular.z;
        std::cout << "[Robot 3 pose]: " << ft->get_world()->get_burger(3)->tostring() << "\n";
    }
}

void init_graphics_and_data(){
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
    j+=1;

    // Robots position and orientatio 
    visualization_msgs::Marker robot_marker;
    geometry_msgs::Point arrow_origin, arrow_end;
    geometry_msgs::Quaternion q;
    visualization_msgs::Marker robot_orientation_marker;
    for(i = 0; i < ft->get_world()->get_n_burgers(); i++){
        // Robot - cilindros que sao os turtlebots
        robot_marker.header.frame_id = "world";
        robot_marker.ns = "simulation_markers";
        robot_marker.id = j;
        robot_marker.type = visualization_msgs::Marker::CYLINDER;
        robot_marker.action = visualization_msgs::Marker::ADD;
        robot_marker.pose.position.x = ft->get_world()->get_burger(i)->get_xc();
        robot_marker.pose.position.y = ft->get_world()->get_burger(i)->get_yc();
        robot_marker.pose.position.z = 1;//0.192 * 0.5;
        robot_marker.pose.orientation.x = 0.0;
        robot_marker.pose.orientation.y = 0.0;
        robot_marker.pose.orientation.z = 0.0;
        robot_marker.pose.orientation.w = 1.0;
        robot_marker.scale.x = ft->get_world()->get_burger(i)->get_diameter();
        robot_marker.scale.y = ft->get_world()->get_burger(i)->get_diameter();
        robot_marker.scale.z = 0.192;
        robot_marker.color.a = 1.0;
        robot_marker.color.r = 0.0;
        robot_marker.color.g = 0.0;
        robot_marker.color.b = 1.0;
        robot_markers.markers.push_back(robot_marker);
        j+=1;
        // Robot orientation - setinha em cima do robot
        robot_orientation_marker.header.frame_id = "robot_" 
        + ft->get_world()->get_burger(i)->get_model() + "_" 
        + ft->get_world()->get_burger(i)->get_name();
        robot_orientation_marker.ns = "simulation_markers";
        robot_orientation_marker.id = j;
        robot_orientation_marker.type = visualization_msgs::Marker::ARROW;
        robot_orientation_marker.action = visualization_msgs::Marker::ADD;
        robot_orientation_marker.pose.orientation.x = 0.0;
        robot_orientation_marker.pose.orientation.y = 0.0;
        robot_orientation_marker.pose.orientation.z = 0.0;
        robot_orientation_marker.pose.orientation.w = 1.0;
        robot_orientation_marker.scale.x = 0.02;
        robot_orientation_marker.scale.y = 0.02;
        robot_orientation_marker.scale.z = 0.02;
        robot_orientation_marker.color.a = 1.0;
        robot_orientation_marker.color.r = 0.0;
        robot_orientation_marker.color.g = 1.0;
        robot_orientation_marker.color.b = 0.0;
        arrow_origin.x = 0.0;
        arrow_origin.y = 0.0;
        arrow_origin.z = robot_marker.scale.z;
        arrow_end.x = arrow_origin.x + 0.3;
        arrow_end.y = arrow_origin.y;
        arrow_end.z = arrow_origin.z;
        robot_orientation_marker.points.clear();    
        robot_orientation_marker.points.push_back(arrow_origin);
        robot_orientation_marker.points.push_back(arrow_end);
        robot_orientation_markers.markers.push_back(robot_orientation_marker);
        j+=1;   
    }

    // Obstacles - cilindros obstaculos
    visualization_msgs::Marker obstacle_marker;
    for(i = 0; i < ft->get_world()->get_round_obstacles().size(); i++){
        obstacle_marker.header.frame_id = "world";
        obstacle_marker.ns = "simulation_markers";
        obstacle_marker.id = j;
        obstacle_marker.type = visualization_msgs::Marker::CYLINDER;
        obstacle_marker.action = visualization_msgs::Marker::ADD;
        obstacle_marker.pose.position.x = ft->get_world()->get_round_obstacle(i)->get_xc();
        obstacle_marker.pose.position.y = ft->get_world()->get_round_obstacle(i)->get_yc();
        obstacle_marker.pose.position.z = 1.0; //0.192 * 0.5;
        obstacle_marker.pose.orientation.x = 0.0;
        obstacle_marker.pose.orientation.y = 0.0;
        obstacle_marker.pose.orientation.z = 0.0;
        obstacle_marker.pose.orientation.w = 1.0;
        obstacle_marker.scale.x = ft->get_world()->get_round_obstacle(i)->get_diameter();
        obstacle_marker.scale.y = ft->get_world()->get_round_obstacle(i)->get_diameter();
        obstacle_marker.scale.z = 2;//0.192;
        obstacle_marker.color.a = 1.0;
        obstacle_marker.color.r = 1.0;
        obstacle_marker.color.g = 1.0;
        obstacle_marker.color.b = 1.0;
        obstacle_markers.markers.push_back(obstacle_marker);
        j+=1;
    }
    // Obstacles - paredes obstaculos
    visualization_msgs::Marker wall_marker;
    for(i = 0; i < ft->get_world()->get_wall_obstacles().size(); i++){
        wall_marker.header.frame_id = "world";
        wall_marker.ns = "simulation_markers";
        wall_marker.id = j;
        wall_marker.type = visualization_msgs::Marker::CUBE;
        wall_marker.action = visualization_msgs::Marker::ADD;
        wall_marker.pose.position.x = ft->get_world()->get_wall_obstacle(i)->get_xc();
        wall_marker.pose.position.y = ft->get_world()->get_wall_obstacle(i)->get_yc();
        wall_marker.pose.position.z = 1.0;
        wall_marker.pose.orientation.x = 0.0;
        wall_marker.pose.orientation.y = 0.0;
        wall_marker.pose.orientation.z = ft->get_world()->get_wall_obstacle(i)->get_angle();//1.0; //1 - para estar assim | ; 0 - para esta assim _ ;
        wall_marker.pose.orientation.w = 1.0;
        wall_marker.scale.x = ft->get_world()->get_wall_obstacle(i)->get_length();
        wall_marker.scale.y = 0.01;//ft->get_world()->get_wall_obstacle(i)->get_length();
        wall_marker.scale.z = 2.0;//ft->get_world()->get_wall_obstacle(i)->get_length();
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
    laser_scan_msg.range_max = MAX_DISTANCE+1e-4;
    laser_scan_msg.time_increment = 0;
    laser_scan_msg.scan_time = 0;

    //only if using a MESH_RESOURCE world_marker type:
    //world_marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
    world_marker_publisher.publish(world_marker);
    robot_markers_publisher.publish(robot_markers);
    robot_orientation_markers_publisher.publish(robot_orientation_markers);
    obstacle_markers_publisher.publish(obstacle_markers);
    wall_markers_publisher.publish(wall_markers);
}

void repaint(){
    // World marker
    world_marker_publisher.publish(world_marker);
    
    // Robot position markers
    for(int i = 0; i < ft->get_world()->get_n_burgers(); i++){
        robot_markers.markers[i].pose.position.x = ft->get_world()->get_burger(i)->x();
        robot_markers.markers[i].pose.position.y = ft->get_world()->get_burger(i)->y();
        //robot_markers.markers[i].pose.position.z = 1;
    }
    robot_markers_publisher.publish(robot_markers);

    // Robot orientation markers
    robot_orientation_markers_publisher.publish(robot_orientation_markers);

    // Obstacles
    obstacle_markers_publisher.publish(obstacle_markers);

    // Walls
    wall_markers_publisher.publish(wall_markers);
}

void publish_data(){
    tf::Transform transform;
    tf::Quaternion q;
    static tf::TransformBroadcaster tf_br_robot;
    static tf::TransformBroadcaster tf_br_map;

    // World
    transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
    q.setRPY(0,0,0);
    transform.setRotation(q);
    tf_br_map.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "world"));

    // Robots
    fast_turtle::RobotDataArray robots_msg;
    fast_turtle::RobotData robot_msg;
    for(int i = 0; i < ft->get_world()->get_n_burgers(); i++){
        // Send tf for robot i
        transform.setOrigin(tf::Vector3(ft->get_world()->get_burger(i)->x(), ft->get_world()->get_burger(i)->y(), 1.0-0.192/2));
        q.setRPY(0, 0, ft->get_world()->get_burger(i)->get_theta());
        transform.setRotation(q);
        tf_br_robot.sendTransform(tf::StampedTransform(transform
        , ros::Time::now()
        , "world"
        , "robot_" 
            + ft->get_world()->get_burger(i)->get_model() + "_" 
            + ft->get_world()->get_burger(i)->get_name()
        ));
        // Send robot data message to topic "robots"
        robots_msg.header.stamp = ros::Time::now();
        robot_msg.pose.position.x = ft->get_world()->get_burger(i)->x();
        robot_msg.pose.position.y = ft->get_world()->get_burger(i)->y();
        robot_msg.header.stamp = robots_msg.header.stamp;
        robot_msg.name = ft->get_world()->get_burger(i)->get_name();
        robot_msg.model = ft->get_world()->get_burger(i)->get_model();
        robot_msg.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, ft->get_world()->get_burger(i)->get_theta());
        robot_msg.scan.header.frame_id =  "robot_" 
            + ft->get_world()->get_burger(i)->get_model() + "_" 
            + ft->get_world()->get_burger(i)->get_name();
        robot_msg.scan.ranges = ft->get_world()->get_burger(i)->get_lidar()->get_lasers();
        robots_msg.robots.push_back(robot_msg); 
    }
    robots_publisher.publish(robots_msg);
}

int main(int argc, char** argv)
{
    // Init node
    ros::init(argc, argv, "arena01");
    ROS_INFO("Initializing arena01");

    // Node object
    ros::NodeHandle nh;

    // Initialize markers
    world_marker_publisher = nh.advertise<visualization_msgs::Marker>("world_marker", 0);
    robot_markers_publisher = nh.advertise<visualization_msgs::MarkerArray>("robot_markers",0);
    robot_orientation_markers_publisher = nh.advertise<visualization_msgs::MarkerArray>("robot_orientation_markers",0);
    obstacle_markers_publisher = nh.advertise<visualization_msgs::MarkerArray>("obstacle_markers",0);
    wall_markers_publisher = nh.advertise<visualization_msgs::MarkerArray>("wall_markers",0);


    // Subscribers for all robots
    ros::Subscriber sub0 = nh.subscribe("cmd_vel0", 1000, listen_cmd_vel);
    ros::Subscriber sub1 = nh.subscribe("cmd_vel1", 1000, listen_cmd_vel1);
    ros::Subscriber sub2 = nh.subscribe("cmd_vel2", 1000, listen_cmd_vel2);
    ros::Subscriber sub3 = nh.subscribe("cmd_vel3", 1000, listen_cmd_vel3);

    // Publishers
    robots_publisher = nh.advertise<fast_turtle::RobotDataArray>("robots", 1000);

    // Initialize simulator object
    float obstacle_radius = 0.15;
    ft->init_world(20, 0, 0, "square");
    
    // Obstaculos
    ft->add_obstacle(-6, 4, obstacle_radius, "round");
    ft->add_obstacle(2, 8, obstacle_radius, "round");
    ft->add_obstacle(-2, -2, obstacle_radius, "round");
    ft->add_obstacle(2, -4, obstacle_radius, "round");
    ft->add_obstacle(6, -6, obstacle_radius, "round");
    ft->add_wall(2, 0, -9, 1, "squared");
    ft->add_wall(2, -1, -9, 1, "squared");
    ft->add_wall(2, 1, -9, 1, "squared");
    ft->add_wall(2, 2, -9, 1, "squared");
    ft->add_wall(2, -6, 7, 1, "squared");
    ft->add_wall(2, -7, 6, 0, "squared");
    ft->add_wall(2, -6, 1, 1, "squared");
    ft->add_wall(2, -5, 0, 0, "squared");
    ft->add_wall(2, -6, -3, 1, "squared");
    ft->add_wall(2, 9, -2, 0, "squared");
    ft->add_wall(2, 8, -3, 1, "squared");
    ft->add_wall(2, -2, 5, 1, "squared");
    ft->add_wall(2, -1, 6, 0, "squared");
    ft->add_wall(2, 0, 5, 1, "squared");
    ft->add_wall(2, 2, 1, 1, "squared");
    ft->add_wall(2, 3, 0, 0, "squared");
    ft->add_wall(2, 4, 1, 1, "squared");
    ft->add_wall(2, 5, 2, 0, "squared");
    ft->add_wall(2, 6, 1, 1, "squared");
    ft->add_wall(2, 6, 7, 1, "squared");
    ft->add_wall(2, 7, 6, 0, "squared");
    ft->add_wall(2, 8, 7, 1, "squared");
    
    // Paredes da bounding box
    ft->add_wall(20, 10, 0, 1, "squared");
    ft->add_wall(20, 0, 10, 0, "squared");
    ft->add_wall(20, -10, 0, 1, "squared");
    ft->add_wall(20, 0, -10, 0, "squared");
    
    // Drones
    ft->add_turtlebot_burger(0.5, -9, M_PI_2, BURGER_RADIUS, "michelangelo", 0.1);
    ft->add_turtlebot_burger(-0.5, -9, M_PI_2, BURGER_RADIUS, "leonardo", 0.2);
    ft->add_turtlebot_burger(1.5, -9, M_PI_2, BURGER_RADIUS, "raphael", 0.2);

    // Send first world data and graphics data
    publish_data();
    init_graphics_and_data();

    // Frames per second
    ros::Rate loop_rate(SIMULATION_FPS);

    // Main cycle
    while(ros::ok()){
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
    }

}
