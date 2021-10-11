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

// Values for food distances
#define FOOD_LIMIT_X_sup -6
#define FOOD_LIMIT_X_inf -10
#define FOOD_LIMIT_Y_sup -6
#define FOOD_LIMIT_Y_inf -10
#define PROXIMITY 0.3

// Drone structure parameters
#define SIMPLE_DRONE_SUPPORTS_THICKNESS 0.01f

// Initialize fast turtle simulator object
FastTurtle* ft = new FastTurtle(SIMULATION_FPS);

// Markers
visualization_msgs::Marker world_marker;
visualization_msgs::MarkerArray obstacle_markers;
visualization_msgs::MarkerArray wall_markers;
visualization_msgs::MarkerArray simple_drone_markers;
visualization_msgs::MarkerArray simple_drone_supports_markers;
visualization_msgs::MarkerArray food_markers;

// Marker Publishers
ros::Publisher world_marker_publisher;
ros::Publisher obstacle_markers_publisher;
ros::Publisher wall_markers_publisher;
ros::Publisher simple_drone_markers_publisher;
ros::Publisher food_markers_publisher; 
ros::Publisher simple_drone_supports_markers_publisher;


visualization_msgs::Marker stats_marker;
ros::Publisher stats_marker_publisher;

// Publishers
ros::Publisher simple_drones_publisher;

// Messages
sensor_msgs::LaserScan laser_scan_msg;

// Vector of real time command velocities for all robots
std::vector<cmd_vel_sd> cmd_vels_simple_drones{{0,0},{0,0},{0,0},{0,0}};

// Move robot 'idx'
void move_simple_drones(int idx)
{
    if (cmd_vels_simple_drones[idx].vx > MAX_LIN_VELOCITY_SIMPLE_DRONE) cmd_vels_simple_drones[idx].vy = MAX_LIN_VELOCITY_SIMPLE_DRONE;
    if (cmd_vels_simple_drones[idx].vy > MAX_LIN_VELOCITY_SIMPLE_DRONE) cmd_vels_simple_drones[idx].vy = MAX_LIN_VELOCITY_SIMPLE_DRONE;
    ft->act_simple_drone(cmd_vels_simple_drones[idx].vx, cmd_vels_simple_drones[idx].vy, idx);
}

// Updates logic of the simulator
void update_physics()
{    
    //if(ft->get_world()->get_burger(idx)->is_visible()){
    for(int idx = 0; idx < ft->get_world()->get_n_simple_drones(); idx++){
        if(ft->get_world()->get_simple_drone(idx)->is_visible()) move_simple_drones(idx);
    } 
        
}


void listen_cmd_vel_sd0(const geometry_msgs::Twist& msg)
{
    if (ft->get_world()->get_n_simple_drones() > 0){
        ROS_INFO("Received commands vx: %f and vy: %f", msg.linear.x, msg.linear.y);
        cmd_vels_simple_drones[0].vx = msg.linear.x;
        cmd_vels_simple_drones[0].vy = msg.linear.y;
        std::cout << "[Simple Drone 0 pose]: " << ft->get_world()->get_simple_drone(0)->tostring() << "\n";
    }
}

void listen_cmd_vel_sd1(const geometry_msgs::Twist& msg)
{
    if (ft->get_world()->get_n_simple_drones() > 0){
        ROS_INFO("Received commands vx: %f and vy: %f", msg.linear.x, msg.linear.y);
        cmd_vels_simple_drones[1].vx = msg.linear.x;
        cmd_vels_simple_drones[1].vy = msg.linear.y;
        std::cout << "[Simple Drone 1 pose]: " << ft->get_world()->get_simple_drone(1)->tostring() << "\n";
    }
}

void listen_cmd_vel_sd2(const geometry_msgs::Twist& msg)
{
    if (ft->get_world()->get_n_simple_drones() > 0){
        ROS_INFO("Received commands vx: %f and vy: %f", msg.linear.x, msg.linear.y);
        cmd_vels_simple_drones[2].vx = msg.linear.x;
        cmd_vels_simple_drones[2].vy = msg.linear.y;
        std::cout << "[Simple Drone 2 pose]: " << ft->get_world()->get_simple_drone(2)->tostring() << "\n";
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

    // Simple drones position
    visualization_msgs::Marker simple_drone_marker;
    for(i = 0; i < ft->get_world()->get_n_simple_drones(); i++)
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
        for(int sw = 0; sw < 4; sw++)
        {
            sd_support_marker.header.frame_id = "simple_drone_" 
                + ft->get_world()->get_simple_drone(i)->get_model() + "_" 
                + ft->get_world()->get_simple_drone(i)->get_name();
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
        wall_marker.pose.position.x = std::get<0>(ft->get_world()->get_wall_obstacle(i)->get_midpoint());
        wall_marker.pose.position.y = std::get<1>(ft->get_world()->get_wall_obstacle(i)->get_midpoint());
        wall_marker.pose.position.z = 1.0;
        wall_marker.pose.orientation.x = 0.0;
        wall_marker.pose.orientation.y = 0.0;
        wall_marker.pose.orientation.z = ft->get_world()->get_wall_obstacle(i)->is_vertical() ? 1.0 : 0.0;//1.0; //1 - para estar assim | ; 0 - para esta assim _ ;
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
    //base walls, don't count as actuall walls
        wall_marker.header.frame_id = "world";
        wall_marker.ns = "simulation_markers";
        wall_marker.id = j;
        wall_marker.type = visualization_msgs::Marker::CUBE;
        wall_marker.action = visualization_msgs::Marker::ADD;
        wall_marker.pose.position.x = -8.0;
        wall_marker.pose.position.y = -6.0;
        wall_marker.pose.position.z = 0.5;
        wall_marker.pose.orientation.x = 0.0;
        wall_marker.pose.orientation.y = 0.0;
        wall_marker.pose.orientation.z = 0.0;//1.0; //1 - para estar assim | ; 0 - para esta assim _ ;
        wall_marker.pose.orientation.w = 1.0;
        wall_marker.scale.x = 4.0;
        wall_marker.scale.y = 0.01;//ft->get_world()->get_wall_obstacle(i)->get_length();
        wall_marker.scale.z = 1.0;//ft->get_world()->get_wall_obstacle(i)->get_length();
        wall_marker.color.a = 1.0;
        wall_marker.color.r = 0.0;
        wall_marker.color.g = 1.0;
        wall_marker.color.b = 0.7;
        wall_markers.markers.push_back(wall_marker);
        j+=1;

        wall_marker.header.frame_id = "world";
        wall_marker.ns = "simulation_markers";
        wall_marker.id = j;
        wall_marker.type = visualization_msgs::Marker::CUBE;
        wall_marker.action = visualization_msgs::Marker::ADD;
        wall_marker.pose.position.x = -6.0;
        wall_marker.pose.position.y = -8.0;
        wall_marker.pose.position.z = 0.5;
        wall_marker.pose.orientation.x = 0.0;
        wall_marker.pose.orientation.y = 0.0;
        wall_marker.pose.orientation.z = 1.0;//1.0; //1 - para estar assim | ; 0 - para esta assim _ ;
        wall_marker.pose.orientation.w = 1.0;
        wall_marker.scale.x = 4.0;
        wall_marker.scale.y = 0.01;//ft->get_world()->get_wall_obstacle(i)->get_length();
        wall_marker.scale.z = 1.0;//ft->get_world()->get_wall_obstacle(i)->get_length();
        wall_marker.color.a = 1.0;
        wall_marker.color.r = 0.0;
        wall_marker.color.g = 1.0;
        wall_marker.color.b = 0.7;
        wall_markers.markers.push_back(wall_marker);
        j+=1;

    //Food
    visualization_msgs::Marker food_marker;
        for(i = 0; i < ft->get_world()->get_food_items().size(); i++){
            food_marker.header.frame_id = "world";
            food_marker.ns = "simulation_markers";
            food_marker.id = j;
            food_marker.type = visualization_msgs::Marker::CYLINDER;
            food_marker.action = visualization_msgs::Marker::ADD;
            food_marker.pose.position.x = ft->get_world()->get_food_item(i)->get_xc();
            food_marker.pose.position.y = ft->get_world()->get_food_item(i)->get_yc();
            food_marker.pose.position.z = 1;
            food_marker.pose.orientation.x = 0.0;
            food_marker.pose.orientation.y = 0.0;
            food_marker.pose.orientation.z = 0.0;
            food_marker.pose.orientation.w = 1.0;
            food_marker.scale.x = ft->get_world()->get_food_item(i)->get_diameter();
            food_marker.scale.y = ft->get_world()->get_food_item(i)->get_diameter();
            food_marker.scale.z = 0.3;
            food_marker.color.a = 1.0;
            food_marker.color.r = 1.0;
            food_marker.color.g = 0.75;
            food_marker.color.b = 0.79;
            food_markers.markers.push_back(food_marker);
            j+=1;
        }

    stats_marker.header.frame_id = "world";
    stats_marker.ns = "simulator_markers";
    stats_marker.id = j;
    stats_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    stats_marker.action = visualization_msgs::Marker::ADD;
    stats_marker.pose.position.x = -8.0;
    stats_marker.pose.position.y = -8.0;
    stats_marker.pose.position.z = 1.0;
    stats_marker.scale.z = 0.3;
    stats_marker.color.a = 1.0; 
    stats_marker.color.r = 1.0;
    stats_marker.color.g = 1.0;
    stats_marker.color.b = 1.0;
    stats_marker.text = "Stats: \n Number of food_items eaten: 20\n Number of food items left: 10\n Time left: 20:00s";

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
    obstacle_markers_publisher.publish(obstacle_markers);
    wall_markers_publisher.publish(wall_markers);
    simple_drone_supports_markers_publisher.publish(simple_drone_supports_markers);
    stats_marker_publisher.publish(stats_marker);

}
bool inside_base(double x, double y){
    if(x > FOOD_LIMIT_X_inf && x < FOOD_LIMIT_X_sup && y > FOOD_LIMIT_Y_inf && y < FOOD_LIMIT_Y_sup) return true;
    else return false;
}
void repaint(){
    ft->check_collisions();
    // World marker
    world_marker_publisher.publish(world_marker);
    
    // Simple drone position markers
    for(int i = 0; i < ft->get_world()->get_n_simple_drones(); i++){
        simple_drone_markers.markers[i].pose.position.x = ft->get_world()->get_simple_drone(i)->x();
        simple_drone_markers.markers[i].pose.position.y = ft->get_world()->get_simple_drone(i)->y();
    }

    //check for collisions and if they collided, make them disapear 
    ft->check_collisions();
    for(int i = 0; i < ft->get_world()->get_n_simple_drones(); i++){
        if(!ft->get_world()->get_burger(i)->is_visible()){
            simple_drone_markers.markers[i].color.a = 0;
            simple_drone_supports_markers.markers[i].color.a = 0;
        }
    }
     
    // Food markers
    for(int i = 0; i < ft->get_world()->get_food_items().size(); i++){
        for(int j = 0; j < ft->get_world()->get_n_simple_drones(); j++){
            
            if(inside_base(food_markers.markers[i].pose.position.x,food_markers.markers[i].pose.position.y)){
                break;
            }
            // Caught food  //only eats food if it's visible and is not holding another food
            if(ft->get_world()->get_food_item(i)->visible && ft->get_world()->get_simple_drone(j)->is_visible() &&
                abs(simple_drone_markers.markers[j].pose.position.x - food_markers.markers[i].pose.position.x) < PROXIMITY && 
                abs(simple_drone_markers.markers[j].pose.position.y - food_markers.markers[i].pose.position.y) < PROXIMITY && 
                !inside_base(simple_drone_markers.markers[j].pose.position.x, simple_drone_markers.markers[j].pose.position.y) &&
                simple_drone_markers.markers[j].color.r == 0.0){
                ft->get_world()->get_food_item(i)->visible = false;
                ft->get_world()->get_food_item(i)->robot = j;
                simple_drone_markers.markers[j].color.r = 1.0;
                simple_drone_markers.markers[j].color.g = 0.75;
                simple_drone_markers.markers[j].color.b = 0.79;
                food_markers.markers[i].color.a = 0;

                break;
            }
            // Arrived at base, drops food
            else if(ft->get_world()->get_food_item(i)->visible == false && 
            ft->get_world()->get_food_item(i)->robot == j && 
            inside_base(simple_drone_markers.markers[j].pose.position.x, simple_drone_markers.markers[j].pose.position.y)){
                food_markers.markers[i].color.a = 1;
                food_markers.markers[i].pose.position.x = simple_drone_markers.markers[j].pose.position.x;
                food_markers.markers[i].pose.position.y = simple_drone_markers.markers[j].pose.position.y;
                ft->get_world()->get_food_item(i)->visible = true;
                ft->get_world()->get_food_item(i)->robot = -1;
                simple_drone_markers.markers[j].color.r = 0.0;
                simple_drone_markers.markers[j].color.g = 0.0;
                simple_drone_markers.markers[j].color.b = 1.0;
                break;
            }

            //object collided while holding food
            else if(!ft->get_world()->get_burger(j)->is_visible() && ft->get_world()->get_food_item(i)->robot == j){
                food_markers.markers[i].color.a = 1;
                ft->get_world()->get_food_item(i)->visible = true;
                ft->get_world()->get_food_item(i)->robot = -1;
                break;
            }

        }
    }
    // Simple drone markers
    simple_drone_markers_publisher.publish(simple_drone_markers);

    // Simple drone wings markers
    simple_drone_supports_markers_publisher.publish(simple_drone_supports_markers);

    // Obstacles
    obstacle_markers_publisher.publish(obstacle_markers);

    // Walls
    wall_markers_publisher.publish(wall_markers);

    // Food Markers
    food_markers_publisher.publish(food_markers);

    // Stats Markers
    stats_marker_publisher.publish(stats_marker);
  
}
void publish_data(){
    tf::Transform transform;
    tf::Quaternion q;
    static tf::TransformBroadcaster tf_br_simple_drone;
    static tf::TransformBroadcaster tf_br_map;

    // World
    transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
    q.setRPY(0,0,0);
    transform.setRotation(q);
    tf_br_map.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "world"));

    // Drones
    fast_turtle::RobotDataArray simple_drones_msg;
    fast_turtle::RobotData simple_drone_msg;

    // Drones
    for(int i = 0; i < ft->get_world()->get_n_simple_drones(); i++){
        // Send tf for drone i
        transform.setOrigin(tf::Vector3(ft->get_world()->get_simple_drone(i)->x(), ft->get_world()->get_simple_drone(i)->y(), 0.0));
        q.setRPY(0, 0, 0);
        transform.setRotation(q);
        tf_br_simple_drone.sendTransform(tf::StampedTransform(transform
        , ros::Time::now()
        , "world"
        , "simple_drone_"
            + ft->get_world()->get_simple_drone(i)->get_model() + "_"
            + ft->get_world()->get_simple_drone(i)->get_name()
        ));
        // Send robot data message to topic "drones"
        simple_drones_msg.header.stamp = ros::Time::now();
        simple_drone_msg.pose.position.x = ft->get_world()->get_simple_drone(i)->x();
        simple_drone_msg.pose.position.y = ft->get_world()->get_simple_drone(i)->y();
        simple_drone_msg.header.stamp = simple_drones_msg.header.stamp;
        simple_drone_msg.name = ft->get_world()->get_simple_drone(i)->get_name();
        simple_drone_msg.model = ft->get_world()->get_simple_drone(i)->get_model();
        simple_drone_msg.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, 0.0);
        simple_drone_msg.scan.header.frame_id =  "simple_drone_"
            + ft->get_world()->get_simple_drone(i)->get_model() + "_"
            + ft->get_world()->get_simple_drone(i)->get_name();
        simple_drone_msg.scan.ranges = ft->get_world()->get_simple_drone(i)->get_lidar()->get_lasers();
        simple_drones_msg.robots.push_back(simple_drone_msg);
    }
    simple_drones_publisher.publish(simple_drones_msg);
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
    food_markers_publisher = nh.advertise<visualization_msgs::MarkerArray>("food_markers",0);
    obstacle_markers_publisher = nh.advertise<visualization_msgs::MarkerArray>("obstacle_markers",0);
    wall_markers_publisher = nh.advertise<visualization_msgs::MarkerArray>("wall_markers",0);
    stats_marker_publisher = nh.advertise<visualization_msgs::Marker>("stats_marker", 0);
    simple_drone_markers_publisher = nh.advertise<visualization_msgs::MarkerArray>("simple_drone_markers",0);
    simple_drone_supports_markers_publisher = nh.advertise<visualization_msgs::MarkerArray>("simple_drone_supports_markers",0);

    // Subscribers for all robots
    ros::Subscriber sub_sd0 = nh.subscribe("cmd_vel_sd0", 1000, listen_cmd_vel_sd0);
    ros::Subscriber sub_sd1 = nh.subscribe("cmd_vel_sd1", 1000, listen_cmd_vel_sd1);
    ros::Subscriber sub_sd2 = nh.subscribe("cmd_vel_sd2", 1000, listen_cmd_vel_sd2);

    // Publishers
    simple_drones_publisher = nh.advertise<fast_turtle::RobotDataArray>("simple_drones", 1000);

    // Initialize simulator object
    float obstacle_radius = 0.15;
    ft->init_world(20, 0, 0, "square");
    
    // Obstaculos
    ft->add_obstacle(-6, 4, obstacle_radius, "round");
    ft->add_obstacle(2, 8, obstacle_radius, "round");
    ft->add_obstacle(-2, -2, obstacle_radius, "round");
    ft->add_obstacle(2, -4, obstacle_radius, "round");
    ft->add_obstacle(6, -6, obstacle_radius, "round");
    ft->add_wall(0, -10, 0, -8); //bpartida dos robots
    ft->add_wall(-1, -10, -1, -8);
    ft->add_wall(1, -10, 1, -8);
    ft->add_wall(2, -10, 2, -8);
    ft->add_wall(-6, 6, -6, 8); //canto superior esq
    ft->add_wall(-8, 6, -6, 6);
    ft->add_wall(-6, 0, -6, 2); //L da esq
    ft->add_wall(-6, 0, -4, 0);
    ft->add_wall(-6, -4, -6, -2); //traço sozinho
    ft->add_wall(8, -2, 10, -2); //canto inferior dir
    ft->add_wall(8, -4, 8, -2);
    ft->add_wall(-2, 4, -2, 6); //U ao contrario do topo
    ft->add_wall(-2, 6, 0, 6);
    ft->add_wall(0, 4, 0, 6);
    ft->add_wall(2, 0, 2, 2); //S deitado
    ft->add_wall(2, 0, 4, 0);
    ft->add_wall(4, 0, 4, 2);
    ft->add_wall(4, 2, 6, 2);
    ft->add_wall(6, 0, 6, 2);
    ft->add_wall(6, 6, 6, 8); //U do canto superior dir
    ft->add_wall(6, 6, 8, 6);
    ft->add_wall(8, 6, 8, 8);
    
    // Paredes da bounding box
    ft->add_wall(10, -10, 10, 10); //direita
    ft->add_wall(-10, -10, 10, -10); //baixo
    ft->add_wall(-10, -10, -10, 10); //esquerda
    ft->add_wall(-10, 10, 10, 10); //cima

    //Comida
    ft->add_food_item(8, -8, FOOD_RADIUS);
    ft->add_food_item(-8, 8, FOOD_RADIUS); 
    ft->add_food_item(7, 7, FOOD_RADIUS); 
    ft->add_food_item(0, -3, FOOD_RADIUS); 
    ft->add_food_item(-8, 1, FOOD_RADIUS); 
    
    // Drones
    ft->add_simple_drone(-0.5, -9.5, 0.5, BURGER_RADIUS, "drone0", 0.2);
    ft->add_simple_drone(0.5, -9.5, 0.5, BURGER_RADIUS, "drone1", 0.2);
    ft->add_simple_drone(1.5, -9.5, 0.5, BURGER_RADIUS, "drone2", 0.2); 


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
