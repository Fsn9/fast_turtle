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
#include <nav_msgs/Odometry.h>
#include <string.h>
// Services
#include "fast_turtle/ResetArena.h"

// #include <time.h>

// Simulation frames per second
#define SIMULATION_FPS 10u

// Values for food distances
#define FOOD_LIMIT_X_sup -6
#define FOOD_LIMIT_X_inf -10
#define FOOD_LIMIT_Y_sup -6
#define FOOD_LIMIT_Y_inf -10
#define PROXIMITY 0.5
#define RED {1,0,0}
#define GREEN {0,1,0}
#define BLUE {0,0,1}
#define YELLOW {1,1,0}
#define ORANGE {1,0.65,0}
#define WHITE {1,1,1}
#define GREY {0.5,0.5,0.5}
#define PINK {1,0,1}
#define CYAN {0,1,1}
#define PURPLE {0.5,0,0.5}
#define LOOP_PERIOD 0.01



// Drone structure parameters
#define SIMPLE_DRONE_SUPPORTS_THICKNESS 0.01f

float colors[10][3]={RED,GREEN,BLUE,YELLOW,ORANGE,WHITE,GREY,PINK,CYAN,PURPLE};

struct drone {
    int team;
    float rgb[3];
};

// // Teams
// struct team Team;

std::string user_id;
// Initialize fast turtle simulator object
FastTurtle* ft = new FastTurtle(SIMULATION_FPS);

// Markers
visualization_msgs::Marker world_marker;
visualization_msgs::MarkerArray wall_markers;
visualization_msgs::MarkerArray simple_drone_markers;
visualization_msgs::MarkerArray simple_drone_supports_markers;
visualization_msgs::MarkerArray food_markers;

// Marker Publishers
ros::Publisher world_marker_publisher;
ros::Publisher wall_markers_publisher;
ros::Publisher simple_drone_markers_publisher;
ros::Publisher simple_drone_supports_markers_publisher;   
ros::Publisher food_markers_publisher;       


// Publishers
ros::Publisher odom_publisher0;
ros::Publisher laser_publisher0;
ros::Publisher odom_publisher1;
ros::Publisher laser_publisher1;


// Messages
sensor_msgs::LaserScan laser_scan_msg;

// Vector of real time command velocities for all robots
//std::vector<cmd_vel_sd> cmd_vels_simple_drones{{0,0},{0,0},{0,0},{0,0}};
std::vector<cmd_vel_sd> cmd_vels_simple_drones{{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0}
,{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0}
,{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0}};
// Move robot 'idx'
void move_simple_drones(int idx)
{
    if (cmd_vels_simple_drones[idx].vx > MAX_LIN_VELOCITY_SIMPLE_DRONE) cmd_vels_simple_drones[idx].vx = MAX_LIN_VELOCITY_SIMPLE_DRONE;
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
       //ROS_INFO("Received commands vx: %f and vy: %f", msg.linear.x, msg.linear.y);
        
        //saturacao
        if (msg.linear.x > MAX_LIN_VELOCITY_SIMPLE_DRONE){
         cmd_vels_simple_drones[0].vx = MAX_LIN_VELOCITY_SIMPLE_DRONE;
        }else if(msg.linear.x  < -MAX_LIN_VELOCITY_SIMPLE_DRONE){
         cmd_vels_simple_drones[0].vx = -MAX_LIN_VELOCITY_SIMPLE_DRONE;
        }else{
        cmd_vels_simple_drones[0].vx = msg.linear.x;
        }
        if (msg.linear.y > MAX_LIN_VELOCITY_SIMPLE_DRONE){
         cmd_vels_simple_drones[0].vy = MAX_LIN_VELOCITY_SIMPLE_DRONE;
        }else if(msg.linear.y < -MAX_LIN_VELOCITY_SIMPLE_DRONE){
         cmd_vels_simple_drones[0].vy = -MAX_LIN_VELOCITY_SIMPLE_DRONE;
        }else{
        cmd_vels_simple_drones[0].vy = msg.linear.y;
        }
    	
    	
        ft->act_simple_drone(cmd_vels_simple_drones[0].vx, cmd_vels_simple_drones[0].vy, 0);
        //std::cout << "[Simple Drone 0 pose]: " << ft->get_world()->get_simple_drone(0)->tostring() << "\n";
    }
}

void listen_cmd_vel_sd1(const geometry_msgs::Twist& msg)
{

    if (ft->get_world()->get_n_simple_drones() > 0){
        //ROS_INFO("Received commands vx: %f and vy: %f", msg.linear.x, msg.linear.y);
        
        //saturacao
        if (msg.linear.x > MAX_LIN_VELOCITY_SIMPLE_DRONE){
         cmd_vels_simple_drones[1].vx = MAX_LIN_VELOCITY_SIMPLE_DRONE;
        }else if(msg.linear.x  < -MAX_LIN_VELOCITY_SIMPLE_DRONE){
         cmd_vels_simple_drones[1].vx = -MAX_LIN_VELOCITY_SIMPLE_DRONE;
        }else{
        cmd_vels_simple_drones[1].vx = msg.linear.x;
        }
        if (msg.linear.y > MAX_LIN_VELOCITY_SIMPLE_DRONE){
         cmd_vels_simple_drones[1].vy = MAX_LIN_VELOCITY_SIMPLE_DRONE;
        }else if(msg.linear.y < -MAX_LIN_VELOCITY_SIMPLE_DRONE){
         cmd_vels_simple_drones[1].vy = -MAX_LIN_VELOCITY_SIMPLE_DRONE;
        }else{
        cmd_vels_simple_drones[1].vy = msg.linear.y;
        }
    	
    	
        ft->act_simple_drone(cmd_vels_simple_drones[1].vx, cmd_vels_simple_drones[1].vy, 1);
        //std::cout << "[Simple Drone 1 pose]: " << ft->get_world()->get_simple_drone(1)->tostring() << "\n";
    }
}
void init_graphics_and_data(){
    // Counter markers
    int j = 0;
    // For int variable
    int i = 0;
    int T = 1;

    // Assigning Drones to Teams
    int num = 1;
    int count = 0;
    struct drone Drones[40];
    for(int k = 0; k < ft->get_world()->get_n_simple_drones(); k++){
        Drones[k].rgb[0] = colors[num-1][0];
        Drones[k].rgb[1] = colors[num-1][1];
        Drones[k].rgb[2] = colors[num-1][2];
        Drones[k].team = num; 
        if (count == 3){
            num++;
            count = 0;
        } else {
            count++;
        }
    }

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
            sd_support_marker.color.r = Drones[i].rgb[0];//0.0;
            sd_support_marker.color.g = Drones[i].rgb[1]; //0.0;
            sd_support_marker.color.b = Drones[i].rgb[2]; //0.0;
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
    wall_markers_publisher.publish(wall_markers);
    simple_drone_supports_markers_publisher.publish(simple_drone_supports_markers);

}

double calculate_distance(double x1, double y1, double x2, double y2){
    double result = (sqrt(std::pow(x2 - x1, 2) +
                std::pow(y2 - y1, 2) * 1.0));
    ROS_INFO("distancia: %f", result);
    return result;
}
bool inside_base(double x, double y){
    if(x > FOOD_LIMIT_X_inf && x < FOOD_LIMIT_X_sup && y > FOOD_LIMIT_Y_inf && y < FOOD_LIMIT_Y_sup) return true;
    else return false;
}

std::tuple<int, int, int> food_status(int id_food){ //first value: 0 if cant be captured, 1 if has one drone near, 2 if it has two drones near; second value: first drone near, -1 if nothing, third value: second drone near, -1 if nothing
int status = 0;
int first = -1;
int second = -1;
if(ft->get_world()->get_food_item(id_food)->visible){
    for(int j = 0; j < ft->get_world()->get_n_simple_drones(); j++){
        //ROS_INFO("POSE %f\n", simple_drone_markers.markers[j].pose.position.x);
        if(inside_base(food_markers.markers[id_food].pose.position.x,food_markers.markers[id_food].pose.position.y)){
                break;
            }
        if(ft->get_world()->get_food_item(id_food)->visible && ft->get_world()->get_simple_drone(j)->is_visible() &&
                calculate_distance(simple_drone_markers.markers[j].pose.position.x,simple_drone_markers.markers[j].pose.position.y,
                 food_markers.markers[id_food].pose.position.x, food_markers.markers[id_food].pose.position.y)<PROXIMITY && 
                !inside_base(simple_drone_markers.markers[j].pose.position.x, simple_drone_markers.markers[j].pose.position.y)){ //found drone close
                    if(status == 0){ //has no first drone
                        status = 1;
                        first = j;
                    }else if(status == 1){
                        status = 2;
                        second = j;
                    } 
                }
    }
}else{ //os dois drones estão afastados, comida volta ao sitio onde estava
    if(calculate_distance(simple_drone_markers.markers[ft->get_world()->get_food_item(id_food)->robot1].pose.position.x,
                        simple_drone_markers.markers[ft->get_world()->get_food_item(id_food)->robot1].pose.position.y,
                        simple_drone_markers.markers[ft->get_world()->get_food_item(id_food)->robot2].pose.position.x,
                        simple_drone_markers.markers[ft->get_world()->get_food_item(id_food)->robot2].pose.position.y)>1.0){
                            status = -1;
                            first = -1;
                            second = -1;
                        }
    else{
        status = 2;
        first = ft->get_world()->get_food_item(id_food)->robot1;
        second = ft->get_world()->get_food_item(id_food)->robot2;
    }
}
   //FALTA FAZER COLISÕES
    
    //ROS_INFO("status = %d, first = %d, second = %d\n", status, first, second);
    return std::make_tuple(status, first, second);
}    
void repaint(){
    
    ft->check_collisions();
    // World marker
    world_marker_publisher.publish(world_marker);
    
    // Simple drone position markers
    for(int i = 0; i < ft->get_world()->get_n_simple_drones(); i++){
        simple_drone_markers.markers[i].pose.position.x = ft->get_world()->get_simple_drone(i)->x();
        simple_drone_markers.markers[i].pose.position.y = ft->get_world()->get_simple_drone(i)->y();
        //stats_markers.markers[i+1].pose.position.x = ft->get_world()->get_simple_drone(i)->x();
        //stats_markers.markers[i+1].pose.position.y = ft->get_world()->get_simple_drone(i)->y();
    }

     // Food markers
    for(int i = 0; i < ft->get_world()->get_food_items().size(); i++){  
        // Caught food  //only eats food if it's visible and is not holding another food
        std::tuple<int, int, int> result = food_status(i);
        
        if(std::get<0>(result) == 1 && ft->get_world()->get_food_item(i)->robot1 == -1){ //only one drone nearby
            ft->get_world()->get_food_item(i)->robot1 = std::get<1>(result);
            simple_drone_markers.markers[std::get<1>(result)].color.r = 1.0;
            simple_drone_markers.markers[std::get<1>(result)].color.g = 0.0;
            simple_drone_markers.markers[std::get<1>(result)].color.b = 0.0;
            food_markers.markers[i].color.r = 1.0;
            food_markers.markers[i].color.g = 0.0;
            food_markers.markers[i].color.b = 0.0;
            
        }
        else if(std::get<0>(result) == 2 && simple_drone_markers.markers[(std::get<2>(result))].color.g == 0.0){ //two drones nearby
            food_markers.markers[i].color.a = 0;
            food_markers.markers[i].color.g = 0.75;
            food_markers.markers[i].color.b = 0.79;
            simple_drone_markers.markers[std::get<2>(result)].color.r = 1.0;
            simple_drone_markers.markers[std::get<2>(result)].color.g = 0.75;
            simple_drone_markers.markers[std::get<2>(result)].color.b = 0.79;
            simple_drone_markers.markers[std::get<1>(result)].color.r = 1.0;
            simple_drone_markers.markers[std::get<1>(result)].color.g = 0.75;
            simple_drone_markers.markers[std::get<1>(result)].color.b = 0.79;
            ft->get_world()->get_food_item(i)->visible = false;
            ft->get_world()->get_food_item(i)->robot1 = std::get<1>(result);
            ft->get_world()->get_food_item(i)->robot2 = std::get<2>(result);
            
        }
        else if(std::get<0>(result) == -1){ //drones too far away from each other, drops food
            food_markers.markers[i].color.a = 1;
            simple_drone_markers.markers[ft->get_world()->get_food_item(i)->robot1].color.r = 0.0;
            simple_drone_markers.markers[ft->get_world()->get_food_item(i)->robot1].color.g = 0.0;
            simple_drone_markers.markers[ft->get_world()->get_food_item(i)->robot1].color.b = 0.0;
            simple_drone_markers.markers[ft->get_world()->get_food_item(i)->robot2].color.r = 0.0;
            simple_drone_markers.markers[ft->get_world()->get_food_item(i)->robot2].color.g = 0.0;
            simple_drone_markers.markers[ft->get_world()->get_food_item(i)->robot2].color.b = 0.0;
            ft->get_world()->get_food_item(i)->robot1 = -1;
            ft->get_world()->get_food_item(i)->robot2 = -1;
            ft->get_world()->get_food_item(i)->visible = true;
        }
        else if(std::get<0>(result) == 0){ //nothing near, paints food pink
            food_markers.markers[i].color.a = 1;
            food_markers.markers[i].color.r = 1.0;
            food_markers.markers[i].color.g = 0.75;
            food_markers.markers[i].color.b = 0.79;
            if(ft->get_world()->get_food_item(i)->robot1 > -1){
                simple_drone_markers.markers[ft->get_world()->get_food_item(i)->robot1].color.r = 0.0;
                simple_drone_markers.markers[ft->get_world()->get_food_item(i)->robot1].color.g = 0.0;
                simple_drone_markers.markers[ft->get_world()->get_food_item(i)->robot1].color.b = 0.0;
                ft->get_world()->get_food_item(i)->robot1 = -1;
            }
            if(ft->get_world()->get_food_item(i)->robot2 > -1){
                simple_drone_markers.markers[ft->get_world()->get_food_item(i)->robot2].color.r = 0.0;
                simple_drone_markers.markers[ft->get_world()->get_food_item(i)->robot2].color.g = 0.0;
                simple_drone_markers.markers[ft->get_world()->get_food_item(i)->robot2].color.b = 0.0;
                ft->get_world()->get_food_item(i)->robot2 = -1;
            }
            ft->get_world()->get_food_item(i)->visible = true;
            
        }
        
        // Arrived at base, drops food
        else if(ft->get_world()->get_food_item(i)->visible == false &&
        inside_base(simple_drone_markers.markers[ft->get_world()->get_food_item(i)->robot1].pose.position.x, simple_drone_markers.markers[ft->get_world()->get_food_item(i)->robot1].pose.position.y) &&
        inside_base(simple_drone_markers.markers[ft->get_world()->get_food_item(i)->robot2].pose.position.x, simple_drone_markers.markers[ft->get_world()->get_food_item(i)->robot2].pose.position.y)){
            ROS_INFO("BHEEED elseif2");
            food_markers.markers[i].color.a = 1;
            food_markers.markers[i].pose.position.x = simple_drone_markers.markers[std::get<1>(result)].pose.position.x;
            food_markers.markers[i].pose.position.y = simple_drone_markers.markers[std::get<1>(result)].pose.position.y;
            ft->get_world()->get_food_item(i)->visible = true;
            ft->get_world()->get_food_item(i)->robot1 = -1;
            ft->get_world()->get_food_item(i)->robot2 = -1;
            simple_drone_markers.markers[std::get<2>(result)].color.r = 0.0;
            simple_drone_markers.markers[std::get<2>(result)].color.g = 0.0;
            simple_drone_markers.markers[std::get<2>(result)].color.b = 0.0;
            simple_drone_markers.markers[std::get<1>(result)].color.r = 0.0;
            simple_drone_markers.markers[std::get<1>(result)].color.g = 0.0;
            simple_drone_markers.markers[std::get<1>(result)].color.b = 0.0;
            
        }

        //object collided while holding food
        else if((!ft->get_world()->get_simple_drone(std::get<1>(result))->is_visible() && ft->get_world()->get_food_item(i)->robot1 == std::get<1>(result)) || 
        (!ft->get_world()->get_simple_drone(std::get<2>(result))->is_visible() && ft->get_world()->get_food_item(i)->robot2 == std::get<2>(result))){
            ROS_INFO("BHEEED elseif3");
            food_markers.markers[i].color.a = 1;
            ft->get_world()->get_food_item(i)->visible = true;
            ft->get_world()->get_food_item(i)->robot1 = -1;
            ft->get_world()->get_food_item(i)->robot2 = -1;
            if(ft->get_world()->get_simple_drone(std::get<1>(result))->is_visible()){
                simple_drone_markers.markers[std::get<1>(result)].color.r = 1.0;
                simple_drone_markers.markers[std::get<1>(result)].color.g = 0.0;
                simple_drone_markers.markers[std::get<1>(result)].color.b = 0.0;
            }
            if(ft->get_world()->get_simple_drone(std::get<2>(result))->is_visible()){
                simple_drone_markers.markers[std::get<2>(result)].color.r = 1.0;
                simple_drone_markers.markers[std::get<2>(result)].color.g = 0.0;
                simple_drone_markers.markers[std::get<2>(result)].color.b = 0.0;
            }
        }
        
    }  

// Simple drone markers
simple_drone_markers_publisher.publish(simple_drone_markers);

// Simple drone wings markers
simple_drone_supports_markers_publisher.publish(simple_drone_supports_markers);

// Walls
wall_markers_publisher.publish(wall_markers);

    // Food Markers
food_markers_publisher.publish(food_markers);
   
}
void publish_data(){
    tf::Transform transform;
    tf::Quaternion q;
    
    ros::Time current_time = ros::Time::now();
    static tf::TransformBroadcaster tf_br_simple_drone;
    static tf::TransformBroadcaster tf_br_map;

    // World
    transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
    q.setRPY(0,0,0);
    transform.setRotation(q);
    tf_br_map.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "world"));

    
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom0";

    //send the position
    odom.pose.pose.position.x = ft->get_world()->get_simple_drone(0)->x();
    odom.pose.pose.position.y = ft->get_world()->get_simple_drone(0)->y();
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, 0.0);
    odom_publisher0.publish(odom);

    odom.pose.pose.position.x = ft->get_world()->get_simple_drone(1)->x();
    odom.pose.pose.position.y = ft->get_world()->get_simple_drone(1)->y();
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, 0.0);
    odom_publisher1.publish(odom);

   /* sensor_msgs::LaserScan scan;
    scan.header.stamp = scan_time;
    scan.header.frame_id = "laser_frame";
    scan.angle_min = -1.57;
    scan.angle_max = 1.57;
    scan.angle_increment = 3.14 / num_readings;
    scan.time_increment = (1 / laser_frequency) / (num_readings);
    scan.range_min = 0.0;
    scan.range_max = 100.0;
    laser_publisher0.publish(scan);*/

  
}
bool reset_arena(fast_turtle::ResetArena::Request &req, fast_turtle::ResetArena::Response& res)
{
    ft->reset_robots();
    ROS_INFO("The arena was reset");
    for(int i = 0; i < ft->get_world()->get_n_simple_drones(); i++)
    {
        cmd_vels_simple_drones[i].vx = 0.0;
        cmd_vels_simple_drones[i].vy = 0.0;
    }
    return true;
}
int main(int argc, char** argv)
{
    // Init node
    ros::init(argc, argv, "arena_simple");
    ROS_INFO("Initializing arena_simple_food");

    // Node object
    ros::NodeHandle nh;
    

    // Initialize markers
    world_marker_publisher = nh.advertise<visualization_msgs::Marker>("world_marker", 0);
    wall_markers_publisher = nh.advertise<visualization_msgs::MarkerArray>("wall_markers",0);
    simple_drone_markers_publisher = nh.advertise<visualization_msgs::MarkerArray>("simple_drone_markers",0);
    simple_drone_supports_markers_publisher = nh.advertise<visualization_msgs::MarkerArray>("simple_drone_supports_markers", 0);
    food_markers_publisher = nh.advertise<visualization_msgs::MarkerArray>("food_markers",0);
    // Subscribers for all robots
    ros::Subscriber sub_sd0 = nh.subscribe("cmd_vel_sd0", 1000, listen_cmd_vel_sd0);
    ros::Subscriber sub_sd1 = nh.subscribe("cmd_vel_sd1", 1000, listen_cmd_vel_sd1);

    // Publishers
    //simple_drones_publisher = nh.advertise<fast_turtle::RobotDataArray>("simple_drones", 1000);
    odom_publisher0 = nh.advertise<nav_msgs::Odometry>("odom0", 50);
    odom_publisher1 = nh.advertise<nav_msgs::Odometry>("odom1", 50);
    // Services
    ros::ServiceServer service = nh.advertiseService("reset_arena", reset_arena);
    // Initialize simulator object
    ft->init_world(20, 0, 0, "square");
    
    
    // Paredes da bounding box
    ft->add_wall(10, 10, 10, -10); //direita
    ft->add_wall(10, -10, -10, -10); //baixo
    ft->add_wall(-10, -10, -10, 10); //esquerda
    ft->add_wall(-10, 10, 10, 10); //cima

    // Teams of Drones
    // Drones #1
    ft->add_simple_drone(0.0, 0.0, 0.5, BURGER_RADIUS, "drone0", 0.01);
    ft->add_simple_drone(1.0, 0.0, 0.5, BURGER_RADIUS, "drone1", 0.01);

    // comida
    ft->add_food_item(1.0, 1.0, 0.1);
    
    // Send first world data and graphics data
    publish_data();
    init_graphics_and_data();

    // Frames per second
    ros::Rate loop_rate(SIMULATION_FPS);

    clock_t start = clock();
    double time_elapsed = 0;

    // Main cycle
    while(ros::ok()){
        /*
        // Updates logic
        update_physics();

        clock_t end = clock();
        time_elapsed = ((double) (end - start)) / CLOCKS_PER_SEC;
        if(time_elapsed>=LOOP_PERIOD){
            publish_data();
            start = clock();
        } 
        */
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