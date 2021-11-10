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
#include <string.h>

// #include <time.h>

// Simulation frames per second
#define SIMULATION_FPS 40u

// Values for food distances
#define FOOD_LIMIT_X_sup -6
#define FOOD_LIMIT_X_inf -10
#define FOOD_LIMIT_Y_sup -6
#define FOOD_LIMIT_Y_inf -10
#define PROXIMITY 0.3
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



// Drone structure parameters
#define SIMPLE_DRONE_SUPPORTS_THICKNESS 0.01f

float colors[10][3]={RED,GREEN,BLUE,YELLOW,ORANGE,WHITE,GREY,PINK,CYAN,PURPLE};

struct drone {
    int team;
    float rgb[3];
};

// // Teams
// struct team Team;


// Initialize fast turtle simulator object
FastTurtle* ft = new FastTurtle(SIMULATION_FPS);

// Markers
visualization_msgs::Marker world_marker;
visualization_msgs::MarkerArray obstacle_markers;
visualization_msgs::MarkerArray wall_markers;
visualization_msgs::MarkerArray simple_drone_markers;
visualization_msgs::MarkerArray simple_drone_supports_markers;
visualization_msgs::MarkerArray food_markers;
//visualization_msgs::Marker stats_marker;                            // stats
visualization_msgs::MarkerArray stats_markers;

// Marker Publishers
ros::Publisher world_marker_publisher;
ros::Publisher obstacle_markers_publisher;
ros::Publisher wall_markers_publisher;
ros::Publisher simple_drone_markers_publisher;
ros::Publisher food_markers_publisher; 
ros::Publisher simple_drone_supports_markers_publisher;
ros::Publisher stats_markers_publisher;                              // stats


// Publishers
ros::Publisher simple_drones_publisher;

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
        // int dist;
        // int pos;
        // for (int h = 0; h < 9; h++){
        //     dist = std::find(std::begin(Team[h].drones), std::end(Team[h].drones), i);
        //     if (dist != std::end(Team[h].drones)) {
        //         pos = std::distance(Team[h].drones, dist);
        //         break;
        //     } 
        // }
        
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

    //visualization_msgs::MarkerArray stats_markers;
    visualization_msgs::Marker stats_marker;
    //for (i=0; i!=1 ;i++){
        
        stats_marker.header.frame_id = "world";
        stats_marker.ns = "simulation_markers";
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
        stats_markers.markers.push_back(stats_marker);
        j+=1;
        
    //}
    
    for(i = 1; i < ft->get_world()->get_n_simple_drones()+1; i++)
    {
        stats_marker.header.frame_id = "world";
        stats_marker.ns = "simulation_markers";
        stats_marker.id = j;
        stats_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        stats_marker.action = visualization_msgs::Marker::ADD;
        stats_marker.pose.position.x = 2.0; //ft->get_world()->get_simple_drone(i-1)->get_xc();
        stats_marker.pose.position.y = 2.0; //ft->get_world()->get_simple_drone(i-1)->get_yc();
        stats_marker.pose.position.z = 2.0; //ft->get_world()->get_simple_drone(i-1)->get_height() + simple_drone_marker.scale.z * 0.5;
        stats_marker.scale.z = 0.3;
        stats_marker.color.a = 1.0; 
        stats_marker.color.r = 1.0;
        stats_marker.color.g = 1.0;
        stats_marker.color.b = 1.0;
        stats_marker.text = "Stats: \n TESTE \n DE TEXTO QUE \n SEGUE O DRONE";
        stats_markers.markers.push_back(stats_marker);
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
    obstacle_markers_publisher.publish(obstacle_markers);
    wall_markers_publisher.publish(wall_markers);
    simple_drone_supports_markers_publisher.publish(simple_drone_supports_markers);
    //stats_markers_publisher.publish(stats_marker);
    stats_markers_publisher.publish(stats_markers);

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
        stats_markers.markers[i+1].pose.position.x = ft->get_world()->get_simple_drone(i)->x();
        stats_markers.markers[i+1].pose.position.y = ft->get_world()->get_simple_drone(i)->y();
    }

    //check for collisions and if they collided, make them disapear 
    ft->check_collisions();
    for(int i = 0; i < ft->get_world()->get_n_simple_drones(); i++){
        if(!ft->get_world()->get_simple_drone(i)->is_visible()){
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
            else if(!ft->get_world()->get_simple_drone(j)->is_visible() && ft->get_world()->get_food_item(i)->robot == j){
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
    //stats_markers_publisher.publish(stats_marker);
    stats_markers_publisher.publish(stats_markers);
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
    ros::init(argc, argv, "arena_simple");
    ROS_INFO("Initializing arena_simple");

    // Node object
    ros::NodeHandle nh;

    // Initialize markers
    world_marker_publisher = nh.advertise<visualization_msgs::Marker>("world_marker", 0);
    food_markers_publisher = nh.advertise<visualization_msgs::MarkerArray>("food_markers",0);
    obstacle_markers_publisher = nh.advertise<visualization_msgs::MarkerArray>("obstacle_markers",0);
    wall_markers_publisher = nh.advertise<visualization_msgs::MarkerArray>("wall_markers",0);
    //stats_markers_publisher = nh.advertise<visualization_msgs::Marker>("stats_marker", 0);
    stats_markers_publisher = nh.advertise<visualization_msgs::MarkerArray>("stats_markers", 0);
    simple_drone_markers_publisher = nh.advertise<visualization_msgs::MarkerArray>("simple_drone_markers",0);
    simple_drone_supports_markers_publisher = nh.advertise<visualization_msgs::MarkerArray>("simple_drone_supports_markers",0);

    // Subscribers for all robots
    ros::Subscriber sub_sd0 = nh.subscribe("cmd_vel_sd0", 1000, listen_cmd_vel_sd0);
    ros::Subscriber sub_sd1 = nh.subscribe("cmd_vel_sd1", 1000, listen_cmd_vel_sd1);
    ros::Subscriber sub_sd2 = nh.subscribe("cmd_vel_sd2", 1000, listen_cmd_vel_sd2);

    // ros::Subscriber sub_team1 = nh.subscribe("cmd_vel_team1", 1000, listen_cmd_vel_team1);
    // ros::Subscriber sub_team2 = nh.subscribe("cmd_vel_team2", 1000, listen_cmd_vel_team2);
    // ros::Subscriber sub_team3 = nh.subscribe("cmd_vel_team3", 1000, listen_cmd_vel_team3);
    // ros::Subscriber sub_team4 = nh.subscribe("cmd_vel_team4", 1000, listen_cmd_vel_team4);
    // ros::Subscriber sub_team5 = nh.subscribe("cmd_vel_team5", 1000, listen_cmd_vel_team5);
    // ros::Subscriber sub_team6 = nh.subscribe("cmd_vel_team6", 1000, listen_cmd_vel_team6);
    // ros::Subscriber sub_team7 = nh.subscribe("cmd_vel_team7", 1000, listen_cmd_vel_team7);
    // ros::Subscriber sub_team8 = nh.subscribe("cmd_vel_team8", 1000, listen_cmd_vel_team8);
    // ros::Subscriber sub_team9 = nh.subscribe("cmd_vel_team9", 1000, listen_cmd_vel_team9);
    // ros::Subscriber sub_team10 = nh.subscribe("cmd_vel_team10", 1000, listen_cmd_vel_team10);

    // Publishers
    simple_drones_publisher = nh.advertise<fast_turtle::RobotDataArray>("simple_drones", 1000);

    // Initialize simulator object
    float obstacle_radius = 0.15;
    ft->init_world(20, 0, 0, "square");
    
    //obstacle
    ft->add_obstacle(0, 0, obstacle_radius, "round");
    
    // Paredes da bounding box
    ft->add_wall(10, 10, 10, -10); //direita
    ft->add_wall(10, -10, -10, -10); //baixo
    ft->add_wall(-10, -10, -10, 10); //esquerda
    ft->add_wall(-10, 10, 10, 10); //cima

    // Teams of Drones
    // Drones #1
    ft->add_simple_drone(-0.5, -9.5, 0.5, BURGER_RADIUS, "drone0", 0.2);
    ft->add_simple_drone(-0.5, -9.0, 0.5, BURGER_RADIUS, "drone1", 0.2);
    ft->add_simple_drone(-0.5, -8.5, 0.5, BURGER_RADIUS, "drone2", 0.2); 
    ft->add_simple_drone(-0.5, -8.0, 0.5, BURGER_RADIUS, "drone3", 0.2); 

    // Drones #2
    ft->add_simple_drone(0.0, -9.5, 0.5, BURGER_RADIUS, "drone4", 0.2);
    ft->add_simple_drone(0.0, -9.0, 0.5, BURGER_RADIUS, "drone5", 0.2);
    ft->add_simple_drone(0.0, -8.5, 0.5, BURGER_RADIUS, "drone6", 0.2); 
    ft->add_simple_drone(0.0, -8.0, 0.5, BURGER_RADIUS, "drone7", 0.2); 

    // Drones #3
    ft->add_simple_drone(0.5, -9.5, 0.5, BURGER_RADIUS, "drone8", 0.2);
    ft->add_simple_drone(0.5, -9.0, 0.5, BURGER_RADIUS, "drone9", 0.2);
    ft->add_simple_drone(0.5, -8.5, 0.5, BURGER_RADIUS, "drone10", 0.2); 
    ft->add_simple_drone(0.5, -8.0, 0.5, BURGER_RADIUS, "drone11", 0.2); 

    // Drones #4
    ft->add_simple_drone(1.0, -9.5, 0.5, BURGER_RADIUS, "drone12", 0.2);
    ft->add_simple_drone(1.0, -9.0, 0.5, BURGER_RADIUS, "drone13", 0.2);
    ft->add_simple_drone(1.0, -8.5, 0.5, BURGER_RADIUS, "drone14", 0.2); 
    ft->add_simple_drone(1.0, -8.0, 0.5, BURGER_RADIUS, "drone15", 0.2); 

    // Drones #5
    ft->add_simple_drone(1.5, -9.5, 0.5, BURGER_RADIUS, "drone16", 0.2);
    ft->add_simple_drone(1.5, -9.0, 0.5, BURGER_RADIUS, "drone17", 0.2);
    ft->add_simple_drone(1.5, -8.5, 0.5, BURGER_RADIUS, "drone18", 0.2); 
    ft->add_simple_drone(1.5, -8.0, 0.5, BURGER_RADIUS, "drone19", 0.2); 

    // Drones #6
    ft->add_simple_drone(2.0, -9.5, 0.5, BURGER_RADIUS, "drone20", 0.2);
    ft->add_simple_drone(2.0, -9.0, 0.5, BURGER_RADIUS, "drone21", 0.2);
    ft->add_simple_drone(2.0, -8.5, 0.5, BURGER_RADIUS, "drone22", 0.2); 
    ft->add_simple_drone(2.0, -8.0, 0.5, BURGER_RADIUS, "drone23", 0.2); 

    // Drones #7
    ft->add_simple_drone(2.5, -9.5, 0.5, BURGER_RADIUS, "drone24", 0.2);
    ft->add_simple_drone(2.5, -9.0, 0.5, BURGER_RADIUS, "drone25", 0.2);
    ft->add_simple_drone(2.5, -8.5, 0.5, BURGER_RADIUS, "drone26", 0.2); 
    ft->add_simple_drone(2.5, -8.0, 0.5, BURGER_RADIUS, "drone27", 0.2); 

    // Drones #8
    ft->add_simple_drone(3.0, -9.5, 0.5, BURGER_RADIUS, "drone28", 0.2);
    ft->add_simple_drone(3.0, -9.0, 0.5, BURGER_RADIUS, "drone29", 0.2);
    ft->add_simple_drone(3.0, -8.5, 0.5, BURGER_RADIUS, "drone30", 0.2); 
    ft->add_simple_drone(3.0, -8.0, 0.5, BURGER_RADIUS, "drone31", 0.2); 

    // Drones #9
    ft->add_simple_drone(3.5, -9.5, 0.5, BURGER_RADIUS, "drone32", 0.2);
    ft->add_simple_drone(3.5, -9.0, 0.5, BURGER_RADIUS, "drone33", 0.2);
    ft->add_simple_drone(3.5, -8.5, 0.5, BURGER_RADIUS, "drone34", 0.2); 
    ft->add_simple_drone(3.5, -8.0, 0.5, BURGER_RADIUS, "drone35", 0.2); 

    // Drones #10
    ft->add_simple_drone(4.0, -9.5, 0.5, BURGER_RADIUS, "drone36", 0.2);
    ft->add_simple_drone(4.0, -9.0, 0.5, BURGER_RADIUS, "drone37", 0.2);
    ft->add_simple_drone(4.0, -8.5, 0.5, BURGER_RADIUS, "drone38", 0.2); 
    ft->add_simple_drone(4.0, -8.0, 0.5, BURGER_RADIUS, "drone39", 0.2); 

    // Send first world data and graphics data
    publish_data();
    init_graphics_and_data();

    // Frames per second
    ros::Rate loop_rate(SIMULATION_FPS);

    //clock_t time;

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