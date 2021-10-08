#include "controller.h"

class TBB1_Controller : public TurtlebotBurgerController
{
    public:
        TBB1_Controller(int robot_idx, std::string topic_name) : TurtlebotBurgerController(robot_idx, topic_name) {}
        void decide()
        {
            // Get real time data
            robot_data data = get_robot_data();
            double pos_x = data.pos_x_;
            double pos_y = data.pos_x_;
            double quat_x = data.quat_x_;
            double quat_y = data.quat_y_;
            double quat_z = data.quat_z_;
            double quat_w = data.quat_w_;
            std::vector<float> laser_scan = data.laser_scan_;

            // Save commands to be published on publish() call
            double v = 0.22;
            double w = 0.2;
            update_commands(v, w);
        }
};

class SD0_Controller : public SimpleDroneController
{
    public:
        SD0_Controller(int robot_idx, std::string topic_name) : SimpleDroneController(robot_idx, topic_name) {}
        void decide()
        {
            // Get real time data
            robot_data data = get_robot_data();
            double pos_x = data.pos_x_;
            double pos_y = data.pos_x_;
            double quat_x = data.quat_x_;
            double quat_y = data.quat_y_;
            double quat_z = data.quat_z_;
            double quat_w = data.quat_w_;
            std::vector<float> laser_scan = data.laser_scan_;

            // Save commands to be published on publish() call
            double vx = 0.22;
            double vy = 0.2;
            update_commands(vx, vy);
        }
};


int main(int argc, char **argv)
{
    // Init node
    ros::init(argc, argv, "my_controller");

    // Define my controller
    int tbb_idx = 1;
    int sd_idx = 0;
    TBB1_Controller tbb1_controller(tbb_idx, "cmd_vel_tbb1");
    SD0_Controller sd0_controller(sd_idx, "cmd_vel_sd0");

    // Publish at a loop_rate (in hz)
    int loop_rate = 3; 
    ros::Rate rate(loop_rate);    

    // Infinite cycle to read and publish commands
    while(ros::ok())
    {
        tbb1_controller.decide();
        sd0_controller.decide();
        tbb1_controller.publish();
        sd0_controller.publish();
        rate.sleep();
        ros::spinOnce();
    }

    return 0;
}