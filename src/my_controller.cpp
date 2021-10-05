#include "controller.h"

class MyController : public Controller
{
    public:
        MyController(int robot_idx) : Controller(robot_idx) {}
        cmd_vel_tbb decide_commands()
        {
            // Get position and orientation
            double pos_x = robot_data_.pos_x;
            double pos_y = robot_data_.pos_x;
            double quat_x = robot_data_.quat_x;
            double quat_y = robot_data_.quat_y;
            double quat_z = robot_data_.quat_z;
            double quat_w = robot_data_.quat_w;
            std::vector<float> laser_scan = robot_laser_scan_;

            // Decide commands
            double v = 0.22;
            double w = 0.2;
            return cmd_vel_tbb{v,w};
        }
};

int main(int argc, char **argv)
{
    // Init node
    ros::init(argc, argv, "my_controller");

    // Define my controller
    int robot_idx = 4;
    MyController my_controller(robot_idx);

    // Commands
    cmd_vel_tbb commands;

    // Publish at a loop_rate
    int loop_rate = 3; // hz
    ros::Rate rate(loop_rate);    
    // Infinite cycle to read and publish commands
    while(ros::ok())
    {
        commands = my_controller.decide_commands();
        my_controller.publish_commands(commands.v, commands.w);
        rate.sleep();
        ros::spinOnce();
    }

    return 0;
}