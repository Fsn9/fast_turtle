#include "ros/ros.h"
#include "fast_turtle.h"

class Controller{
    public:
        Controller();

        std::vector<float> decide(int obs){
            std::vector<float> cmds{0,0};
            if (obs > 0){
                cmds[0] = 0.1;
                cmds[1] = 0;
            }
            else{
                cmds[0] = -0.1;
                cmds[1] = 0;
            }
            return cmds;
        }
};

int main(int argc, char **argv)
{
    // Init node
    ros::init(argc, argv, "controller_training");

    // Node object
    ros::NodeHandle nh;

    int count = 0;
    int obs;
    std::vector<float> cmds;
    FastTurtle ft(&nh);
    Controller c;

    while(ros::ok()){
        std::cout << "Count:" << count << "\n";
        obs = ft.observe();
        std::cout << "obs:" << obs << "\n";
        cmds = c.decide(obs);
        std::cout << "cmds: (" << cmds[0] << "," << cmds[1] << ")\n";
        obs = ft.act(cmds[0], cmds[1]);
        std::cout << "obs':" << obs << "\n";
        if (count == 5) break;
        count += 1;
    }
}