#ifndef SWARM_ROBOTICS_H
#define SWARM_ROBOTICS_H
#include "robot.h"
#include <map>

class SwarmRobot // a robot belonging to a swarm. it could be a leader
{
    public:
        std::tuple<float, float> get_nearest_drone(); // return x,y nearest drone
        std::tuple<float, float> get_nearest_obstacle(); // return x,y nearest obstacle/wall
    protected:
        bool leader;
};

class Swarm // has robot names and their interconnection
{
    public:
        void add_robot(SwarmRobot sd);
        std::vector<SwarmRobot> get_robots();
    protected:
        std::vector<SwarmRobot> robots_;
};

class SwarmTeam : public Swarm // a swarm team is a specific kind of swarm
{
    public:
        SwarmTeam(int id);
        int get_id();
        unsigned int get_num_alive();
        double get_lifetime();
        int get_foods_collected();
        bool has_started();
    private:
        int id_;
        unsigned int num_alive_;
        double lifetime_; // seconds
        int foods_collected_; // max_foods
        bool started_; // started competition
};

class SwarmCompetition // has swarm teams
{
    public:
        SwarmCompetition();
        void init(std::vector<std::string> robot_names);
        void add_robot(SwarmRobot robot, int team_id);
        std::vector<SwarmTeam> get_teams();
        void enlist(std::string robot_name, int team_id);
    private:
        std::map<std::string, int> robot_list_; 
        std::vector<SwarmTeam> teams_;
};

#endif
