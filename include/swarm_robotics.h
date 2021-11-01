#ifndef SWARM_ROBOTICS_H
#define SWARM_ROBOTICS_H
#include "robot.h"
#include <map>
#include <array>
#include <memory>

const int ROBOTS_PER_TEAM = 5;
const int NUM_TEAMS = 8;

class SwarmTeam // a swarm team is a specific kind of swarm
{
    public:
        SwarmTeam(int id);
        int get_id();
        unsigned int get_num_alive();
        double get_lifetime();
        int get_foods_collected();
        bool has_started();
        void enlist(std::string robot_name);
        void the_robot_lost(std::string robot_name);
        void food_was_captured(std::string robot_name);
        bool is_robot_enlisted(std::string name);
        void start_time(std::string robot_name);
        void increase_lifetime(double step);
    private:
        int id_;
        unsigned int num_alive_;
        double lifetime_; // seconds
        int foods_collected_; // max_foods
        bool started_; // started competition
        std::map<std::string, bool> robots_;
};

class SwarmCompetition // has swarm teams
{
    public:
        SwarmCompetition(unsigned int simulation_fps);
        void init(std::vector<std::string> robot_names);
        void enlist(std::string robot_name, int team_id);
        void the_robot_lost(std::string robot_name);
        void food_was_captured(std::string robot_name);
        void start_time(std::string robot_name);
        void step();
        std::vector<std::shared_ptr<SwarmTeam>> get_teams();
        std::shared_ptr<SwarmTeam> get_team(std::string robot_name);
        std::string log();
        std::string log_lifetimes();
    private:
        bool is_robot_enlisted(std::string name);
        std::map<std::string, int> robot_list_; 
        std::vector<std::shared_ptr<SwarmTeam>> teams_;
        float simulation_dt_;
};

#endif
