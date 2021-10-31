#include "swarm_robotics.h"

const int ROBOTS_PER_TEAM = 5;
const int NUM_TEAMS = 8;

SwarmTeam::SwarmTeam(int id)
{
    id_ = id;
    num_alive_ = ROBOTS_PER_TEAM;
    lifetime_ = 0;
    foods_collected_ = 0;
    started_ = false;
}

SwarmCompetition::SwarmCompetition()
{
    teams_.reserve(NUM_TEAMS);
    std::vector<SwarmTeam>::iterator it;
    int i = 0;
    for(it = teams_.begin(); it != teams_.end(); ++it)
    {
        SwarmTeam st(i);
        teams_.insert(it, st);
        ++i;
    }
}

void SwarmCompetition::init(std::vector<std::string> robot_names)
{
    for(std::string name : robot_names)
    {
        robot_list_.insert({name,-1});
    }
}

void SwarmCompetition::enlist(std::string robot_name, int team_id)
{

}

void SwarmCompetition::add_robot(SwarmRobot robot, int team_id)
{
    if(team_id > 0 && team_id < NUM_TEAMS)
    {
        teams_[team_id].add_robot(robot);
    }
    else
    {
        throw std::out_of_range("Invalid team id. Needs to be between 0 and "+std::to_string(NUM_TEAMS)+"\n");
    }
    
}

void Swarm::add_robot(SwarmRobot sd)
{
    robots_.emplace_back(sd);
}

std::vector<SwarmRobot> Swarm::get_robots()
{
    return robots_;
}

std::vector<SwarmTeam> SwarmCompetition::get_teams()
{
    return teams_;
}

int SwarmTeam::get_id()
{
    return id_;
}

unsigned int SwarmTeam::get_num_alive()
{
    return num_alive_;
}

double SwarmTeam::get_lifetime()
{
    return lifetime_;
}

int SwarmTeam::get_foods_collected()
{
    return foods_collected_;
}

bool SwarmTeam::has_started()
{
    return started_;
}



