#include "swarm_robotics.h"

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
    for(int i = 0; i < NUM_TEAMS; i++)
    {
        SwarmTeam st(i);
        teams_.emplace_back(st);
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
    if(team_id >= 0 && team_id < NUM_TEAMS)
    {
        if(is_robot_enlisted(robot_name))
        {
            std::map<std::string, int>::iterator it = robot_list_.find(robot_name);
            it->second = team_id;
            teams_[team_id].enlist(robot_name);
        }
        else
        {
            throw std::invalid_argument("Invalid robot name '" + robot_name 
            + "'. It is not equal to any robot name in the fast turtle world\n");    
        }
    }
    else
    {
        throw std::out_of_range("Invalid team id. Needs to be between 0 and " + std::to_string(NUM_TEAMS)+"\n");
    }
}

bool SwarmCompetition::is_robot_enlisted(std::string name)
{
    return robot_list_.find(name) != robot_list_.end();
}

bool SwarmTeam::is_robot_enlisted(std::string name)
{
    return robots_.find(name) != robots_.end();
}

void SwarmTeam::enlist(std::string robot_name)
{
    robots_.insert({robot_name, true});
}

void SwarmCompetition::the_robot_lost(std::string robot_name)
{
    for(SwarmTeam team : teams_)
    {
        if(team.is_robot_enlisted(robot_name))
        {   
            team.the_robot_lost(robot_name);
        }
    }
}

void SwarmTeam::the_robot_lost(std::string robot_name)
{
    std::map<std::string, bool>::iterator it = robots_.find(robot_name);
    if(it != robots_.end())
    {
        it->second = false;
    }
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

/*
void Swarm::add_robot(SwarmRobot sd)
{
    robots_.emplace_back(sd);
}

std::vector<SwarmRobot> Swarm::get_robots()
{
    return robots_;
}
*/


