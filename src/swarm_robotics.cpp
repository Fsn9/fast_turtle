#include "swarm_robotics.h"

SwarmTeam::SwarmTeam(int id)
{
    id_ = id;
    num_alive_ = ROBOTS_PER_TEAM;
    lifetime_ = 0;
    foods_collected_ = 0;
    started_ = false;
}

SwarmCompetition::SwarmCompetition(unsigned int simulation_fps)
{
    simulation_dt_ = 1.0 / simulation_fps;
    for(int i = 0; i < NUM_TEAMS; i++)
    {
        std::shared_ptr<SwarmTeam> st = std::make_shared<SwarmTeam>(i);
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
            (*teams_[team_id]).enlist(robot_name);
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
    for(std::shared_ptr<SwarmTeam> team : teams_)
    {
        if((*team).is_robot_enlisted(robot_name))
        {   
            (*team).the_robot_lost(robot_name);
            return;
        }
    }
}

void SwarmTeam::the_robot_lost(std::string robot_name)
{
    std::map<std::string, bool>::iterator it = robots_.find(robot_name);
    if(it != robots_.end())
    {
        it->second = false;
        --num_alive_;
    }
}

void SwarmCompetition::food_was_captured(std::string robot_name)
{
    for(std::shared_ptr<SwarmTeam> team : teams_)
    {
        if((*team).is_robot_enlisted(robot_name))
        {
            (*team).food_was_captured(robot_name);
            return;
        }
    }
}

void SwarmTeam::food_was_captured(std::string robot_name)
{
    ++foods_collected_;
}

void SwarmCompetition::start_time(std::string robot_name)
{
    for(std::shared_ptr<SwarmTeam> team : teams_)
    {
        if((*team).is_robot_enlisted(robot_name))
        {
            (*team).start_time(robot_name);
            return;
        }
    }
}

void SwarmTeam::start_time(std::string robot_name)
{
    if(!started_)
    {
        started_ = true;
    }
}

void SwarmTeam::increase_lifetime(double step)
{
    lifetime_ += step;
}

void SwarmCompetition::step()
{
    for(std::shared_ptr<SwarmTeam> team : teams_)
    {
        if((*team).has_started())
        {
            (*team).increase_lifetime(simulation_dt_);
        }
    }
}

std::shared_ptr<SwarmTeam> SwarmCompetition::get_team(std::string robot_name)
{
    int team_id;
    if(is_robot_enlisted(robot_name))
    {
        team_id = robot_list_.find(robot_name)->second;    
    }
    return teams_[team_id];
}

std::vector<std::shared_ptr<SwarmTeam>> SwarmCompetition::get_teams()
{
    return teams_;
}

std::map<std::string, int> SwarmCompetition::get_robot_list()
{
    return robot_list_;
}

std::map<std::string, bool> SwarmTeam::get_robots_state()
{
    return robots_;
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

std::string SwarmCompetition::log()
{
    std::ostringstream oss;
    oss << "Swarm Competition log\n";
    oss << "NUM TEAMS: " << NUM_TEAMS << "\n";
    oss << "NUM ROBOTS PER TEAM" << ROBOTS_PER_TEAM << "\n";
    oss << "Teams: \n";
    for(std::shared_ptr<SwarmTeam> team : teams_)
    {
        oss << "\t# Team " << team->get_id() << "\n";
        oss << "\tFoods collected: " << team->get_foods_collected() << "\n";
        oss << "\tLifetime: " << team->get_lifetime() << "\n";
        oss << "\tNumber of robots alive: "<< team->get_num_alive() << "\n";
        oss << "\tHas started: " << team->has_started() << "\n";
    }
    return oss.str();
}

std::string SwarmCompetition::log_lifetimes()
{
    std::ostringstream oss;
    oss << "Swarm Competition lifetimes log\n";
    for(std::shared_ptr<SwarmTeam> team : teams_)
    {
        oss << "\t# Team " << team->get_id() << "\n";
        oss << "\tLifetime: " << team->get_lifetime() << "\n";
    }
    return oss.str();
}
