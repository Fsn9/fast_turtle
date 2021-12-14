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
        if(teams_.find(team_id) == teams_.end())
        {
            teams_[team_id] = std::make_shared<SwarmTeam>(team_id);
        }
        if(is_robot_enlisted(robot_name))
        {
            std::map<std::string, int>::iterator it = robot_list_.find(robot_name);
            it->second = team_id;
            if(teams_[team_id]->get_num_robots() != ROBOTS_PER_TEAM)
            {
                teams_[team_id]->enlist(robot_name);
            }
            else
            {
                throw std::length_error("Maximum number of robots per team is "
                + std::to_string(ROBOTS_PER_TEAM) + ". Robot "
                + robot_name + " was not added.");
            }      
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

SwarmTeam::RobotState::RobotState()
{
    alive_ = true;
    leader_ = false;
}

SwarmTeam::RobotState::RobotState(bool alive, bool leader)
{
    alive_ = alive;
    leader_ = leader;
}

bool SwarmTeam::RobotState::is_leader()
{
    return leader_;
}

bool SwarmTeam::RobotState::is_alive()
{
    return alive_;
}

void SwarmTeam::RobotState::kick_out()
{
    alive_ = false;
    if(leader_)
    {
        leader_ = false;
    }
}

void SwarmTeam::RobotState::declare_leader()
{
    leader_ = true;
}

std::string SwarmTeam::get_leader()
{
    for(std::pair<std::string, std::shared_ptr<RobotState>> r : robots_)
    {
        if(r.second->is_leader())
        {
            return r.first;
        }
    }
    return "none";
}

size_t SwarmTeam::get_num_robots()
{
    return robots_.size();
}

void SwarmTeam::enlist(std::string robot_name)
{
    std::shared_ptr<RobotState> rs = std::make_shared<RobotState>();
    if(robots_.empty())
    {
        rs->declare_leader();
    }
    robots_.insert({robot_name, rs});
}

void SwarmCompetition::the_robot_lost(std::string robot_name)
{
    for(std::map<int, std::shared_ptr<SwarmTeam>>::iterator team =
    teams_.begin(); team != teams_.end(); team++)
    {
        if(team->second->is_robot_enlisted(robot_name))
        {   
            team->second->the_robot_lost(robot_name);
            return;
        }
    }
}

void SwarmTeam::the_robot_lost(std::string robot_name)
{
    std::map<std::string, std::shared_ptr<RobotState>>::iterator it = robots_.find(robot_name);
    if(it != robots_.end())
    {
        it->second->kick_out();   
        pass_leadership();    
        --num_alive_;
    }
}

void SwarmTeam::pass_leadership()
{
    for(std::pair<std::string, std::shared_ptr<RobotState>> r : robots_)
    {
        if(r.second->is_alive() && !r.second->is_leader())
        {
            r.second->declare_leader();
            return;
        }
    }
    std::cout << "Team " << get_id() << " is dead\n";
}

void SwarmCompetition::food_was_captured(std::string robot_name)
{
    for(std::map<int, std::shared_ptr<SwarmTeam>>::iterator team =
    teams_.begin(); team != teams_.end(); team++)
    {
        if(team->second->is_robot_enlisted(robot_name))
        {
            team->second->food_was_captured(robot_name);
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
    for(std::map<int, std::shared_ptr<SwarmTeam>>::iterator team =
    teams_.begin(); team != teams_.end(); team++)
    {
        if(team->second->is_robot_enlisted(robot_name))
        {
            team->second->start_time(robot_name);
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
    for(std::map<int, std::shared_ptr<SwarmTeam>>::iterator team =
    teams_.begin(); team != teams_.end(); team++)
    {
        if(team->second->has_started())
        {
            team->second->increase_lifetime(simulation_dt_);
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

std::shared_ptr<SwarmTeam> SwarmCompetition::get_team(int id)
{
    if(id >= 0 && id < NUM_TEAMS)
    {
        for(std::map<int, std::shared_ptr<SwarmTeam>>::iterator team =
        teams_.begin(); team != teams_.end(); team++)
        {
            if(team->second->get_id() == id)
            {
                return team->second;
            }
        }
        return {};
    }
    else
    {
        throw std::out_of_range("Invalid team id. Needs to be between 0 and " + std::to_string(NUM_TEAMS)+"\n");
    }
}

std::map<int,std::shared_ptr<SwarmTeam>> SwarmCompetition::get_teams()
{
    return teams_;
}

int SwarmCompetition::get_team_id(std::string robot_name)
{
    for(std::map<int, std::shared_ptr<SwarmTeam>>::iterator team =
    teams_.begin(); team != teams_.end(); team++)
    {
        if(team->second->is_robot_enlisted(robot_name))
        {
            return team->second->get_id();
        }
    }
    return -1;
}

unsigned int SwarmCompetition::get_num_alive(int team_id)
{
    try
    {
        return get_team(team_id)->get_num_alive();
    }
    catch(const std::out_of_range& e)
    {
        throw std::out_of_range("Invalid team id. Needs to be between 0 and " + std::to_string(NUM_TEAMS)+"\n");
    }
}

double SwarmCompetition::get_lifetime(int team_id)
{
    try
    {
        return get_team(team_id)->get_lifetime();
    }
    catch(const std::out_of_range& e)
    {
        throw std::out_of_range("Invalid team id. Needs to be between 0 and " + std::to_string(NUM_TEAMS)+"\n");
    }
}

unsigned int SwarmCompetition::get_foods_collected(int team_id)
{
    try
    {
        return get_team(team_id)->get_foods_collected();
    }
    catch(const std::out_of_range& e)
    {
        throw std::out_of_range("Invalid team id. Needs to be between 0 and " + std::to_string(NUM_TEAMS)+"\n");
    }
}

bool SwarmCompetition::team_has_started(int team_id)
{
    try
    {
        return get_team(team_id)->has_started();
    }
    catch(const std::out_of_range& e)
    {
        throw std::out_of_range("Invalid team id. Needs to be between 0 and " + std::to_string(NUM_TEAMS)+"\n");
    }
}

std::string SwarmCompetition::get_leader(int team_id)
{
    try
    {
        return get_team(team_id)->get_leader();
    }
    catch(const std::out_of_range& e)
    {
        throw std::out_of_range("Invalid team id. Needs to be between 0 and " + std::to_string(NUM_TEAMS)+"\n");
    }
}

std::map<std::string, int> SwarmCompetition::get_robot_list()
{
    return robot_list_;
}

std::map<std::string, std::shared_ptr<SwarmTeam::RobotState>> SwarmTeam::get_robots_state()
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
    for(std::map<int, std::shared_ptr<SwarmTeam>>::iterator team =
    teams_.begin(); team != teams_.end(); team++)
    {
        oss << "\t# Team " << team->second->get_id() << "\n";
        oss << "\tFoods collected: " << team->second->get_foods_collected() << "\n";
        oss << "\tLifetime: " << team->second->get_lifetime() << "\n";
        oss << "\tNumber of robots alive: "<< team->second->get_num_alive() << "\n";
        oss << "\tHas started: " << team->second->has_started() << "\n";
        oss << "\tLeader: " << team->second->get_leader() << "\n";
    }
    return oss.str();
}

std::string SwarmCompetition::log_lifetimes()
{
    std::ostringstream oss;
    oss << "Swarm Competition lifetimes log\n";
    for(std::map<int, std::shared_ptr<SwarmTeam>>::iterator team =
    teams_.begin(); team != teams_.end(); team++)
    {
        oss << "\t# Team " << team->second->get_id() << "\n";
        oss << "\tLifetime: " << team->second->get_lifetime() << "\n";
    }
    return oss.str();
}
