#ifndef GLOBAL_PLANNER_H
#define GLOBAL_PLANNER_H

#include <iostream>
#include <math.h>
#include <vector>
#include <cfloat>
#include <unordered_map>
#include <set>

using namespace std;

struct Global_State
{
    double x; 
    double y; 
    double theta;
    Global_State(): x(0), y(0), theta(0)
    {}
    Global_State(double a, double b, double c): x(a), y(b), theta(c)
    {}
    Global_State(const Global_State& h)
    {
        this->x = h.x;
        this->y = h.y;
        this->theta = h.theta;
    }
    bool operator==(const Global_State& t) const
    {
        return (this->x==t.x && this->y==t.y && this->theta==t.theta);
    }
};

struct f_COORDINATE
{
    double h;
    Global_State st;
    f_COORDINATE(): h(0), st()
    {}
    f_COORDINATE(double a, Global_State b): h(a), st(b)
    {}
    bool operator==(const f_COORDINATE& t) const
    {
        return (this->st == t.st);
    }
    bool operator<(const f_COORDINATE& t) const
    {
        return (this->h < t.h);
    }
};

class GlobalPlanner
{
    public:
        
        Global_State m_start_state;
        Global_State m_goal_state;
        struct Graph_Node
        {
            Global_State state;
            int parent;
            double f = DBL_MAX, g = DBL_MAX, h = DBL_MAX; 
        };
        double m_max_steering_angle; // Max steering angle of vehicle
        double m_dt; // Time step for lattice graph
        double m_desired_velocity;
        double m_car_length;
        // typedef pair<double, Global_State> f_COORDINATE;
        unordered_map<int, Graph_Node> global_graph;
    
    public:
        GlobalPlanner(Global_State start_state, Global_State goal_state, double max_steering_angle, double dt,
         double desire_vel, double car_length);
        
        vector<Global_State> get_motion_primitive(Global_State current_state, double steering_angle);
        Global_State get_new_state(Global_State current_state, double str_angle);
        int get_state_hash(Global_State state);
        bool is_valid_primitive(vector<Global_State> motion_primitve);
        vector<Global_State> solutionPath(int goal);
        vector<Global_State> A_star(Global_State start_state, Global_State goal_state);
};
#endif