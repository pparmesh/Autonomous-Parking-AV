#ifndef GLOBAL_PLANNER_H
#define GLOBAL_PLANNER_H

#include <iostream>
#include <math.h>
#include <vector>
#include <cfloat>
#include <unordered_map>
#include <set>
#include <Eigen/Dense>

#define num_steps 9
#define PI 3.141592654
#define mapX 920    // xlim [-62, 30]
#define mapY 800    // ylim [-40,  40]

using namespace std;
using namespace Eigen;


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


class MotionPrimitive
{
    private:
        vector<Global_State> primitives;
    public:
        MotionPrimitive()
        {}

        MotionPrimitive(const vector<Global_State>& p)
        {
            primitives.clear();
            for(Global_State g: p)
                this->primitives.push_back(g);
        }
        MotionPrimitive(const MotionPrimitive& p)
        {
            primitives.clear();
            vector<Global_State> h = p.primitives; 
            for(Global_State g : h)
                this->primitives.push_back(g);
        }

        void insert_state(Global_State st);
        
        void clear_primitives();
        
        vector<Global_State> get_primitive();

        Global_State next_state();

};

class GlobalPlanner
{
    public:
        
        Global_State start_state;
        Global_State goal_state;
        struct GNode
        {
            Global_State state;
            int parent;
            double f;
            double g;
            double h;
            GNode(): state(), parent(0), f(DBL_MAX), g(DBL_MAX), h(DBL_MAX)
            {}
            GNode(const Global_State& st, int p, double a, double b, double c): state(st), parent(p), f(a), g(b), h(c)
            {}
        };
        double max_steering_angle; // Max steering angle of vehicle
        double dt; // Time step for lattice graph
        double desired_velocity;
        double car_length;

        vector<MotionPrimitive> motion_primitives;

        Matrix<double, 3, num_steps*(27+22)> primitive_M;

        unordered_map<int, GNode> gmap;

        vector<double> xlim {-62, 30};
        vector<double> ylim {-40, 40};
        double dx = 0.1;
        double dy = 0.1;
    
    public:
        GlobalPlanner(Global_State start_state, Global_State goal_state, double max_steering_angle, double dt,
         double desire_vel, double car_length);

        void generate_motion_primitives();

        double PrecomputeCost();

        double compute_H(Global_State st);
        
        vector<MotionPrimitive> transform_primitive(Global_State n_st);

        vector<int> xy2i(Global_State state);
        
        int get_state_hash(Global_State state);

        bool CollisionCheck(MotionPrimitive motion);
        
        bool is_valid_primitive(MotionPrimitive motion);
        
        vector<Global_State> solutionPath(int goal);
        
        vector<Global_State> A_star(Global_State start_state, Global_State goal_state);
};
#endif

/*
ToDo's:
    - Heuristic Computation (Preferably Pre-Compute)
    - Cost Computation (Preferable Pre-Compute)
    - Goal Region Define, (Check if Goal Reached)
    - Collision Checking
        -Preliminary : Circle Based
        -Improved : 3 Circle Based
        -Final : Swath generation (pre-compute) and check.
    - Weighing the forward and backward motion primitives differently (in cost or heuristics)
    - Occupancy Grid integration
    - Anytime D* 

Completed:
    - A* Pseudo Code
    - Motion Primitive Precomputation
    - Motion Primitive Transformation
*/