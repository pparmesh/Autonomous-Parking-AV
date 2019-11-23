#ifndef GLOBAL_PLANNER_H
#define GLOBAL_PLANNER_H

#include <iostream>
#include <math.h>
#include <vector>
#include <cfloat>
#include <unordered_map>
#include <set>
#include <Eigen/Dense>

#define num_stepsL 9
#define num_stepsS 4
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
    private:
        
        Global_State start_state;
        Global_State goal_state;
        struct GNode
        {
            Global_State state;
            string parent;
            double f;
            double g;
            double h;
            GNode(): state(), parent(""), f(DBL_MAX), g(DBL_MAX), h(DBL_MAX)
            {}
            GNode(const Global_State& st, string p, double a, double b, double c): state(st), parent(p), f(a), g(b), h(c)
            {}
            GNode(const GNode& pp)
            {
                this->state = Global_State(pp.state);
                this->parent = pp.parent;
                this->f = pp.f;
                this->g = pp.g;
                this->h = pp.h;
            }
        };
        double max_steering_angle; // Max steering angle of vehicle
        double dt; // Time step for lattice graph
        double desired_velocity;
        double car_length;
        typedef pair <double, string> f_COORDINATE;

        vector<MotionPrimitive> motion_primitives;

        vector<double> cost_of_motion;
        MatrixXd primitive_M= MatrixXd(3,(num_stepsL+num_stepsS)*(15+9)); //num_steps*(28+23)> primitive_M;

        unordered_map<string, GNode> gmap;

        vector<double> xlim {-62, 30};
        vector<double> ylim {-40, 40};

    
    public:
        GlobalPlanner(Global_State start_state, Global_State goal_state, double max_steering_angle, double dt,
         double desire_vel, double car_length);

        void generate_motion_primitives();

        void PrecomputeCost(vector<double> steerF, vector<double> steerB);

        double computeEucH(Global_State st);
        double compute2DH(Global_State st);
        
        vector<MotionPrimitive> transform_primitive(Global_State n_st);

        vector<int> xy2i(Global_State state);

        vector<double> i2xy(vector <int> sti);
        
        string get_state_hash(Global_State state);

        bool CollisionCheck(MotionPrimitive motion);

        bool isGoalState(Global_State st);
        
        bool is_valid_primitive(MotionPrimitive motion);
        
        vector<Global_State> solutionPath(string goal);
        
        vector<Global_State> A_star(Global_State start_state, Global_State goal_state);

        void print_path(vector <Global_State> path);
    
};

class OccGrid
{
    private:
        double l = 5.142044059999996;   // Dimensions of each parking space
        double w = 2.7572021484375;     // dimensions of each parking space
        double xlim[2] = {-62, 30};
        double ylim[2] = {-40, 40};
        double dx = 0.1;
        double dy = 0.1;
        vector<vector<double>> occ_map;
        vector<Global_State> parking_locations;

    public:
        OccGrid()
        {
            vector<double> map_row {0, mapY};
            for(int i=0;i<mapX;++i)
                occ_map.push_back(map_row);
        
        }

        void update_occ(vector<Global_State> vehicles);

        double check_occ(Global_State loc);

        bool collision_check(MotionPrimitive pattern);
        
        vector<Global_State> get_parking_loc();


};
#endif

/*
ToDo's:
    - Update state_hash to incorporate [x,y,theta], & not just [x,y]
    - Goal Region Define, (Check if Goal Reached)
    - Occupancy Grid
    - Heuristic Computation (Preferably Pre-Compute)
    - Collision Checking
        -Preliminary : Circle Based
        -Improved : 3 Circle Based
        -Final : Swath generation (pre-compute) and check.
    - Weighing the forward and backward motion primitives differently (in cost or heuristics)
    - Occupancy Grid integration
    - Anytime D* 

Completed:
    - Add motion primitives of 2 step sizes.
    - A* Pseudo Code
    - Motion Primitive Precomputation
    - Motion Primitive Transformation
    - Cost Computation (Preferable Pre-Compute) -> forward and backward primitives weighed differently.
    - Preliminary Goal State check implemented

*/