#ifndef GLOBAL_PLANNER_H
#define GLOBAL_PLANNER_H

#include <iostream>
#include <math.h>
#include <vector>
#include <cfloat>
#include <unordered_map>
#include <set>
#include <Eigen/Dense>
#include <fstream>

#define num_stepsL 9
#define num_stepsS 0
#define PI 3.141592654
#define mapX 460    // xlim [-62, 30]
#define mapY 400    // ylim [-40,  40]

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

// ----------_________________----------------________________---------_____________---
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
// --------_________________-------------______________-----------------______---

class parking
{
    private:
        vector<Global_State> parkX;
        vector<int> isfull = vector<int> (110, 1);
    public:
        parking();
        void emptylots(vector<int> lots);
        vector<Global_State> get_locs();
        Global_State get_loc(int j);
        vector<int> parking_state();
        bool isAvailable(int j);

};

// ----------_______________-------______________-----____________________-
class OccGrid
{
    private:
        double l = 5.142044059999996;   // Dimensions of each parking space
        double w = 2.7572021484375;     // dimensions of each parking space
        double xlim[2] = {-62, 30};
        double ylim[2] = {-40, 40};
        double dx = 0.2;
        double dy = 0.2;
        vector<vector<int>> occ_map = vector<vector<int>> (mapX, vector<int> (mapY,0));

    public:
        OccGrid(double ddx, double ddy)
        {
            dx = ddx;
            dy = ddy;
        }

        void generate_static_occ(parking box);

        // double check_occ(Global_State loc);

        // bool collision_check(MotionPrimitive pattern);
        

        vector<double> pBoxlim(Global_State ploc);
        
        bool isEmpty(int xi, int yi);

        vector<int> xy2i(vector<double> xy);
        void update_static_occ(vector <int> veh_i, int full);
        vector<vector<int>> get_occmap();


};
// --------___________-------------___________------_______________----_____----

struct Node2D
{
    int xi, yi;
    string p;
    double g;
    Node2D(): xi(-1), yi(-1), p(""), g(DBL_MAX)
    {}
    Node2D(int a, int b, string par, double d): xi(a), yi(b), p(par), g(d)
    {}
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
            MotionPrimitive ac;
            GNode(): state(), parent(""), f(DBL_MAX), g(DBL_MAX), h(DBL_MAX), ac()
            {}
            GNode(const Global_State& st, string p, double a, double b, double c, const MotionPrimitive& pat): state(st), parent(p), f(a), g(b), h(c), ac(pat)
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
        vector<double> thetas;

        unordered_map<string, GNode> gmap;

        vector<double> xlim {-62, 30};
        vector<double> ylim {-40, 40};
        double dx = 0.2;
        double dy = 0.2;

    
    public:
        GlobalPlanner(Global_State start_state, Global_State goal_state, double max_steering_angle, double dt,
         double desire_vel, double car_length, double ddx, double ddy);

        void generate_motion_primitives();

        void PrecomputeCost(vector<double> steerF, vector<double> steerB);

        double computeEucH(Global_State st);
        string stateHash2D(int sx, int sy);
        
        double compute2DH(Global_State st, OccGrid occupancy);
        
        vector<MotionPrimitive> transform_primitive(Global_State n_st);

        vector<int> xy2i(Global_State state);

        vector<double> i2xy(vector <int> sti);
        
        string get_state_hash(Global_State state);

        bool CollisionCheck(MotionPrimitive motion);

        bool isGoalState(Global_State st);
        
        bool is_valid_primitive(MotionPrimitive motion);
        
        vector<Global_State> solutionPath(string goal);
        
        vector<Global_State> A_star(Global_State start_state, Global_State goal_state, OccGrid ocmap);

        void print_path(vector <Global_State> path);

        void print_primitives(vector<MotionPrimitive> mpd);

        void motion_primitive_writer(vector <MotionPrimitive> mpd, string file_name);

        void publish_path(vector<Global_State> path, string file_name);

        vector<MotionPrimitive> startS_primitives();

    
};

#endif

/*
ToDo's:
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
    - Update state_hash to incorporate [x,y,theta], & not just [x,y]
    - Added parking class 
*/