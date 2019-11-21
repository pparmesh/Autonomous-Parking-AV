#include "global_planner.h"
#include <ctime>
// #include <iostream>
// #include <math.h>
// #include <vector>
// #include <cfloat>
// #include <unordered_map>
// #include <set>


void MotionPrimitive::insert_state(Global_State st)
{
    primitives.push_back(st);
}

void MotionPrimitive::clear_primitives()
{
    primitives.clear();
}

vector<Global_State> MotionPrimitive::get_primitive()
{
    return primitives;
}

Global_State MotionPrimitive::next_state()
{
    return primitives.back();
}

// -----------------------------------------------------------------------------------------------------------------------------------------

GlobalPlanner::GlobalPlanner(Global_State st_state, Global_State g_state, double max_str_angle, double delt,
double desire_vel, double car_len)
{
    start_state = st_state;
    goal_state = g_state;
    max_steering_angle = max_str_angle;
    dt = delt;
    desired_velocity = desire_vel;
    car_length= car_len;
}

void GlobalPlanner::generate_motion_primitives()
{
    // Function to Precompute the set of motion primitives
    Global_State st_0 = start_state;

    int nsteps = 8;
    int m=0;
    // ------Motion Primitives in the forward direction--------------
    double f_Vx = 2;
    double x = st_0.x;
    double y = st_0.y;
    double theta = st_0.theta;
    double d_delta = 0.08;  // steering angle discretization in radians
    vector<double> deltaF;
    for(double d = -max_steering_angle; d <= max_steering_angle; d += d_delta)
        deltaF.push_back(d);
    for(double d_del : deltaF)
    {
        MotionPrimitive p;
        p.insert_state(st_0);
        primitive_M.col(m) << st_0.x, st_0.y, st_0.theta;
        ++m;
        for(int i = 0; i < nsteps; ++i)
        {
            x += f_Vx*cos(theta)*dt;
            y += f_Vx*sin(theta)*dt;
            theta += f_Vx*tan(d_del)*dt/car_length;
            p.insert_state(Global_State(x, y, theta));
            
            //  Adding the motion primitives to the matrix
            primitive_M.col(m) << x, y, theta;
            ++m;
            
        }
        motion_primitives.push_back(p);

    }
    // -----------------Motion Primimtives in the backward direction-------------
    double b_Vx = 1.2;
    x = st_0.x;
    y = st_0.y;
    theta = st_0.theta;
    d_delta = 0.1;
    vector <double> deltaB;
    for(double d =-max_steering_angle; d<= max_steering_angle; d+=d_delta)
        deltaB.push_back(d);
    for(double d_del :  deltaB)
    {
        MotionPrimitive p;
        p.insert_state(st_0);
        primitive_M.col(m) <<st_0.x, st_0.y, st_0.theta;
        ++m;
        for(int i=0; i<nsteps; i++)
        {
            x += b_Vx*cos(theta)*dt;
            y += b_Vx*sin(theta)*dt;
            theta += b_Vx*tan(d_del)*dt/car_length;
            p.insert_state(Global_State(x, y, theta));

            //  Adding the motion primitives to the matrix
            primitive_M.col(m) << x ,y, theta;
            ++m;
        }
        motion_primitives.push_back(p);
    
        // Precomputing the cost of each motion primitive
        PrecomputeCost(deltaF, deltaB);
    }

}

void GlobalPlanner::PrecomputeCost(vector<double> steerF, vector<double> steerB)
{
    // Function to pre-compute the cost for different motion patterns weighed on curvature, forward/reverse.
    double cost;
    for(double delta : steerF)
    {
        cost = abs(delta)*180/PI;
        cost_of_motion.push_back(cost);   
    }

    // For Backward motion primitives---------------------
    for(double delta : steerB)
    {
        cost = 2*abs(delta)*180/PI;     // Setting the cost of backward motion = 2*cost of forward with same steering angle
        cost_of_motion.push_back(cost);
    }
}


double GlobalPlanner::compute_H(Global_State st)
{

}

vector<MotionPrimitive> GlobalPlanner::transform_primitive(Global_State n_st)
{
    // Function to transform (tranlate and rotate) the initial motion primitives to the current vehicle state
    double d_x = n_st.x - start_state.x;
    double d_y = n_st.y - start_state.y;
    double dtheta = n_st.theta - start_state.theta;

    // Formulating the transformation matrix
    Matrix<double, 3, 3> M;
    M << cos(dtheta), -sin(dtheta), d_x,
        sin(dtheta), cos(dtheta), d_y,
        0, 0, 1;
    Matrix<double, 3, num_steps*(27+22)> n_p = M*primitive_M;
    
    // Converting the matrix to vector of motion primitives
    vector<MotionPrimitive> imap;

    MotionPrimitive p;
    for(int i=0; i<n_p.cols();++i)
    {
        p.insert_state(Global_State(n_p.col(i)[0], n_p.col(i)[1], n_p.col(i)[2]+dtheta));
        if((i+1)%num_steps == 0)
        {
            MotionPrimitive p_new(p);
            imap.push_back(p_new);
            p.clear_primitives();
        }
    }
    return imap;
}

vector<int> GlobalPlanner::xy2i(Global_State state)
{
    //Function to conver the parking lot spatial location to occupancy grid indices
     
    int ax = floor((state.x - xlim[0])/dx);
    int ay = floor((state.y - ylim[0])/dy);
    vector <int> pi {ax, ay};
    return pi;
}

int GlobalPlanner::get_state_hash(Global_State state)
{
    // Function to compute unique hash key for each set
    vector<int> ind = xy2i(state);
    int st_hash = ind[0] + ind[1]*mapX;
    return st_hash;
}

bool GlobalPlanner::CollisionCheck(MotionPrimitive motion)
{
    return 1;
}

bool GlobalPlanner::is_valid_primitive(MotionPrimitive motion)
{
    // Function to check if the motion primitive is valid. Checks if the motion primitive goes out of the map size
    // Also checks for collission with obstacles in the occupancy grid.
    
    vector<Global_State> m_step = motion.get_primitive();
    //  Checking for out of bounds.......................
    
    for(Global_State st : m_step)
    {
        vector<int> ind = xy2i(st);
        int x = ind[0];
        int y = ind[1];
        if(x<0 || x>=mapX || y<0 || y>=mapY)
            return 0;

    }

    return 1;
}

bool GlobalPlanner::isGoalState(Global_State st)
{
    // Function to check if the goal_state st can be considered as the goal state
    // Considering a euclidean distance < epsilon to check if goal found (Temporary)
    double eps = 3;
    double diff = sqrt((st.x-goal_state.x)*(st.x-goal_state.x) + (st.y-goal_state.y)*(st.y-goal_state.y) + (st.theta-goal_state.theta)*(st.theta-goal_state.theta));
    if(diff < eps)
        return 1;
    // else
    //     cout<<"diff = "<<diff<<" st:("<<st.x<<","<<st.y<<"); goal st:("<<goal_state.x<<","<<goal_state.y<<")"<<endl;
    return 0;
}


vector<Global_State> GlobalPlanner::solutionPath(int goal)
{
    vector< Global_State> waypoints;
    int curr_state = goal;
    while (gmap[curr_state].parent!=-1)
    {
        waypoints.insert(waypoints.begin(), gmap[curr_state].state);
        curr_state = gmap[curr_state].parent;
    }
    return waypoints;
}

vector<Global_State> GlobalPlanner::A_star(Global_State start_state, Global_State goal_state)
{
    vector<Global_State> path;

    // closed list to keep track of expanded nodes
    unordered_map <int, bool> closed_list;
    int start = get_state_hash(start_state);
    int goal = get_state_hash(goal_state);
    closed_list[goal] = false;
    closed_list[start] = false;


    // A map to store the cell info such as f,g,h values of nodes
    

    // Inititalize the start cell. It has no parents and its f, g & h values are 0
    gmap[start] = GNode(start_state, -1, 0, 0, 0);    // start cell as its parent as itself
    

    // Implement the open list to keep track of states to expand using a set
    // It is a set of f_COORINATE, i.e it has location of state and its f value
    set<f_COORDINATE> open_list; 
    // Add my start cell to my open list
    open_list.insert(f_COORDINATE (0.0, start_state)); 

    int mexp = 0;
    // Expand till open list is not empty
    while(!open_list.empty() && !closed_list[goal])
    {   
        // Pick index with lowest f value from open list. Set will help in doing this as it is ordered.
        //Put this in closed list.
        // Find neighbors of my current index and find f values only if they are not in closed list.
        // If they are not in closed list, find their f-values. If they are in the open list with a larger
        // f-value then update it, otherwise add this index to the open list. 
        // Loop till goal state has not been expanded.

        if(mexp > 10000)
            break;
        ++mexp;
        // Get index from openlist. Pop the first value from the open list.
        f_COORDINATE q = *open_list.begin();
        // Remove it from the open list
        open_list.erase(open_list.begin());
        // Get index of this node
        Global_State q_current = q.st;
        int current_state = get_state_hash(q_current);
        
        // Checking if the state has already been expanded
        if((closed_list.find(current_state)!=closed_list.end()) && closed_list[current_state])
            continue;
        
        // ------If Goal reached, terminate-----------
        if(isGoalState(q_current))
        {
            cout<<" Goal State Reached"<<endl;
            closed_list[goal]=true;
            gmap[goal] = gmap[current_state];
            gmap[goal].state = goal_state;
            break;
        }


        // Pushing the state into current state
        closed_list[current_state] = true; 

        vector<MotionPrimitive> actions = transform_primitive(q_current);   // computing the set of motion patterns for the current set

        for (int mp_i=0;mp_i<actions.size();++mp_i)
        {
            MotionPrimitive step = actions[mp_i];
            // Check if motion pattern is valid
            if(! is_valid_primitive(step))
                continue;
            
            if(! CollisionCheck(step))  // Checking for collision
                continue;
            

            Global_State q_new = step.next_state();

            double fNew, gNew, hNew; // Variables used to find f, g & h values
            
            int new_state = get_state_hash(q_new);

            if((closed_list.find(new_state) != closed_list.end()) && closed_list[new_state])
                continue;   // Skipping if the state is already in the closed list.

            double cost = cost_of_motion[mp_i];
            hNew = compute_H(q_new);
            
            if(gmap.find(new_state) == gmap.end())
                gmap[new_state] = GNode(q_new, current_state, DBL_MAX, DBL_MAX, DBL_MAX);

            if(gmap[new_state].g > gmap[current_state].g + cost)
            {
                gNew = gmap[current_state].g;
                fNew = gNew + hNew;
                gmap[new_state] = GNode(q_new, current_state, fNew, gNew, hNew);
                open_list.insert(f_COORDINATE(fNew, q_new));
            }         
        }    
    }
    cout<<" Number of states expanded: "<<mexp<<endl;
    // -- Backtracking to compute the path if Reached Goal
    if (closed_list[goal])
    {
        cout<<"Solution Found "<<endl;
        return solutionPath(goal);
    }
    return path;
}

int main()
{
    double dx = 0.1 ,dy = 0.1;  // Grid discretization 
    double v_des = 1.2;     // Desrired Velocity
    double l_car = 2.2;     // Wheelbase of the vehicle
    double delT = 0.1;      // delta time of the simulation
    double steer_limit = PI*61/180;

    clock_t start_t = clock();

    Global_State startS = Global_State(0,0,0);
    Global_State goalS = Global_State(15, 15, PI/4);

    GlobalPlanner g_planner(startS, goalS, steer_limit, delT, v_des, l_car);

    // Prcomputing the motion primitives
    g_planner.generate_motion_primitives();
    g_planner.A_star(startS, goalS);

    cout<<" Time taken for computation : "<<(double)(clock() - start_t)/CLOCKS_PER_SEC<<" s"<<endl;
    return 0;
}