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

    int m=0;
    int nsteps;
    // ------Motion Primitives in the forward direction--------------
    
    // Formulating a list of discretized steering angles
    // Forward direction.............
    double f_Vx = 2;
    double x = st_0.x;
    double y = st_0.y;
    double theta = st_0.theta;
    double d_delta = 0.15;  // steering angle discretization in radians 
   
    vector<double> deltaF;
    for(double d=-d_delta;d>=-max_steering_angle;d-=d_delta)
        deltaF.insert(deltaF.begin(),d);
    deltaF.push_back(0);
    for(double d = d_delta; d<= max_steering_angle;d += d_delta)
        deltaF.push_back(d);
    
    //  Backward direction..........
        double b_Vx = 1.2;
    x = st_0.x;
    y = st_0.y;
    theta = st_0.theta;
    d_delta = 0.22;

    vector <double> deltaB;
    for(double d =-d_delta;d>=-max_steering_angle;d-=d_delta)
        deltaB.insert(deltaB.begin(), d);
    deltaB.push_back(0);
    for(double d=d_delta;d<= max_steering_angle; d+=d_delta)
        deltaB.push_back(d);

    // // Resizing the primitive_M Eigen matrix 
    // primitive_M = MatrixXd(3, 400);// num_steps*(deltaF.size() + deltaB.size()));
    
    //  Computing and storing the motion primitives......... 
    for(int jj=0;jj<2;++jj)
    {
        if(jj==0)
            nsteps = num_stepsL-1;
        else
            nsteps = num_stepsS-1;
        //  Defining motion primitives for 2 step sizes
         
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
        }
    }    
    PrecomputeCost(deltaF, deltaB);

    // //  Priniting the motion primitives and cost:::
    // vector <double> st_ang;
    // for(double f : deltaF)
    //     st_ang.push_back(f);
    // for(double f : deltaB)
    //     st_ang.push_back(f);
    
    // for(int i=0;i<st_ang.size();++i)
    //     cout<<"Angle: "<<st_ang[i]<<" "<<" ;Cost: "<<cost_of_motion[i]<<endl;
}

void GlobalPlanner::PrecomputeCost(vector<double> steerF, vector<double> steerB)
{
    // Function to pre-compute the cost for different motion patterns weighed on curvature, forward/reverse.
    double cost;
    for(int jj=0;jj<2;++jj)
    {
        for(double delta : steerF)
        {
            cost = abs(delta)*180/PI;
            cost_of_motion.push_back(cost);   
        }

        // For Backward motion primitives---------------------
        for(double delta : steerB)
        {
            if(delta == 0)
                cost = 4;
            else
                cost = 4*abs(delta)*180/PI;     // Setting the cost of backward motion = 2*cost of forward with same steering angle
            cost_of_motion.push_back(cost);
        }
    }
}


double GlobalPlanner::computeEucH(Global_State st)
{
    // Euclidean between current state [x,y,theta] and the goal state [x,y,theta]
    double h =sqrt((st.x-goal_state.x)*(st.x-goal_state.x) + (st.y-goal_state.y)*(st.y-goal_state.y) + (st.theta-goal_state.theta)*(st.theta-goal_state.theta));

    return h;
}

double GlobalPlanner::compute2DH(Global_State st)
{
    // Using a 2D robot in xy world to compute the heurisics
    
    
}

vector<MotionPrimitive> GlobalPlanner::transform_primitive(Global_State n_st)
{
    // Function to transform (tranlate and rotate) the initial motion primitives to the current vehicle state
    double d_x = n_st.x - start_state.x;
    double d_y = n_st.y - start_state.y;
    double dtheta = n_st.theta - start_state.theta;
    int num_steps;
    // Formulating the transformation matrix
    Matrix<double, 3, 3> M;
    M << cos(dtheta), -sin(dtheta), d_x,
        sin(dtheta), cos(dtheta), d_y,
        0, 0, 1;

    Matrix<double, 3, (num_stepsL+num_stepsS)*(15+9)> n_p = M*primitive_M;
    
    // Converting the matrix to vector of motion primitives
    vector<MotionPrimitive> imap;

    MotionPrimitive p;
    for(int jj=0;jj<2;++jj)
    {
        if(jj==0)
            num_steps = num_stepsL-1;
        else
            num_steps = num_stepsS-1;

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
    }
    return imap;
}

vector<int> GlobalPlanner::xy2i(Global_State state)
{
    //Function to conver the parking lot spatial location to occupancy grid indices
    double dx = 0.1, dy = 0.1; 
    int ax = floor((state.x - xlim[0])/dx);
    int ay = floor((state.y - ylim[0])/dy);
    vector <int> pi {ax, ay};
    return pi;
}

string GlobalPlanner::get_state_hash(Global_State state)
{
    // Function to compute unique hash key for each set
    vector<int> ind = xy2i(state);
    string st_hash = "";
    string g = to_string(state.theta);
    st_hash += to_string(ind[0]) + to_string(ind[1]) + g.substr(0.4); 
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
    double eps = 2;
    double diff = sqrt((st.x-goal_state.x)*(st.x-goal_state.x) + (st.y-goal_state.y)*(st.y-goal_state.y));// + (st.theta-goal_state.theta)*(st.theta-goal_state.theta));
    double d_thet = abs(st.theta-goal_state.theta);
    double dstr = PI/18;
    // cout<<diff<<" "<<d_thet<<endl;
    if(diff < eps && d_thet<dstr)
        return 1;

    return 0;
}


vector<Global_State> GlobalPlanner::solutionPath(string goal)
{
    vector< Global_State> waypoints;
    string curr_state = goal;
    while (gmap[curr_state].parent!="-1")
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
    unordered_map <string, bool> closed_list;
    string start = get_state_hash(start_state);
    string goal = get_state_hash(goal_state);
    closed_list[goal] = false;
    closed_list[start] = false;


    // A map to store the cell info such as f,g,h values of nodes
    

    // Inititalize the start cell. It has no parents and its f, g & h values are 0
    gmap[start] = GNode(start_state, "-1", 0, 0, 0);    // start cell as its parent as itself
    

    // Implement the open list to keep track of states to expand using a set
    // It is a set of f_COORINATE, i.e it has location of state and its f value
    set<f_COORDINATE> open_list; 
    // Add my start cell to my open list
    open_list.insert(make_pair(0.0, start)); 

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

        if(mexp > 20000)
            break;
        ++mexp;
        // Get index from openlist. Pop the first value from the open list.
        f_COORDINATE q = *open_list.begin();
        // Remove it from the open list
        open_list.erase(open_list.begin());
        // Get index of this node
        Global_State q_current = gmap[q.second].state;
        string curr_state = get_state_hash(q_current);
        
        // Checking if the state has already been expanded
        if((closed_list.find(curr_state)!=closed_list.end()) && closed_list[curr_state])
            continue;
        
        // ------If Goal reached, terminate-----------
        if(curr_state == goal)
        {
            cout<<" Goal Reached" << endl;
            cout<<q_current.x <<" "<<q_current.y<<" "<<q_current.theta<<endl;
            cout<<goal_state.x<<" "<<goal_state.y<<" "<<goal_state.theta<<endl;
            closed_list[goal] = true;
            break;
        }
        if(isGoalState(q_current))
        {
            cout<<" Goal State Reached"<<endl;
            closed_list[goal]=true;
            gmap[goal] = gmap[curr_state];
            gmap[goal].state = goal_state;
            break;
        }


        // Pushing the state into current state
        closed_list[curr_state] = true; 

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
            
            string new_state = get_state_hash(q_new);

            if((closed_list.find(new_state) != closed_list.end()) && closed_list[new_state])
                continue;   // Skipping if the state is already in the closed list.

            double cost = cost_of_motion[mp_i];
            hNew = computeEucH(q_new);
            
            if(gmap.find(new_state) == gmap.end())
                gmap[new_state] = GNode(q_new, curr_state, DBL_MAX, DBL_MAX, DBL_MAX);

            if(gmap[new_state].g > gmap[curr_state].g + cost)
            {
                gNew = gmap[curr_state].g;
                fNew = gNew + hNew;
                gmap[new_state] = GNode(q_new, curr_state, fNew, gNew, hNew);
                open_list.insert(make_pair(fNew, new_state));
            }         
        }    
    }
    cout<<" Number of states expanded: "<<mexp<<endl;
    cout<<" Open list size: "<<open_list.size()<<endl;
    // -- Backtracking to compute the path if Reached Goal
    if (closed_list[goal])
    {
        cout<<"Solution Found "<<endl;
        path = solutionPath(goal);
    }
    return path;
}

void print_path(vector<Global_State> path)
{
    vector<double> vx;
    vector<double> vy;
    vector<double>  vtheta;
    for(int i=0;i<path.size();++i)
    {
        vx.push_back(path[i].x);
        vy.push_back(path[i].y);
        vtheta.push_back(path[i].theta);
    }  
    cout<<"X: \n";
    for(double i : vx)
        cout<<i<<",";
    cout<<"\n Y: \n";
    for(double i : vy)
        cout<<i<<",";
    cout<<"\n theta: \n";
    for(double i : vtheta)
        cout<<i<<",";
    cout<<endl;
}

int main()
{
    double dx = 0.1 ,dy = 0.1;  // Grid discretization 
    double v_des = 1.2;     // Desrired Velocity
    double l_car = 2.2;     // Wheelbase of the vehicle
    double delT = 0.1;      // delta time of the simulation
    double steer_limit = PI*61/180;

    clock_t start_t = clock();

    Global_State startS = Global_State(-44.81,-31.04,PI/2);
    Global_State goalS = Global_State(-13.5, -31.27, PI/2);
    // Global_State startS = Global_State(-48, 11.4, 0);
    // Global_State goalS = Global_State(26, 11.4, 0);

    GlobalPlanner g_planner(startS, goalS, steer_limit, delT, v_des, l_car);

    vector<Global_State> vehicle_path;
    // Prcomputing the motion primitives
    g_planner.generate_motion_primitives();
    vehicle_path = g_planner.A_star(startS, goalS);

    cout<<" Time taken for computation : "<<(double)(clock() - start_t)/CLOCKS_PER_SEC<<" s"<<endl;
    print_path(vehicle_path);
    return 0;
}   