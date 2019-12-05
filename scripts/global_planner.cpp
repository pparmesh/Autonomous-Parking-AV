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
double desire_vel, double car_len, double ddx, double ddy)
{
    start_state = st_state;
    goal_state = g_state;
    max_steering_angle = max_str_angle;
    dt = delt;
    desired_velocity = desire_vel;
    car_length= car_len;
    dx = ddx;
    dy = ddy;
}


void GlobalPlanner::generate_motion_primitives()
{
    // Function to Precompute the set of motion primitives
    Global_State st_0 = start_state;

    int m=0;
    int sm=0;
    int nsteps = num_steps -1;
    // ------Motion Primitives in the forward direction--------------
    
    // Formulating a list of discretized steering angles
    // Forward direction.............
    double f_Vx = 2.0;
    double x = st_0.x;
    double y = st_0.y;
    double x0 = st_0.x;
    double y0 = st_0.y;
    double theta = st_0.theta;
    double d_delta = 0.1;  // steering angle discretization in radians 
   
    vector<double> deltaF;
    for(double d=-d_delta;d>=-max_steering_angle;d-=d_delta)
        deltaF.insert(deltaF.begin(),d);
    deltaF.push_back(0);
    for(double d = d_delta; d<= max_steering_angle;d += d_delta)
        deltaF.push_back(d);

        
    //  Backward direction..........
    double b_Vx = -1.2;    
    d_delta = 0.2;

    vector <double> deltaB;
    for(double d =-d_delta;d>=-max_steering_angle;d-=d_delta)
        deltaB.insert(deltaB.begin(), d);
    deltaB.push_back(0);
    for(double d=d_delta;d<= max_steering_angle; d+=d_delta)
        deltaB.push_back(d);
    
    //  Computing and storing the motion primitives......... 
    for(double d_del : deltaF)
    {
        MotionPrimitive p;
        MotionPrimitive swc;

        x = st_0.x;
        y = st_0.y;
        theta = st_0.theta;
        p.insert_state(st_0);
        generate_cc(swc, st_0, sm); //+++++++++++++++++++++++++++++++++++++++++++++++++++++++

        primitive_M.col(m) << x-x0, y-y0, 1.0;  //theta;
        thetas.push_back(theta);
        ++m;
        for(int i = 0; i < nsteps; ++i)
        {
            x += f_Vx*cos(theta)*dt;
            y += f_Vx*sin(theta)*dt;
            theta += f_Vx*tan(d_del)*dt/car_length;
            p.insert_state(Global_State(x, y, theta));
            generate_cc(swc, Global_State(x, y, theta), sm);	//+++++++++++++++++++++++++++++
            //  Adding the motion primitives to the matrix
            primitive_M.col(m) << x-x0, y-y0, 1.0;  //theta;
            thetas.push_back(theta);
            ++m;
            
        }
        motion_primitives.push_back(p);
        swath_p.push_back(swc);	// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    }

    // -----------------Motion Primimtives in the backward direction-------------
    /*
    for(double d_del :  deltaB)
    {
        MotionPrimitive p;
        x = st_0.x;
        y = st_0.y;
        theta = st_0.theta;
        p.insert_state(st_0);
        primitive_M.col(m) <<x-x0, y-y0, 1.0; //theta;
        thetas.push_back(theta);
        ++m;
        for(int i=0; i<nsteps; i++)
        {
            x += b_Vx*cos(theta)*dt;
            y += b_Vx*sin(theta)*dt;
            theta += b_Vx*tan(d_del)*dt/car_length;
            p.insert_state(Global_State(x, y, theta));

            //  Adding the motion primitives to the matrix
            primitive_M.col(m) << x-x0 ,y-y0, 1.0;  //theta;
            thetas.push_back(theta);
            ++m;
        }
        motion_primitives.push_back(p);
    
        // Precomputing the cost of each motion primitive
    }
    */

    PrecomputeCost(deltaF, deltaB);
  
}

vector<MotionPrimitive> GlobalPlanner::generate_c(vector<MotionPrimitive> steps)
{
    vector <MotionPrimitive> omap;
    for(MotionPrimitive a : steps)
    {
        MotionPrimitive p;
        vector<Global_State> pp = a.get_primitive();
        for(Global_State ap : pp)
        {
            double x = ap.x;
            double y = ap.y;
            double ang = wrap2pi(ap.theta);

            p.insert_state(Global_State(x-cos(ang), y-sin(ang), ang));
            p.insert_state(Global_State(x+cos(ang), y+sin(ang), ang));
        }
        omap.push_back(p);
    }
    return omap;

}

void GlobalPlanner::generate_cc(MotionPrimitive& sw, Global_State st, int& i)
{
	// Function to add the circle centers for all the states in the current motion primitive.
	double x1 = st.x;
	double y1 = st.y ;
	double ang = wrap2pi(st.theta);

	double x2 = x1 - cos(ang);
	double y2 = y1 - sin(ang);
	sw.insert_state(Global_State(x2, y2, ang));
	sw_M.col(i)<<x2-x1, y2-y1, 1.0;
	++i;

	x2 = x1 +cos(ang);
	y2 = y1+ sin(ang);
	sw.insert_state(Global_State(x2, y2, ang));
	sw_M.col(i)<<x2-x1, y2-y1, 1.0;
	++i;
}

void GlobalPlanner::PrecomputeCost(vector<double> steerF, vector<double> steerB)
{
    // Function to pre-compute the cost for different motion patterns weighed on curvature, forward/reverse.
    double cost;

    for(double delta : steerF)
    {
        if(delta == 0)
            cost = 1;   
        else
            cost = 15*abs(delta); 
        // cost = abs(delta)*180/PI;
        cost_of_motion.push_back(cost);  
    cout<<"detla: "<<delta<<", cost: "<<cost<<endl;
    }
    // For Backward motion primitives---------------------
    /*
    for(double delta : steerB)
    {
        if(delta == 0)
            cost = 2*5;
        else
            cost = 5*abs(delta);     // Setting the cost of backward motion = 2*cost of forward with same steering angle
        cost_of_motion.push_back(cost);
    }
    */

}


double GlobalPlanner::computeEucH(Global_State st)
{
    // Euclidean between current state [x,y,theta] and the goal state [x,y,theta]
    // double h =sqrt((st.x-goal_state.x)*(st.x-goal_state.x) + (st.y-goal_state.y)*(st.y-goal_state.y) + (st.theta-goal_state.theta)*(st.theta-goal_state.theta));
    double h =0;
    return h;
}

string GlobalPlanner::stateHash2D(int sx, int sy)
{
    string stg = "";
    stg += to_string(sx) + to_string(sy);

    return stg;
}

// ------2D Heuristics---------
void GlobalPlanner::pre_compute2DH(Global_State st, OccGrid occupancy)
{
 // Using a 2D robot in xy world to compute the heurisics
    unordered_map <string, bool> p_close;
    set<f_COORDINATE> p_open;
    int dirs = 8;
    double euH;

    vector <int> startS = xy2i(st);
    vector <int> goalS = xy2i(goal_state);
    string startH = stateHash2D(startS[0], startS[1]);
    string goalH = stateHash2D(goalS[0], goalS[1]);
    p_close[startH] = false;
    p_close[goalH] = false;

    // p_open.insert(make_pair(0.0, startH));
    p_open.insert(make_pair(0.0, goalH));

    // unordered_map<string, Node2D> hmap;
    // hmap[startH] = Node2D(startS[0], startS[1], "-1", 0);
    hmap[goalH] = Node2D(goalS[0], goalS[1], "-1", 0);

    int dX[dirs] = {-1, -1, -1, 0, 0, 1, 1, 1};
    int dY[dirs] = {-1, 0, 1, -1, 1, -1, 0, 1};

    int qx, qy;
    while(!p_open.empty()) // && !p_close[goalH])
    {
        //  Popping the top node on the open list
        f_COORDINATE q =*p_open.begin();
        p_open.erase(p_open.begin());
        qx = hmap[q.second].xi;
        qy = hmap[q.second].yi;
        string qH = stateHash2D(qx, qy);
        if(p_close.find(qH)!=p_close.end() && p_close[qH])
            continue;       // skipping if the state already in closed list
        
        // if(qH == goalH)
        // {
        //     // cout<<"goal reached"<<endl;
        //     p_close[goalH]=true;
        //     break;
        // }
        if(qH == startH)
        {
            p_close[startH]= true;
            // break;
        }

        for(int i=0; i<dirs;++i)
        {
            int newX = qx + dX[i];
            int newY = qy + dY[i];
            string q_newH = stateHash2D(newX, newY);

            if(p_close[q_newH])
                continue;   //skipping if q_new in closed list already

            if(newX>=0 && newX<mapX && newY>=0 && newY<mapY)
            {
                if(!occupancy.isEmpty(newX, newY))
                { 
                    continue;   // skipping if there is an obstacle at the (x,y) location
                }
                if(hmap.find(q_newH) == hmap.end())
                    hmap[q_newH] = Node2D(newX, newY, qH, DBL_MAX);
                euH = 0; //sqrt((newX-goalS[0])*(newX-goalS[0]) + (newY-goalS[1])*(newY-goalS[1]));

                double gg = hmap[qH].g +1;
                if(hmap[q_newH].g > gg)
                {
                    hmap[q_newH] = Node2D(newX, newY, qH, gg);
                    p_open.insert(make_pair(gg+euH, q_newH));
                }
            }
        }
    }
}

double GlobalPlanner::computeH(Global_State st, OccGrid occupancy)
{    
 vector <int> qst = xy2i(st);
 string q_new = stateHash2D(qst[0], qst[1]);

 if(hmap.find(q_new) == hmap.end())
 {
    cout<<" heuristic could not find the state \n";
    pre_compute2DH(st, occupancy);
    cout<<hmap[q_new].g<<endl;
    return hmap[q_new].g;
 }
 else
    return hmap[q_new].g;
}


double GlobalPlanner::compute2DH(Global_State st, OccGrid occupancy)
{
    // Using a 2D robot in xy world to compute the heurisics
    unordered_map <string, bool> p_close;
    set<f_COORDINATE> p_open;
    int dirs = 8;
    double euH;

    vector <int> startS = xy2i(st);
    vector <int> goalS = xy2i(goal_state);
    string startH = stateHash2D(startS[0], startS[1]);
    string goalH = stateHash2D(goalS[0], goalS[1]);
    p_close[startH] = false;
    p_close[goalH] = false;

    p_open.insert(make_pair(0.0, startH));

    unordered_map<string, Node2D> imap;
    imap[startH] = Node2D(startS[0], startS[1], "-1", 0);

    p_open.insert(make_pair(0.0, startH));
    int dX[dirs] = {-1, -1, -1, 0, 0, 1, 1, 1};
    int dY[dirs] = {-1, 0, 1, -1, 1, -1, 0, 1};

    int qx, qy;
    while(!p_open.empty() && !p_close[goalH])
    {
        //  Popping the top node on the open list
        f_COORDINATE q =*p_open.begin();
        p_open.erase(p_open.begin());
        qx = imap[q.second].xi;
        qy = imap[q.second].yi;
        string qH = stateHash2D(qx, qy);
        if(p_close.find(qH)!=p_close.end() && p_close[qH])
            continue;       // skipping if the state already in closed list
        
        if(qH == goalH)
        {
            // cout<<"goal reached"<<endl;
            p_close[goalH]=true;
            break;
        }

        for(int i=0; i<dirs;++i)
        {
            int newX = qx + dX[i];
            int newY = qy + dY[i];
            string q_newH = stateHash2D(newX, newY);

            if(p_close[q_newH])
                continue;   //skipping if q_new in closed list already

            if(newX>=0 && newX<mapX && newY>=0 && newY<mapY)
            {
                if(!occupancy.isEmpty(newX, newY))
                { 
                    continue;   // skipping if there is an obstacle at the (x,y) location
                }
                if(imap.find(q_newH) == imap.end())
                    imap[q_newH] = Node2D(newX, newY, qH, DBL_MAX);
                euH = sqrt((newX-goalS[0])*(newX-goalS[0]) + (newY-goalS[1])*(newY-goalS[1]));

                double gg = imap[qH].g +1;
                if(imap[q_newH].g > gg)
                {
                    imap[q_newH] = Node2D(newX, newY, qH, gg);
                    p_open.insert(make_pair(gg+euH, q_newH));
                }
            }
        }
    }
    
    if(!p_close[goalH])
    {
        cout<<" 2D heuristic could not find a path"<<endl;
        return -1;
    }
    // int h=0;
    // while(goalH != startH)
    // {
    //     ++h;
    //     goalH = imap[goalH].p;
    // }
    // return h;
    return imap[goalH].g;
}

vector<MotionPrimitive> GlobalPlanner::transform_primitive(Global_State n_st)
{
    // Computing the relative transformation (rotation and translation) between the two motion primitive frames.
    double dx = n_st.x - start_state.x;
    double dy = n_st.y - start_state.y;
    double dtheta = n_st.theta - start_state.theta;


    // Formulating the transformation matrix
    Matrix<double, 3, 3> M;
    M << cos(dtheta), -sin(dtheta), start_state.x,
        sin(dtheta), cos(dtheta), start_state.y,
        0, 0, 1; 

    // Transforming the frame of reference of the motion primitives
    const int nmp =21;//+11;
    Matrix<double, 3, (num_steps)*(nmp)> n_p = M*primitive_M;
    
    // Converting the matrix to vector of motion primitives
    vector<MotionPrimitive> imap;
    MotionPrimitive p;

    // motion primitives with 9 steps
    int mi;
    for(mi=0;mi<(num_steps*(nmp));++mi)
    {
        p.insert_state(Global_State(n_p.col(mi)[0]+dx, n_p.col(mi)[1]+dy, wrap2pi(thetas[mi]+dtheta)));
        if((mi+1)%num_steps == 0)
        {
            MotionPrimitive p_new(p);
            imap.push_back(p_new);
            p.clear_primitives();
        }
    }

    return imap;
}

vector<MotionPrimitive> GlobalPlanner::transform_swath(Global_State n_st)
{
	// ++++++++++++++++++++++++Function to tranform (translate + rotate) te circle centers for swath generation
	double dx = n_st.x - start_state.x;
	double dy = n_st.y - start_state.y;
	double dtheta = n_st.theta - start_state.theta;

	// Formulating the transformation matrix.......
	Matrix <double, 3, 3>   M;
	M<<cos(dtheta), -sin(dtheta), start_state.x,
		sin(dtheta), cos(dtheta), start_state.y,
		0, 0, 1;

	const int nmp = 21;		// number of motion primitives
	Matrix<double, 3, num_steps*4*nmp> n_p = M*sw_M;

	// Converting the matrix to a vector of motion primitives
	vector <MotionPrimitive> smap;
	MotionPrimitive s;

	
	for(int pi=0; pi<(num_steps*nmp); ++pi)
	{
		for(int mi =pi*2;mi<(pi*2 + 2);++mi)
		{
			s.insert_state(Global_State(n_p.col(mi)[0]+dx, n_p.col(mi)[1]+dy, wrap2pi(thetas[mi]+dtheta)));
		}
		if((pi+1)%num_steps ==0)
		{
			MotionPrimitive s_new(s);
			smap.push_back(s_new);
			s.clear_primitives();
		}
	}
	return smap;
}

vector<int> GlobalPlanner::xy2i(Global_State state)
{
    //Function to conver the parking lot spatial location to occupancy grid indices
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
    st_hash += to_string(ind[0]) +","+ to_string(ind[1]) +","+ g.substr(0,4); 
    return st_hash;
}


bool GlobalPlanner::CollisionCheck(Global_State st, MotionPrimitive motion, OccGrid ocmap)
{
    // Function to check if the motion primitive path is collision free.
    // 0: Collision free, 1: Collision

    vector <Global_State> m_step = motion.get_primitive();
    // Collision check for a point robot......................

    // for(Global_State stg : m_step)
    // {
    //     vector <int> index = xy2i(stg);
    //     if(ocmap.isEmpty(index[0], index[1]))
    //         continue;
    //     else
    //         return 1;
    // }

    // Collision check for swath..........................
    vector< vector<int>> ic;
    for(Global_State stg : m_step)
    	ic.push_back(xy2i(stg));

    vector <vector<int>> obs_map = ocmap.get_occmap();
    vector<int> state = xy2i(st);
    int x1 = max(0, state[0]-20);
    int x2 = min(mapX, state[0] + 20);
    int y1 = max(0, state[1]-20);
    int y2 = min(mapY, state[1] + 20)   ;

    for(int i=x1;i<x2;++i)
    {
        for(int j=y1; j<y2;++j)
        {
            for(int p = 0;p<ic.size();++p)
            {
                if(obs_map[i][j]==0)
                    continue;
                double d = sqrt((ic[p][0]-i)*(ic[p][0]-i) + (ic[p][1]-j)*(ic[p][1]-j));
                if(d<=1.1)
                    return 1;
            }
        }
    }    

    return 0;
}

bool GlobalPlanner::is_valid_primitive(MotionPrimitive motion)
{
    // Function to check if the motion primitive is valid. Checks if the motion primitive goes out of the map size
    // Also checks for collission with obstacles in the occupancy grid.
    
    vector<Global_State> m_step = motion.get_primitive();
    //  Checking for out of bounds.......................
    
    for(Global_State stg : m_step)
    {
        vector<int> ind = xy2i(stg);
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
    double eps = 0.5;
    double diff = sqrt((st.x-goal_state.x)*(st.x-goal_state.x) + (st.y-goal_state.y)*(st.y-goal_state.y));// + (st.theta-goal_state.theta)*(st.theta-goal_state.theta));
    double d_thet = abs(st.theta-goal_state.theta);
    double dstr = PI/28;
    // cout<<diff<<" "<<d_thet<<endl;
    if(diff < eps && d_thet<dstr)
        return 1;

    return 0;
}


vector<Global_State> GlobalPlanner::solutionPath(string goal)
{
    vector< Global_State> waypoints;
    string curr_state = goal;
    while (true)
    {
        waypoints.insert(waypoints.begin(), gmap[curr_state].state);
        // Storing the waypoints from the motion primitive action that led to the current state
        vector<Global_State> actions = gmap[curr_state].ac.get_primitive();
        if(gmap[curr_state].parent == "-1")
            break;
        else
            curr_state = gmap[curr_state].parent;

        // Adding the waypoints from the motion primitive.........
        for(int i = actions.size()-1;i>=0;--i)
            waypoints.insert(waypoints.begin(), actions[i]);
        
    }
    return waypoints;
}

vector<Global_State> GlobalPlanner::A_star(Global_State start_state, Global_State goal_state, OccGrid ocmap)
{
    vector<Global_State> path;

    cout<<"start state: "<<start_state.x<<" "<<start_state.y<<" "<<start_state.theta<<endl;
    cout<<"goal state: "<<goal_state.x<<" "<<goal_state.y<<" "<<goal_state.theta<<endl;
    // closed list to keep track of expanded nodes
    unordered_map <string, bool> closed_list;
    string start = get_state_hash(start_state);
    string goal = get_state_hash(goal_state);
    closed_list[goal] = false;
    closed_list[start] = false;


    // A map to store the cell info such as f,g,h values of nodes
    

    // Inititalize the start cell. It has no parents and its f, g & h values are 0
    gmap[start] = GNode(start_state, "-1", 0, 0, 0, motion_primitives[0]);    // start cell as its parent as itself
    

    // Implement the open list to keep track of states to expand using a set
    // It is a set of f_COORINATE, i.e it has location of state and its f v2 = x1 + 2*cos(ang);
    // y2 = y1 + 2*sin(ang);
    // sw.insert_state(Global_State(x2, y2, ang));
    // sw_M.col(i)<<x2-x1, y2-y1, 1.0;
    // +alue
    set<f_COORDINATE> open_list; 
    // Add my start cell to my open list
    open_list.insert(make_pair(0.0, start)); 

    int mexp = 0;
    // Expand till open list is not empty
    double wt = 2.4;    // weight for weighted heuristic
    while(!open_list.empty() && !closed_list[goal])
    {   
        // Pick index with lowest f value from open list. Set will help in doing this as it is ordered.
        //Put this in closed list.
        // Find neighbors of my current index and find f values only if they are not in closed list.
        // If they are not in closed list, find their f-values. If they are in the open list with a larger
        // f-value then update it, otherwise add this index to the open list. 
        // Loop till goal state has not been expanded.
        if(mexp > 5000)
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
        // cout<<"open list size: "<<open_list.size()<<" "<<q_current.x<<" "<<q_current.y<<" "<<q_current.theta<<endl;

        // Pushing the state into current state
        closed_list[curr_state] = true; 

        vector <MotionPrimitive> actions = transform_primitive(q_current);   // computing the set of motion patterns for the current set
        vector <MotionPrimitive> swath_cc = transform_primitive(q_current);
        // vector <MotionPrimitive> swath_cc = generate_c(actions);
        for (int mp_i=0;mp_i<actions.size();++mp_i)
        {
            MotionPrimitive step = actions[mp_i];
            // Check if motion pattern is valid
            if(! is_valid_primitive(step))
            {   
                // Global_State sfg = step.next_state();
                // cout<<"not valid: "<<sfg.x<<" ,"<<sfg.y<<endl;
                continue;
            }
            if(CollisionCheck(q_current, swath_cc[mp_i], ocmap))  // Checking for collision
                continue;
            

            Global_State q_new = step.next_state();

            double fNew, gNew, hNew; // Variables used to find f, g & h values
            
            string new_state = get_state_hash(q_new);

            if((closed_list.find(new_state) != closed_list.end()) && closed_list[new_state])
                continue;   // Skipping if the state is already in the closed list.

            double cost = cost_of_motion[mp_i];
            hNew = computeH(q_new, ocmap);
            // hNew = compute2DH(q_new, ocmap);
            if(hNew == -1)  // skipping the state if 2D heuristic couldn't find path to goal-state
                continue;
            else
                hNew = hNew * wt;
            if(gmap.find(new_state) == gmap.end())
                gmap[new_state] = GNode(q_new, curr_state, DBL_MAX, DBL_MAX, DBL_MAX, step);

            if(gmap[new_state].g > (gmap[curr_state].g + cost))
            {
                gNew = gmap[curr_state].g + cost;
                fNew = gNew + hNew;
                gmap[new_state] = GNode(q_new, curr_state, fNew, gNew, hNew, step);
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

void GlobalPlanner::print_primitives(vector<MotionPrimitive> mpd)
{
    for(int i=0;i< mpd.size(); ++i)
    {
        MotionPrimitive j=mpd[i];
        cout<<"------------------------------\n";
        vector<Global_State> jp = j.get_primitive();
        for(Global_State o : jp)
            cout<<"["<<o.x<<","<<o.y<<","<<o.theta<<"],"<<endl;
        cout<<" Cost= "<<cost_of_motion[i]<<endl;
    }
}


void GlobalPlanner::motion_primitive_writer(vector<MotionPrimitive> mpd, string file_name)
{
    // Converting the vector of MotionPrimitives into a 2d Vector.
    vector< vector<double>> pmap (num_steps, vector<double> {0});
    for(MotionPrimitive m : mpd)
    {
        vector<Global_State> mj = m.get_primitive();
        cout<<mj.size()<<endl;
        if (mj.size() < 8)
            continue;       // ignoring the motion primitives with the 8 steps curently
        for(int e=0;e<mj.size();++e)
        {
            pmap[e].push_back(mj[e].x);
            pmap[e].push_back(mj[e].y);
            pmap[e].push_back(mj[e].theta);
        }
    }

    // Writing the motion primitive 2d vector to a csv file
    ofstream ffr (file_name);
    for(int p =0;p<pmap.size();++p)
    {
        for(int q=1;q<pmap[p].size();++q)
            ffr<<pmap[p][q]<<", ";
        ffr<<endl;
    }
    ffr.close();    

}

void GlobalPlanner::publish_path(vector<Global_State> path, string file_name)
{
    ofstream ffr (file_name);
    for(Global_State sf : path)
        ffr<<sf.x<<", "<<sf.y<<", "<<sf.theta<<endl;
    ffr.close();
}

vector<MotionPrimitive> GlobalPlanner::startS_primitives()
{
    return motion_primitives;
}

// -----------------------------------------------------------------------------------------------------------------------------------------------
void OccGrid::generate_static_occ(parking box)
{
    //  Function to pre_compute the static occupancy grid from the parked vehicles in the frame
    vector<Global_State> v_loc = box.get_locs();
    vector<int> v_states = box.parking_state();

    for(int i=0; i<v_states.size();++i)
    {
        if(v_states[i]==0)  // parking box empty
            continue;
        vector<double> pl_xy = pBoxlim(v_loc[i]); 
        vector<int> obstacles = xy2i(pl_xy);
        // cout<<"("<<obstacles[0]<<","<<obstacles[2]<<"); ("<<obstacles[1]<<","<<obstacles[3]<<") \n";
        update_static_occ(obstacles, 1);
    }

}


void OccGrid::update_static_occ(vector<int> veh_i, int full)
{
    //  Function to update the occupancy grid values for the indices in vehXY
    // limits: (veh_i[0], vehh_i[2]) ; (veh_i[1], veh_h[3])
    for(int m=veh_i[0]; m<=veh_i[2]; ++m)
    {
        for(int n=veh_i[1]; n<=veh_i[3]; ++n)
        {
            occ_map[m][n] = full;
        }
    }
}

vector<double> OccGrid::pBoxlim(Global_State ploc)
{
    // Function to compute the parking box limits [x,y] 
    double wid = w/2;
    double x = ploc.x;
    double y = ploc.y;
    double thet = wrap2pi(ploc.theta);
    vector<double> xy;
    
    double ax = x + lf*cos(thet) + wid*sin(thet);
    double bx = x + lf*cos(thet) - wid*sin(thet);
    double cx = x - lb*cos(thet) - wid*sin(thet);
    double dx = x - lb*cos(thet) + wid*sin(thet);
    double ay = y + lf*sin(thet) - wid*cos(thet);
    double by = y + lf*sin(thet) + wid*cos(thet);
    double cy = y - lb*sin(thet) + wid*cos(thet);
    double dy = y - lb*sin(thet) - wid*cos(thet);

    if(thet == 0)
    {
        xy.push_back(cx);
        xy.push_back(bx);
        xy.push_back(dy);
        xy.push_back(cy);
    }
    else if(thet <= (0.1+PI/2))
    {
        xy.push_back(bx);
        xy.push_back(ax);
        xy.push_back(dy);
        xy.push_back(ay);
    }
    else if(thet <= (0.1 + PI))
    {
        xy.push_back(bx);
        xy.push_back(cx);
        xy.push_back(by);
        xy.push_back(ay);
    }
    else
    {
        xy.push_back(ax);
        xy.push_back(bx);
        xy.push_back(ay);
        xy.push_back(dy);
    }

    // xy.push_back(x - cos(thet)*(l/2) - sin(thet)*(w/2));
    // xy.push_back(x + cos(thet)*(l/2) + sin(thet)*(w/2));
    // xy.push_back(y - cos(thet)*(w/2) - sin(thet)*(l/2));
    // xy.push_back(y + cos(thet)*(w/2) + sin(thet)*(l/2));
    return xy;
}

vector<int> OccGrid::xy2i(vector<double> xy)
{
    vector<int> ind;
    ind.push_back(floor((xy[0] - xlim[0])/dx));
    ind.push_back(floor((xy[2] - ylim[0])/dy));
    ind.push_back(ceil((xy[1] - xlim[0])/dx));
    ind.push_back(ceil((xy[3] - ylim[0])/dy));
    return ind;
}

bool OccGrid::isEmpty(int xi, int yi)
{
    // returns 1 if the location (xi,yi) is empty; 0 otherwise.
    return !occ_map[xi][yi];       
}

vector<vector<int>> OccGrid::get_occmap()
{
    return occ_map;
}

void OccGrid::occ_map_publish(string file_name)
{
    ofstream ffr (file_name);
    for(int o=0; o<occ_map.size();++o)
    {
        for(int og : occ_map[o])
            ffr<<og<<" ,";
        ffr<<endl;
    }
}

// -----------------------------------------------------------------------------------------------------------------------

parking::parking()
{
    // Storing the prking box locations................
    parkX.push_back(Global_State(-47.61477279663086, 31.042869567871094, -1.57078873678579));
    parkX.push_back(Global_State(-13.505621910095215, -31.273136138916016, 1.5708025852234582));
    parkX.push_back(Global_State(-16.27899932861328, -31.273151397705078, 1.5708025852234582));
    parkX.push_back(Global_State(-19.057735443115234, -31.27314567565918, 1.5708025852234582));
    parkX.push_back(Global_State(-21.841434478759766, -31.27312660217285, 1.5708025852234582));
    parkX.push_back(Global_State(-24.631704330444336, -31.273109436035156, 1.5708025852234582));
    parkX.push_back(Global_State(-10.727664947509766, -31.273151397705078, 1.5708025852234582));
    parkX.push_back(Global_State(-7.950075626373291, -31.27320671081543, 1.5708025852234582));
    parkX.push_back(Global_State(-5.160816669464111, -31.273231506347656, 1.5708025852234582));
    parkX.push_back(Global_State(-2.4036195278167725, -31.273216247558594, 1.5708025852234582));
    parkX.push_back(Global_State(2.1477136611938477, -13.62131118774414, -0.0));
    parkX.push_back(Global_State(0.4022550880908966, -31.27322006225586, 1.5708025852234582));
    parkX.push_back(Global_State(24.663219451904297, 0.28649184107780457, -0.0));
    parkX.push_back(Global_State(24.663219451904297, 3.059866189956665, -0.0));
    parkX.push_back(Global_State(24.663219451904297, 5.83858585357666, -0.0));
    parkX.push_back(Global_State(24.663219451904297, 8.622309684753418, -0.0));
    parkX.push_back(Global_State(24.663219451904297, 11.412550926208496, -0.0));
    parkX.push_back(Global_State(24.66319465637207, -2.4914541244506836, -0.0));
    parkX.push_back(Global_State(24.663150787353516, -5.269047260284424, -0.0));
    parkX.push_back(Global_State(24.663150787353516, -8.058199882507324, -0.0));
    parkX.push_back(Global_State(24.663150787353516, -10.815412521362305, -0.0));
    parkX.push_back(Global_State(24.663150787353516, -13.621313095092773, -0.0));
    parkX.push_back(Global_State(-23.06333351135254, 0.28649425506591797, -0.0));
    parkX.push_back(Global_State(-23.06333351135254, 3.059868812561035, -0.0));
    parkX.push_back(Global_State(-23.06333351135254, 5.838588237762451, -0.0));
    parkX.push_back(Global_State(-23.06333351135254, 8.622312545776367, -0.0));
    parkX.push_back(Global_State(-23.06333351135254, 11.412553787231445, -0.0));
    parkX.push_back(Global_State(-23.063358306884766, -2.4914517402648926, -0.0));
    parkX.push_back(Global_State(-23.063398361206055, -5.269044876098633, -0.0));
    parkX.push_back(Global_State(-23.06340217590332, -8.058197021484375, -0.0));
    parkX.push_back(Global_State(-23.06340217590332, -10.815409660339355, -0.0));
    parkX.push_back(Global_State(2.147777795791626, 3.059868812561035, -0.0));
    parkX.push_back(Global_State(-23.063398361206055, -13.62131118774414, -0.0));
    parkX.push_back(Global_State(-48.98688888549805, 0.2864780128002167, -0.0));
    parkX.push_back(Global_State(-48.98688888549805, 3.0598528385162354, -0.0));
    parkX.push_back(Global_State(-48.98688888549805, 5.838579177856445, -0.0));
    parkX.push_back(Global_State(-48.98688888549805, 8.622303009033203, -0.0));
    parkX.push_back(Global_State(-48.98688888549805, 11.412550926208496, -0.0));
    parkX.push_back(Global_State(-48.98691177368164, -2.491468906402588, -0.0));
    parkX.push_back(Global_State(-48.98695373535156, -5.269053936004639, -0.0));
    parkX.push_back(Global_State(-48.98695755004883, -8.058206558227539, -0.0));
    parkX.push_back(Global_State(-48.98695755004883, -10.815412521362305, -0.0));
    parkX.push_back(Global_State(2.147777795791626, 5.838588237762451, -0.0));
    parkX.push_back(Global_State(-48.98695373535156, -13.621313095092773, -0.0));
    parkX.push_back(Global_State(-54.12901306152344, -2.4843921661376953, -3.1415891914803757));
    parkX.push_back(Global_State(-54.12900161743164, -5.257752418518066, -3.1415891914803757));
    parkX.push_back(Global_State(-54.12900161743164, -8.036470413208008, -3.1415891914803757));
    parkX.push_back(Global_State(-54.1290168762207, -10.820185661315918, -3.1415891914803757));
    parkX.push_back(Global_State(-54.129032135009766, -13.61042308807373, -3.1415891914803757));
    parkX.push_back(Global_State(-54.12899398803711, 0.2935601472854614, -3.1415891914803757));
    parkX.push_back(Global_State(-54.12893295288086, 3.071143627166748, -3.1415891914803757));
    parkX.push_back(Global_State(-54.12893295288086, 5.860317230224609, -3.1415891914803757));
    parkX.push_back(Global_State(-54.12893295288086, 8.617523193359375, -3.1415891914803757));
    parkX.push_back(Global_State(2.147777795791626, 8.622312545776367, -0.0));
    parkX.push_back(Global_State(-54.12893295288086, 11.423429489135742, -3.1415891914803757));
    parkX.push_back(Global_State(-28.20282745361328, -2.4843077659606934, -3.1415891914803757));
    parkX.push_back(Global_State(-28.202816009521484, -5.257676124572754, -3.1415891914803757));
    parkX.push_back(Global_State(-28.202816009521484, -8.036394119262695, -3.1415891914803757));
    parkX.push_back(Global_State(-28.202831268310547, -10.820122718811035, -3.1415891914803757));
    parkX.push_back(Global_State(-28.20284652709961, -13.61036205291748, -3.1415891914803757));
    parkX.push_back(Global_State(-28.20280647277832, 0.293643981218338, -3.1415891914803757));
    parkX.push_back(Global_State(-28.202749252319336, 3.0712268352508545, -3.1415891914803757));
    parkX.push_back(Global_State(-28.202749252319336, 5.860393524169922, -3.1415891914803757));
    parkX.push_back(Global_State(-28.202749252319336, 8.617599487304688, -3.1415891914803757));
    parkX.push_back(Global_State(2.147777795791626, 11.412553787231445, -0.0));
    parkX.push_back(Global_State(-28.202749252319336, 11.423492431640625, -3.1415891914803757));
    parkX.push_back(Global_State(-3.0148494243621826, -2.4842357635498047, -3.1415891914803757));
    parkX.push_back(Global_State(-3.0148396492004395, -5.25760555267334, -3.1415891914803757));
    parkX.push_back(Global_State(-3.0148396492004395, -8.036323547363281, -3.1415891914803757));
    parkX.push_back(Global_State(-3.0148544311523438, -10.820058822631836, -3.1415891914803757));
    parkX.push_back(Global_State(-3.014868974685669, -13.610297203063965, -3.1415891914803757));
    parkX.push_back(Global_State(-3.0148298740386963, 0.2937156558036804, -3.1415891914803757));
    parkX.push_back(Global_State(-3.0147714614868164, 3.071298837661743, -3.1415891914803757));
    parkX.push_back(Global_State(-3.0147714614868164, 5.860464572906494, -3.1415891914803757));
    parkX.push_back(Global_State(-3.0147714614868164, 8.617670059204102, -3.1415891914803757));
    parkX.push_back(Global_State(2.147754669189453, -2.4914517402648926, -0.0));
    parkX.push_back(Global_State(-3.0147714614868164, 11.42355728149414, -3.1415891914803757));
    parkX.push_back(Global_State(19.509239196777344, -2.4841694831848145, -3.1415891914803757));
    parkX.push_back(Global_State(19.509248733520508, -5.257540702819824, -3.1415891914803757));
    parkX.push_back(Global_State(19.509248733520508, -8.036258697509766, -3.1415891914803757));
    parkX.push_back(Global_State(19.509233474731445, -10.819997787475586, -3.1415891914803757));
    parkX.push_back(Global_State(19.509220123291016, -13.610236167907715, -3.1415891914803757));
    parkX.push_back(Global_State(19.509258270263672, 0.29378244280815125, -3.1415891914803757));
    parkX.push_back(Global_State(19.50931739807129, 3.0713627338409424, -3.1415891914803757));
    parkX.push_back(Global_State(19.50931739807129, 5.860528945922852, -3.1415891914803757));
    parkX.push_back(Global_State(19.50931739807129, 8.617734909057617, -3.1415891914803757));
    parkX.push_back(Global_State(2.1477136611938477, -5.269044876098633, -0.0));
    parkX.push_back(Global_State(19.50931739807129, 11.42361831665039, -3.1415891914803757));
    parkX.push_back(Global_State(7.5478339195251465, 31.41176414489746, -1.57078873678579));
    parkX.push_back(Global_State(10.321207046508789, 31.41177749633789, -1.57078873678579));
    parkX.push_back(Global_State(13.09992790222168, 31.411773681640625, -1.57078873678579));
    parkX.push_back(Global_State(15.883647918701172, 31.411766052246094, -1.57078873678579));
    parkX.push_back(Global_State(18.673912048339844, 31.411752700805664, -1.57078873678579));
    parkX.push_back(Global_State(4.769878387451172, 31.411779403686523, -1.57078873678579));
    parkX.push_back(Global_State(1.9922960996627808, 31.411842346191406, -1.57078873678579));
    parkX.push_back(Global_State(-0.7968673706054688, 31.411840438842773, -1.57078873678579));
    parkX.push_back(Global_State(-3.5540740489959717, 31.41183853149414, -1.57078873678579));
    parkX.push_back(Global_State(2.147712469100952, -8.058197021484375, -0.0));
    parkX.push_back(Global_State(-6.359940528869629, 31.411834716796875, -1.57078873678579));
    parkX.push_back(Global_State(-33.7070198059082, 31.042797088623047, -1.57078873678579));
    parkX.push_back(Global_State(-30.933645248413086, 31.04281234741211, -1.57078873678579));
    parkX.push_back(Global_State(-28.15492820739746, 31.04280662536621, -1.57078873678579));
    parkX.push_back(Global_State(-25.37120819091797, 31.042802810668945, -1.57078873678579));
    parkX.push_back(Global_State(-22.580942153930664, 31.042787551879883, -1.57078873678579));
    parkX.push_back(Global_State(-36.48496627807617, 31.04281234741211, -1.57078873678579));
    parkX.push_back(Global_State(-39.26255416870117, 31.042875289916992, -1.57078873678579));
    parkX.push_back(Global_State(-42.05170822143555, 31.042875289916992, -1.57078873678579));
    parkX.push_back(Global_State(-44.80891036987305, 31.042869567871094, -1.57078873678579));
    parkX.push_back(Global_State(2.1477112770080566, -10.815409660339355, -0.0));
    parkX.push_back(Global_State(2.147777795791626, 0.28649425506591797, -0.0));
}

void parking::emptylots(vector<int> lots)
{
    // Sets the lots with the given index to empty (available for parking)
    for(int i : lots)
        isfull[i] = 0;
}

vector<Global_State> parking::get_locs()
{
    return parkX;
}

Global_State parking::get_loc(int j)
{
    return parkX[j];
}

vector <int> parking::parking_state()
{
    return isfull;
}

bool parking::isAvailable(int j)
{
    return !isfull[j];
}

void parking::reserve_spot(vector <int> inds)
{
    // Function to set the parking spots with indices in "inds" as empty
    for(int i : inds)
        isfull[i] = 0;
}

// ------------------------------------------------------------------------------------------------

void GlobalPlanner::print_path(vector<Global_State> path)
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

// -------------------------------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------------------------------------

int main()
{
    double dx = 0.2, dy = 0.2;     // Grid discretization
    double v_des = 1.2;     // Desrired Velocity
    double l_car = 2.2;     // Wheelbase of the vehicle
    double delT = 0.1;      // delta time of the simulation
    double steer_limit = PI*61/180;

    clock_t start_t = clock();

    // Global_State startS = Global_State(-50, -30, 0);
    // Global_State goalS = Global_State(-13.5, -31.04, 3*PI/4);
    Global_State startS = Global_State(-15, 30, 3*PI/2);
    // Global_State goalS = Global_State(-3, -13.7, 0);   //0.40,-31.27,0);
    Global_State goalS = Global_State(-54.12901306152344, -2.4843921661376953, 0);        //{44}
    // Global_State goalS = Global_State(2.1477136611938477, -13.62131118774414, PI);      // {38}

    GlobalPlanner g_planner(startS, goalS, steer_limit, delT, v_des, l_car, dx, dy);

    vector<Global_State> vehicle_path;
    // Prcomputing the motion primitives
    g_planner.generate_motion_primitives();

    parking parkV;
    parkV.reserve_spot({59, 48, 39, 44, 10, 70}); // Setting parking lot as empty

    // instantanting the occupance grid...........
    OccGrid occ(dx, dy);
    occ.generate_static_occ(parkV);
    occ.occ_map_publish("occupancy.csv");

    // ---------Checking if the goal state is empty----------------------
    vector <int> ind = g_planner.xy2i(goalS);
    if(occ.isEmpty(ind[0], ind[1]))
        cout<<"Vehicle can be parked at the goal state \n";
    else
    {
        cout<<"Vehicle cannot be parked at the goal stare, Exiting........\n";
        return 0;
    }

    // g_planner.motion_primitive_writer(g_planner.startS_primitives(), "startS.csv");
    // vector <MotionPrimitive> pp = g_planner.transform_primitive(goalS);
    // cout<<"+++\n+++++++++\n+++++++++"<<endl;
    // g_planner.motion_primitive_writer(pp, "goalS.csv");
    
    // g_planner.print_primitives(pp);

    // Pre-Computing the 2D heuristic....
    g_planner.pre_compute2DH(startS, occ);

    // Searching for path to the goal...................................... 
    vehicle_path = g_planner.A_star(startS, goalS ,occ);
    // print_path(vehicle_path);
    g_planner.publish_path(vehicle_path, "waypoints.csv");

    
    cout<<" Time taken for computation : "<<(double)(clock() - start_t)/CLOCKS_PER_SEC<<" s"<<endl;
        
    return 0;
}   