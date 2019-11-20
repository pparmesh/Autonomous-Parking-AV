#include "global_planner.h"
#include <iostream>
#include <math.h>
#include <vector>
#include <cfloat>
#include <unordered_map>
#include <set>

using namespace std;

GlobalPlanner::GlobalPlanner(Global_State start_state, Global_State goal_state, double max_steering_angle, double dt,
double desire_vel, double car_length)
{
    m_start_state = start_state;
    m_goal_state = goal_state;
    m_max_steering_angle = max_steering_angle;
    m_dt = dt;
    m_desired_velocity = desire_vel;
    m_car_length= car_length;
}

vector<Global_State> GlobalPlanner::get_motion_primitive(Global_State current_state, double steering_angle)
{
    vector<Global_State> state_vector;
    Global_State state = current_state;
    for (double t=0; t<1; t++) // Change depending on allowed time for motion
    {
        state.x = state.x + m_desired_velocity*cos(state.theta)*m_dt;
        state.y = state.y + m_desired_velocity*sin(state.theta)*m_dt;
        state.theta = state.theta + m_desired_velocity/m_car_length*tan(m_max_steering_angle)*m_dt;
        state_vector.push_back(state);
    }
    return state_vector;
}

Global_State GlobalPlanner::get_new_state(Global_State current_state, double str_angle)
{
    vector<Global_State> primitive = get_motion_primitive(current_state,str_angle);
    return primitive[primitive.size()-1];
}

int GlobalPlanner::get_state_hash(Global_State state)
{
    return 0;
}

bool GlobalPlanner::is_valid_primitive(vector<Global_State> motion_primitve)
{
    return 1;
}

vector<Global_State> GlobalPlanner::solutionPath(int goal)
{
    vector<int> solution_vector;
    int curr_state = goal;
    while (global_graph[curr_state].parent!=-1)
    {
        solution_vector.push_back(curr_state);
        curr_state = global_graph[curr_state].parent;
    }
    vector<Global_State> solution_path;
    for(int i=0;i<solution_vector.size();i++)
    {
        solution_path.push_back(global_graph[solution_vector[i]].state);
    }
    return solution_path;
}

vector<Global_State> GlobalPlanner::A_star(Global_State start_state, Global_State goal_state)
{
    vector<Global_State> path;
    // Initialize the closed list to keep track of expanded nodes
    unordered_map <int, bool> closed_list;
    int start = get_state_hash(start_state);
    int goal = get_state_hash(goal_state);


    // A map to store the cell info such as f,g,h values of nodes
    

    // Inititalize the start cell. It has no parents and its f, g & h values are 0
    global_graph[start].state = start_state;
    global_graph[start].f = 0;
    global_graph[start].g = 0;
    global_graph[start].h = 0;
    global_graph[start].parent = -1; // start cell as its parent as itself


    // Implement the open list to keep track of states to expand using a set
    // It is a set of f_COORINATE, i.e it has location of state and its f value
    set<f_COORDINATE> open_list; 
    // Add my start cell to my open list
    open_list.insert(f_COORDINATE (0.0, start_state)); 


    // Expand till open list is not empty
    while(!open_list.empty() && closed_list[goal]!=true)
    {   
        // Pick index with lowest f value from open list. Set will help in doing this as it is ordered.
        //Put this in closed list.
        // Find neighbors of my current index and find f values only if they are not in closed list.
        // If they are not in closed list, find their f-values. If they are in the open list with a larger
        // f-value then update it, otherwise add this index to the open list. 
        // Loop till goal state has not been expanded.

        // Get index from openlist. Pop the first value from the open list.
        f_COORDINATE q = *open_list.begin();
        // Remove it from the open list
        open_list.erase(open_list.begin());
        // Get index of this node
        Global_State q_current = q.st;
        int current_state = get_state_hash(q_current);
        closed_list[current_state] = true; 
        double str_angle = -m_max_steering_angle;
        vector<double> valid_actions;
        while (str_angle<m_max_steering_angle)
        {
            vector<Global_State> motion_primitive = get_motion_primitive(q_current,str_angle);
            if (is_valid_primitive(motion_primitive))
                valid_actions.push_back(str_angle);
            str_angle = str_angle + 0.1;
        }
        // Loop through motion primitives
        for (auto str : valid_actions)
        {
            Global_State q_new = get_new_state(q_current,str);

            int fNew, gNew, hNew; // Variables used to find f, g & h values
            int new_state = get_state_hash(q_new);
            // Only proceed if it is not in closed list
            if (closed_list[new_state] != true)
            {
                // Compute fNew, gNew, hNew.
                gNew = global_graph[current_state].g + 1;
                hNew = 0;
                fNew = gNew + hNew;

                if (global_graph[new_state].f == FLT_MAX || global_graph[new_state].f > fNew)
                {
                    open_list.insert(f_COORDINATE(fNew, q_new));
                    global_graph[new_state].f = fNew;
                    global_graph[new_state].g = gNew;
                    global_graph[new_state].h = hNew;
                    global_graph[new_state].parent = current_state;
                }
            }
        }    
    }
    if (closed_list[goal])
    {
        cout<<"Solution Found "<<endl;
        return solutionPath(goal);
    }
    return path;
}

int main()
{
    return 0;
}