#include "AV_Parking_Planning/AV_Planner.hpp"
#include "AV_Parking_Planning/local_planner.hpp"
#include "AV_Parking_Planning/global_planner.h"
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <trajectory_msgs/JointTrajectory.h>
#include "ros/ros.h"
#include <eigen3/Eigen/Dense>
#include <unordered_map>
#include <vector>
using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Vector3d;


AV_Planner::AV_Planner() // May need to change constructor
{
  // Global Planner Parameters
  m_global_plan = {};

  // Local Planner Parameters
  m_ctrl_freq = 10;
  m_near_goal = false;
}


void AV_Planner::set_global_plan(Global_State startS, Global_State goalS)
{
    double dx = 0.2, dy = 0.2;     // Grid discretization
    double v_des = 1.2;     // Desrired Velocity
    double l_car = 2.2;     // Wheelbase of the vehicle
    double delT = 0.1;      // delta time of the simulation
    double steer_limit = PI*61/180;

    GlobalPlanner g_planner(startS, goalS, steer_limit, delT, v_des, l_car, dx, dy);

    vector<Global_State> vehicle_path;
    // Prcomputing the motion primitives
    g_planner.generate_motion_primitives();

    parking parkV;
    parkV.reserve_spot({59, 48, 39, 44, 10, 70}); // Setting parking lot as empty

    // instantanting the occupance grid...........
    OccGrid occ(dx, dy);
    occ.generate_static_occ(parkV);
    // occ.occ_map_publish("occupancy.csv");

    // ---------Checking if the goal state is empty----------------------
    vector <int> ind = g_planner.xy2i(goalS);
    if(occ.isEmpty(ind[0], ind[1]))
        cout<<"Vehicle can be parked at the goal state \n";
    else
    {
        cout<<"Vehicle cannot be parked at the goal stare, Exiting........\n";
        return;
    }

    g_planner.pre_compute2DH(startS, occ);

    // Searching for path to the goal...................................... 
    vehicle_path = g_planner.A_star(startS, goalS ,occ);
    m_global_plan = vehicle_path;

    return;
  
}
void AV_Planner::plan_to_goal(Global_State pre_goal, Global_State goal)
{
  cout<<"Planning to goal"<<endl;
  NodeState pre_goal_node = {pre_goal.x, pre_goal.y, pre_goal.theta, 2, 0, 0, 0, 0};
  NodeState goal_node = {goal.x, goal.y, goal.theta, 0, 0, 0, 0, 0};
  LocalPlanner loc(m_ctrl_freq, pre_goal_node, goal_node);
  MatrixXd coef = loc.getPolynomialCoefficients();
  cout<<coef<<endl;
  m_goal_region_plan = loc.generateLocalPlan();
  cout<<"DONE"<<endl;
  cout<<"Local Plan Size: "<<m_goal_region_plan.rows();
}

void AV_Planner::publishTrajectory()
{
    trajectory_msgs::JointTrajectory total_traj;
    // if (!m_near_goal)
    for (int i=0; i<m_global_plan.size();i++)
    {
      // cout<<"plan: "<<m_global_plan[i].x<<" "<<m_global_plan[i].y<<endl;
      trajectory_msgs::JointTrajectoryPoint point;
      point.positions = {m_global_plan[i].x, m_global_plan[i].y};
      point.velocities = {2,0};      
      total_traj.points.push_back(point);

    }
    for (int i=0; i<m_goal_region_plan.rows();i++)
    {
        trajectory_msgs::JointTrajectoryPoint point;
        point.positions = {m_goal_region_plan(i,0),m_goal_region_plan(i,1)};
        point.velocities = {m_goal_region_plan(i,2),m_goal_region_plan(i,3)};
        total_traj.points.push_back(point);
    }
    cout<<"Traj size: "<<total_traj.points.size();
    m_traj_pub.publish(total_traj);
}

void AV_Planner::run()
{
  // Get occupancy grid along with start and goal locations
    Global_State startS = Global_State(-15, 30, 3*PI/2);
    Global_State goalS = Global_State(-54.12901306152344, -2.4843921661376953, 0);
    set_global_plan(startS, goalS);
    plan_to_goal(m_global_plan[m_global_plan.size()-5], goalS); // Change when local plan is needed
    // plan_to_goal(startS, goalS);
    
  // Run global planner for vehicle and get set global state waypoints
  // Run local planner on global plan
  // Publish waypoints
    publishTrajectory();
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "Carla_Parking_Planner");

  ros::NodeHandle n;
  AV_Planner planner_obj; 

  planner_obj.m_traj_pub = n.advertise<trajectory_msgs::JointTrajectory>("/planner/trajectory", 1);
  planner_obj.run();

  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    int a = 1;
    if (a==1){
      a++;
    }
    // NodeState strt = {0,0,0,1,1,0,0,0};
    // NodeState goal = {2,2,0,1,1,0,0,0};
    // LocalPlanner loc(5,strt,goal);
    // MatrixXd coef = loc.getPolynomialCoefficients();
    // MatrixXd ref = loc.generateLocalPlan();
    // publishTrajectory(ref,control_pub);
    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}