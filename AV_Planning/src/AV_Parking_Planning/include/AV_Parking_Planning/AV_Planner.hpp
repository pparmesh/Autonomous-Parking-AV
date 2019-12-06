#ifndef _AV_PLANNER_H_
#define _AV_PLANNER_H_

#include "ros/ros.h"
#include <eigen3/Eigen/Dense>
#include "AV_Parking_Planning/global_planner.h"
#include "AV_Parking_Planning/local_planner.hpp"
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <trajectory_msgs/JointTrajectory.h>

class AV_Planner
{
  public:
    // Data members for Global Planner
    Global_State m_global_start;
    Global_State m_global_goal;
    
    //
    double m_ctrl_freq; // Controller frequency for local planner
    vector<Global_State> m_global_plan;
    Eigen::MatrixXd m_goal_region_plan;
    bool m_near_goal;
    int m_goal_ind;

  public:
    AV_Planner();
    // Make callback for Carla publisherto get start, goal and occupancy
    ros::Publisher m_traj_pub;
    void set_global_plan(Global_State startS, Global_State goalS);
    void set_final_plan(); // populates m_reference_trajectory
    void publishTrajectory();
    void plan_to_goal(Global_State a, Global_State b);
    double wrap2pi(double angle);

    void run();

};

// @TODO:: 
// 1. Add ROS subscriber that subscribes to Occ Grid data

#endif