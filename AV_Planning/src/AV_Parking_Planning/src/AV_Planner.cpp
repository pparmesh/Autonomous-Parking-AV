#include "AV_Parking_Planning/AV_Planner.hpp"
#include "AV_Parking_Planning/local_planner.hpp"
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <trajectory_msgs/JointTrajectory.h>
#include "ros/ros.h"
#include <eigen3/Eigen/Dense>
#include <unordered_map>
#include <vector>


AV_Planner::AV_Planner() // May need to change constructor
{
  // Global Planner Parameters
  m_global_plan = {};

  // Local Planner Parameters
  m_ctrl_freq = 10;
}


void set_global_plan()
{
  
}
void AV_Planner::set_final_plan()
{
  // Iterate over global plan nodes and create a final 
  // trajectory for the vehicle
  // Iterate to get size of plan
  int plan_size(0);
  unordered_map<int,MatrixXd> local_traj_map;
  vector<int> plan_sizes;
  for (int i=0; i<m_global_plan.size()-1; i++) // Loop used to get size of plan at compile time
  {
    LocalPlanner::LocalPlanner local_planner(m_ctrl_freq,m_global_plan[i],m_global_plan[i+1]);
    MatrixXd ref = local_planner.generateLocalPlan();
    local_traj_map[i] = ref;
    plan_sizes.push_back(ref.rows());
    plan_size = plan_size + ref.rows();
  }
  MatrixXd temp_plan(plan_size,4); // Initialize plan at compile time
  int idx(0);
  for (int j=0; j<m_global_plan.size()-1; j=j+plan_sizes[j])
  {
    temp_plan.block<plan_sizes[j],4>(j,0) = local_traj_map[j]; // Populate matrix (waypoints for every two successive nodes)
  }

  m_reference_trajectory = temp_plan;
}



void AV_Planner::publishTrajectory()
{
    trajectory_msgs::JointTrajectory total_traj;
    for (int i=0; i<m_reference_trajectory.rows();i++)
    {
        trajectory_msgs::JointTrajectoryPoint point;
        point.positions = {m_reference_trajectory(i,0),m_reference_trajectory(i,1)};
        point.velocities = {m_reference_trajectory(i,2),m_reference_trajectory(i,3)};
        total_traj.points.push_back(point);
    }
    m_traj_pub.publish(total_traj);
}

void AV_Planner::run()
{
  // Get occupancy grid along with start and goal locations
  // Run global planner for vehicle and get set global state waypoints
  // Run local planner on global plan
  // Publish waypoints
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "Carla_Parking_Planner");

  ros::NodeHandle n;
  AV_Planner planner_obj(); // May need to change constructor

  ros::Publisher control_pub = n.advertise<trajectory_msgs::JointTrajectory>("/planner/trajectory", 1);

  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    planner_obj.run();
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