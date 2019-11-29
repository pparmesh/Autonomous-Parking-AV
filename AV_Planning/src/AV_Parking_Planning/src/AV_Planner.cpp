#include "ros/ros.h"
#include <eigen3/Eigen/Dense>
#include "AV_Parking_Planning/local_planner.hpp"
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <trajectory_msgs/JointTrajectory.h>

void publishTrajectory(MatrixXd ref_traj,ros::Publisher control_pub)
{
    trajectory_msgs::JointTrajectory total_traj;
    for (int i=0; i<ref_traj.rows();i++)
    {
        trajectory_msgs::JointTrajectoryPoint point;
        point.positions = {ref_traj(i,0),ref_traj(i,1)};
        point.velocities = {ref_traj(i,2),ref_traj(i,3)};
        total_traj.points.push_back(point);
    }
    control_pub.publish(total_traj);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");

  ros::NodeHandle n;

  ros::Rate loop_rate(10);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  ros::Publisher control_pub = n.advertise<trajectory_msgs::JointTrajectory>("/planner/trajectory", 1);
  int count = 0;
  while (ros::ok())
  {
    NodeState strt = {0,0,0,1,1,0,0,0};
    // cout<<strt.x;
    NodeState goal = {2,2,0,1,1,0,0,0};
    LocalPlanner loc(5,strt,goal);
    MatrixXd coef = loc.getPolynomialCoefficients();
    MatrixXd ref = loc.generateLocalPlan();
    publishTrajectory(ref,control_pub);
    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}