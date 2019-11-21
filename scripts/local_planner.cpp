
#include "local_planner.hpp"
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <math.h>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Vector3d;
using namespace std;

// TODO: Need velocity information

LocalPLanner::LocalPLanner(double ctrl_freq, Global_State node_start, Global_State node_end)
{
    m_ctrl_freq = ctrl_freq; // Controller frequency
    m_node_start = node_start;
    m_node_end = node_end;
}
// Define member functions
double LocalPLanner::getCtrlFreq() { return m_ctrl_freq; } // Getter for control freq

void LocalPLanner::setCtrlFreq(double new_ctrl_freq) { m_ctrl_freq = new_ctrl_freq; } // Setter for control freq


MatrixXd LocalPLanner::homogenousTransWorldEgo(VehicleState state)
{
    MatrixXd homo_trans(3,3);
    homo_trans<<cos(_ego_state.yaw), -sin(_ego_state.yaw), _ego_state.x,
                sin(_ego_state.yaw), cos(_ego_state.yaw), _ego_state.y,
                0, 0, 1;
    return homo_trans;
}
double LocalPLanner::getMaxPlanningTime()
{
    double planning_time_x = abs(m_node_end.x - m_node_start.x)/m_node_start.vel_x;
    double planning_time_y = abs(m_node_end.y - m_node_start.y)/m_node_start.vel_y;
    return max(planning_time_x,planning_time_y);
}

// Member function for generating evasive trajectory
MatrixXd LocalPLanner::getPolynomialCoefficients()
{
    // Boundary vals is [xi,yi,xf,yf,vxi,vyi,axi,ayi]
    // minimum jerk trajectory is a 5th order polynomial
    // y = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
    // Given initial and final values in pos, vel and acc 
    
    // Position
    double xi  = m_node_start.x;
    double yi = m_node_start.y;

    double xf = m_node_end.x;
    double yf = m_node_end.y;
    
    //Velocity
    double vxi  = m_node_start.vel_x;
    double vyi = m_node_start.vel_y;

    double vxf = m_node_end.vel_x;
    double vyf = m_node_end.vel_y;

    //Acceleration
    double axi  = m_node_start.acc_x;
    double ayi = m_node_start.acc_y;

    double axf = m_node_end.acc_x;
    double ayf = m_node_end.acc_y;
    

    return coeffs;
}

MatrixXd LocalPLanner::getEvasiveTrajectory(VehicleState _ego_state, double y_final)
{
    MatrixXd coeffs = getPolynomialCoefficients(_ego_state,y_final);

    double dt = 1 / m_ctrl_freq;
    int num_steps = (int)(getMaxPlanningTime(_ego_state) * m_ctrl_freq);
    MatrixXd reference_trajectory(num_steps, 4);
    for (int i = 0; i < num_steps; i++) {
        VectorXd time_step(6, 1);
        time_step << 1, dt, pow(dt, 2), pow(dt, 3), pow(dt, 4), pow(dt, 5); // Define time vector
        VectorXd tmp = coeffs * time_step; // Get traj info x,y vx, vy
        reference_trajectory.row(i) = tmp.transpose();
        dt = dt + 1/m_ctrl_freq;
    }
    MatrixXd world_vel(3,num_steps);
    for (int j=0; j<num_steps; j++)
    {
        world_vel.col(j)<<reference_trajectory(j,2),reference_trajectory(j,3),0;
    }

    MatrixXd homo_trans = homogenousTransWorldEgo(_ego_state);
    // cout<<"world vel: "<<homo_trans.inverse()<<'\n'<<endl;
    MatrixXd ego_vel = homo_trans.inverse()*world_vel;
    for (int k=0;k<num_steps;k++)
    {
        reference_trajectory(k,2) = ego_vel(0,k);
        reference_trajectory(k,3) = ego_vel(1,k);
    }
    // cout<<"traj: "<<reference_trajectory<<'\n'<<endl;
    return reference_trajectory;
}