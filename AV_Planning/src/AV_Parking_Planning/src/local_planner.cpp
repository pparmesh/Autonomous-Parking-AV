#include "AV_Parking_Planning/local_planner.hpp"
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <math.h>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Vector3d;
using namespace std;


LocalPlanner::LocalPlanner(double ctrl_freq, NodeState node_start, NodeState node_end)
{
    m_ctrl_freq = ctrl_freq; // Controller frequency
    m_node_start = node_start;
    m_node_end = node_end;
}
// Define member functions
double LocalPlanner::getCtrlFreq() { return m_ctrl_freq; } // Getter for control freq

void LocalPlanner::setCtrlFreq(double new_ctrl_freq) { m_ctrl_freq = new_ctrl_freq; } // Setter for control freq

double LocalPlanner::getMaxPlanningTime()
{
    // double planning_time_x = abs(m_node_end.x - m_node_start.x)/m_node_start.vx;
    // double planning_time_y = abs(m_node_end.y - m_node_start.y)/m_node_start.vy;
    // return max(planning_time_x,planning_time_y);
    double planning_time_x = abs(m_node_end.x - m_node_start.x)/m_node_start.vx;
    return planning_time_x;

}

// Member function to get coeffs of polynomials for trajectory
MatrixXd LocalPlanner::getPolynomialCoefficients()
{
    // Boundary vals is [xi,yi,xf,yf,vxi,vyi,axi,ayi]
    // minimum jerk trajectory is a 5th order polynomial
    // y = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
    // Given initial and final values in pos, vel and acc 
    double T = getMaxPlanningTime();
    cout<<"TIME: "<<T<<endl;
    // Position
    double xi  = m_node_start.x;
    double yi = m_node_start.y;

    double xf = m_node_end.x;
    double yf = m_node_end.y;
    
    //velocity
    double vxi  = m_node_start.vx;
    double vyi = m_node_start.vy;

    double vxf = m_node_end.vx;
    double vyf = m_node_end.vy;

    //Acceleration
    double axi  = m_node_start.acc_x;
    double ayi = m_node_start.acc_y;

    double axf = m_node_end.acc_x;
    double ayf = m_node_end.acc_y;

    MatrixXd coeffs(4, 6); // Matrix of coefficients
    // Populate matrix coeffs
    // a0 = xi
    // a1 = vi
    // a2 = Ai/2
    // a3 = -(20*xi-20*xf + 8*T*vf + 12*T*vi -Af*T^2 + 3*Ai*T^2)/(2*T^3)
    // a4 = (30*xi- 30*xf + 14*T*vf + 16*T*vi - 2*Af*T^2 + 3*Ai*T^2)/(2*T^4)
    // a5 = -(12*xi-12*xf + 6*T*vf + 6*T*vi - Af*T^2 + Ai*T^2)/(2*T^5)

    // Pose x
    coeffs(0,0) = xi;
    coeffs(0,1) = vxi;
    coeffs(0,2) = axi/2;
    coeffs(0,3) = -(20*xi - 20*xf + 8*T*vxf + 12*T*vxi - axf*T*T + 3*axi*T*T)/(2*pow(T,3));
    coeffs(0,4) = (30*xi- 30*xf + 14*T*vxf + 16*T*vxi - 2*axf*T*T + 3*axi*T*T)/(2*pow(T,4));
    coeffs(0,5) = -(12*xi-12*xf + 6*T*vxf + 6*T*vxi - axf*T*T + axi*T*T)/(2*pow(T,5));
    
    // Pose y
    coeffs(1,0) = yi;
    coeffs(1,1) = vyi;
    coeffs(1,2) = ayi/2;
    coeffs(1,3) = -(20*yi - 20*yf + 8*T*vyf + 12*T*vyi - ayf*T*T + 3*ayi*T*T)/(2*pow(T,3));
    coeffs(1,4) = (30*yi- 30*yf + 14*T*vyf + 16*T*vyi - 2*ayf*T*T + 3*ayi*T*T)/(2*pow(T,4));
    coeffs(1,5) = -(12*yi-12*yf + 6*T*vyf + 6*T*vyi - ayf*T*T + ayi*T*T)/(2*pow(T,5));

    // velocity in x
    coeffs(2, 0) = coeffs(0, 1);
    coeffs(2, 1) = 2 * coeffs(0, 2);
    coeffs(2, 2) = 3 * coeffs(0, 3);
    coeffs(2, 3) = 4 * coeffs(0, 4);
    coeffs(2, 4) = 5 * coeffs(0, 5);
    coeffs(2, 5) = 0;

    // velocity in y
    coeffs(3, 0) = coeffs(1, 1);
    coeffs(3, 1) = 2 * coeffs(1, 2);
    coeffs(3, 2) = 3 * coeffs(1, 3);
    coeffs(3, 3) = 4 * coeffs(1, 4);
    coeffs(3, 4) = 5 * coeffs(1, 5);
    coeffs(3, 5) = 0;

    return coeffs;
}
// Member function to generate local trajectory waypoints
MatrixXd LocalPlanner::generateLocalPlan()
{
    MatrixXd coeffs = getPolynomialCoefficients();

    double dt = 1 / m_ctrl_freq;
    int num_steps = (int)(getMaxPlanningTime() * m_ctrl_freq);
    MatrixXd reference_trajectory(num_steps, 4);
    for (int i = 0; i < num_steps; i++) {
        VectorXd time_step(6, 1);
        time_step << 1, dt, pow(dt, 2), pow(dt, 3), pow(dt, 4), pow(dt, 5); // Define time vector
        VectorXd tmp = coeffs * time_step; // Get traj info x,y vx, vy
        reference_trajectory.row(i) = tmp.transpose();
        dt = dt + 1/m_ctrl_freq;
    }
    return reference_trajectory;
}

// int main()
// {
//     int a = 1;
//     NodeState strt = {0,0,0,1,1,0,0,0};
//     // cout<<strt.x;
//     NodeState goal = {5,5,0,0,0,0,0,0};
//     LocalPlanner loc(5,strt,goal);
//     MatrixXd coef = loc.getPolynomialCoefficients();
//     MatrixXd ref = loc.generateLocalPlan();
//     cout<<ref;
//     return 0;
// }