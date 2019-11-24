#ifndef LOCAL_PLANNER_H
#define LOCAL_PLANNER_H

#include "local_waypoints.hpp"
#include <iostream>
#include <math.h>
#include <eigen3/Eigen/Dense>

using Eigen::MatrixXd;
using Eigen::VectorXd;

using namespace std;

class LocalPlanner {
    // Define member variables
private:
    double m_ctrl_freq; // Controller frequency
    NodeState m_node_start;
    NodeState m_node_end;

public:

    LocalPlanner(double ctrl_freq, NodeState node_start, NodeState node_end);

    // Define member functions
    double getCtrlFreq(); // Getter for control freq
    void setCtrlFreq(double new_ctrl_freq); // Setter for control freq
    double getMaxPlanningTime(); // Get time of execution of trajectory
    MatrixXd getPolynomialCoefficients(); // coeffs of polynomials
    MatrixXd generateLocalPlan(); // create local plan
};

#endif