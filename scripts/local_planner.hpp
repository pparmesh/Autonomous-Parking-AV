#ifndef QUINTIC_POLYNOMIAL_GENERATION_H
#define QUINTIC_POLYNOMIAL_GENERATION_H

#include <iostream>
#include <math.h>
#include <eigen3/Eigen/Dense>
#include "delta_planning_controls/vehicle_state.hpp"

using Eigen::MatrixXd;
using Eigen::VectorXd;

using namespace std;

struct Global_State
{
    double x; 
    double y; 
    double theta;
    Global_State(): x(0), y(0), theta(0)
    {}
    Global_State(double a, double b, double c): x(a), y(b), theta(c)
    {}
    Global_State(const Global_State& h)
    {
        this->x = h.x;
        this->y = h.y;
        this->theta = h.theta;
    }
    bool operator==(const Global_State& t) const
    {
        return (this->x==t.x && this->y==t.y && this->theta==t.theta);
    }
};
class LocalPlanner {
    // Define member variables
private:
    double m_ctrl_freq; // Controller frequency
    Global_State m_node_start;
    Global_State m_node_end;

public:

    LocalPlanner(double ctrl_freq, Global_State node_start, Global_State node_end);

    // Define member functions
    double getCtrlFreq(); // Getter for control freq
    void setCtrlFreq(double new_ctrl_freq); // Setter for control freq
};

#endif