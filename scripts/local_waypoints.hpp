#ifndef _LOCAL_WAYPOINTS_H_
#define _LOCAL_WAYPOINTS_H_

#include <cmath>

struct NodeState {
    double x;
    double y;
    double yaw;
    double vx;
    double vy;
    double yaw_rate;
    double acc_x;
    double acc_y;

    NodeState() : x(0.0), y(0.0), yaw(0.0), vx(0.0), vy(0.0), yaw_rate(0.0), acc_x(0.0), acc_y(0.0)
    {
    }

    NodeState(double val_x, double val_y, double val_yaw, double val_vx, double val_vy, double val_yaw_rate, 
        double val_acc_x, double val_acc_y) : x(val_x), y(val_y), yaw(val_yaw), vx(val_vx), vy(val_vy), 
                                                yaw_rate(val_yaw_rate), acc_x(val_acc_y), acc_y(val_acc_y)
    {
    }

    double getDistance(double pos_x, double pos_y)
    {
        return sqrt(pow((pos_x - x), 2) + pow((pos_y - y), 2));
    }

    double getVelocity()
    {
        return sqrt(pow(vx, 2) + pow(vy, 2));
    }
};

#endif 