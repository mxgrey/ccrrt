
#include "../ccrrt/Constraint.h"

#include <iostream>

using namespace ccrrt;
using namespace Eigen;

double Constraint::getSpeed(const Trajectory &traj, size_t waypoint)
{
    double d = 0;

    size_t s = traj.state_space;

    const VectorXd& xi = traj.xi;
    const VectorXd& start = traj.start;
    const VectorXd& end = traj.end;
    if(waypoint == 0 && traj.waypoints == 1)
    {
        d = (end - xi).norm() + (xi - start).norm();
    }
    else if(waypoint == 0)
    {
        d = (xi.block(s,0,s,1) - xi.block(0,0,s,1)).norm()
                + (xi.block(0,0,s,1) - start).norm();
    }
    else if(waypoint==traj.waypoints-1)
    {
        d = (traj.end - xi.block(s*(waypoint),0,s,1)).norm()
                + (xi.block(s*(waypoint),0,s,1) - xi.block(s*(waypoint-1),0,s,1)).norm();
    }
    else
    {
        d = (xi.block(s*(waypoint+1),0,s,1) - xi.block(s*(waypoint),0,s,1)).norm()
                + (xi.block(s*(waypoint),0,s,1) - xi.block(s*(waypoint-1),0,s,1)).norm();
    }

    return d/(2*traj.waypoints);
}

void Constraint::getVelocity(Eigen::VectorXd& vel, const Trajectory& traj, size_t waypoint)
{
    size_t s = traj.state_space;

    const VectorXd& xi = traj.xi;
    const VectorXd& start = traj.start;
    const VectorXd& end = traj.end;
    if(waypoint == 0 && traj.waypoints == 1)
    {
        vel = (end - xi) + (xi - start);
    }
    else if(waypoint == 0)
    {
        vel = (xi.block(s,0,s,1) - xi.block(0,0,s,1))
                + (xi.block(0,0,s,1) - start);
    }
    else if(waypoint==traj.waypoints-1)
    {
        vel = (traj.end - xi.block(s*(waypoint),0,s,1))
                + (xi.block(s*(waypoint),0,s,1) - xi.block(s*(waypoint-1),0,s,1));
    }
    else
    {
        vel = (xi.block(s*(waypoint+1),0,s,1) - xi.block(s*(waypoint),0,s,1))
                + (xi.block(s*(waypoint),0,s,1) - xi.block(s*(waypoint-1),0,s,1));
    }

    vel = vel/(2*traj.waypoints);
}
