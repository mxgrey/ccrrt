
#include "../ccrrt/Constraint.h"
#include "../ccrrt/CircleConstraint.h"

#include <iostream>

using namespace ccrrt;
using namespace Eigen;

void apply_constraint(Constraint& constraint, Trajectory& traj)
{
    MatrixXd H;
    constraint.getJacobian(H, traj);
    std::cout << H << std::endl;
}

void get_cost(Constraint& constraint, Trajectory& traj)
{
    VectorXd cost;
    constraint.getCost(cost, traj);
    std::cout << cost.transpose() << std::endl;
}

//void get_cost(Constraint& constraint, )

int main(int argc, char* argv[])
{
    CircleConstraint circle(Vector2d(0,0), 2, 0.1);
    Trajectory traj;
    traj.state_space = 2;

//    traj.start.resize(2);
//    traj.start << 0, 1;

//    traj.end.resize(2);
//    traj.end << 0, -1;

//    traj.xi.resize(8);
//    traj.xi << 0, 1.7, 2.04, 0, 1.2, 1.2, 3, 1;

    traj.start.resize(2);
    traj.start << -3, 0;
    traj.end.resize(2);
    traj.end << 3, 0;

    traj.xi.resize(2);
    traj.xi << 0 + 1, 1.00;

    traj.waypoints = traj.xi.size()/2;

//    apply_constraint(circle, traj);
    get_cost(circle, traj);
    
    return 0;
}
