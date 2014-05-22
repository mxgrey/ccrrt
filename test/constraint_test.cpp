
#include "../ccrrt/Drawer.h"
#include "../ccrrt/Constraint.h"
#include "../ccrrt/CircleConstraint.h"

#include <iostream>

using namespace ccrrt;
using namespace Eigen;

MatrixXd apply_constraint(Constraint& constraint, Trajectory& traj)
{
    MatrixXd J;
    constraint.getJacobian(J, traj);
    std::cout << J << std::endl;
    return J;
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
    std::cout << "Eigen version: world " << EIGEN_WORLD_VERSION
              << ", major " << EIGEN_MAJOR_VERSION << ", minor " << EIGEN_MINOR_VERSION
              << std::endl;

    CircleConstraint circle(Vector2d(0,0), 2, 0.1);
    Trajectory traj;
    traj.state_space = 2;

//    traj.start.resize(2);
//    traj.start << 0, 1;

//    traj.end.resize(2);
//    traj.end << 0, -1;
    
    traj.start.resize(2);
    traj.start << -3, 0;
    traj.end.resize(2);
    traj.end << 3, -0.5;
    traj.xi.resize(8);
    traj.xi << 0, 1.7, 1.2, 1.2, 2.04, 0, 3, 1;


//    traj.xi.resize(2);
//    traj.xi << -1.50, 1.00;

    traj.waypoints = traj.xi.size()/2;

    
    Drawer draw;
    draw.draw_trajectory(traj);
    draw.draw_circle(circle);
    
    
    MatrixXd J = apply_constraint(circle, traj);
    
    for(int i=0; i<J.size()/2; ++i)
    {
        draw.draw_vector(-Vector2d(J(0,2*i),J(0,2*i+1)),
                         Vector2d(traj.xi[2*i],traj.xi[2*i+1]));
    }
    draw.run();
    
    return 0;
}
