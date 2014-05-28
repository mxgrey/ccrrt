
#include "../ccrrt/Chomper.h"
#include "../ccrrt/CircleConstraint.h"
#include "../ccrrt/Drawer.h"
#include <iostream>

using namespace ccrrt;
using namespace Eigen;

int main(int argc, char* argv[])
{
    CircleConstraint circle(Vector2d(0,0), 2, 0.1);

    Trajectory traj;
    traj.state_space = 2;

//    traj.start.resize(2);
//    traj.start << -3, 0;
//    traj.end.resize(2);
//    traj.end << 3, 1;
//    traj.xi.resize(2);
//    traj.xi << -1.50, 1.00;
////    traj.xi << -1.50, 3.00;


    traj.start.resize(2);
    traj.start << -3, 0;
    traj.end.resize(2);
    traj.end << 3, 1;
    traj.xi.resize(8);
    traj.xi << 0, 1.7, 1.2, 1.2, 2.04, 0, 3, 1;



    traj.waypoints = traj.xi.size()/2;


    Chomper chomp;
    chomp.initialize(traj, &circle);
    chomp.alpha = 0.5;

    Drawer draw;
    draw.draw_circle(circle);
    Eigen::VectorXd last_xi = chomp.getTrajectory().xi;
    
    size_t counter=0, max = 10000;
    while(chomp.iterate() == Constraint::INVALID)
    {
        ++counter;
        if(counter > max)
        {
            std::cout << "Hit maximum number of iterations (" << max << ")" << std::endl;
            break;
        }
        
        if(counter==1000)
        {
            draw.draw_trajectory(chomp.getTrajectory(),osg::Vec4(0.1,0.9,0.1,1.0));
        }
    }

//    for(size_t i=0; i<10; ++i)
//    {
////        chomp.iterate();
//        draw.draw_vector(chomp.iterate(),last_xi);
//        last_xi = chomp.getTrajectory().xi;
//    }

    draw.draw_trajectory(chomp.getTrajectory());
    draw.draw_trajectory(traj, osg::Vec4(0.7,0.7,0.7,1.0));



    draw.run();

    return 0;
}
