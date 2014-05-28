

#include "../ccrrt/MultiChomper.h"
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

    traj.start.resize(2);
    traj.start << -3, 0;
    traj.end.resize(2);
    traj.end << -1, 2;
    traj.xi.resize(2);
    traj.xi << 1.50, -2.00;
//    traj.xi << -1.50, 3.00;

    traj.waypoints = traj.xi.size()/2;


    MultiChomper multichomp;
    multichomp.alpha = 0.5;
    multichomp.run(traj, &circle, 0.2);

    Drawer draw;
    draw.draw_circle(circle);
    draw.draw_trajectory(multichomp.getTrajectory());
    draw.draw_trajectory(traj, osg::Vec4(0.7,0.7,0.7,1.0));

    draw.run();

    return 0;
}
