
#include "../ccrrt/MultiChomper.h"
#include "../ccrrt/CircleConstraint.h"
#include "../ccrrt/ConstraintGroup.h"
#include "../ccrrt/Drawer.h"
#include <iostream>

using namespace ccrrt;
using namespace Eigen;

int main(int argc, char* argv[])
{
    ConstraintGroup group;

    std::vector<CircleConstraint*> circles;
    circles.push_back(new CircleConstraint(Vector2d(0,0), 2, 0.1));
    circles.push_back(new CircleConstraint(Vector2d(3.1,0), 1, 0.1));

    for(size_t i=0; i<circles.size(); ++i)
    {
        group.addConstraint(circles[i]);
    }

    Trajectory traj;
    traj.state_space = 2;

    traj.start.resize(2);
    traj.start << -3, 0;
    traj.end.resize(2);
    traj.end << 3.4, 1.2;
    traj.xi.resize(2);
    traj.xi << 2.2, -0.20;
//    traj.xi << -1.50, 3.00;

    traj.waypoints = traj.xi.size()/2;


    MultiChomper multichomp;
    multichomp.alpha = 0.5;
    multichomp.run(traj, &group, 0.2);

    Drawer draw;

    for(size_t i=0; i<circles.size(); ++i)
    {
        draw.draw_circle(*circles[i]);
    }

    draw.draw_trajectory(multichomp.getTrajectory());
    draw.draw_trajectory(traj, osg::Vec4(0.7,0.7,0.7,1.0));

    draw.run();

    return 0;


    return 0;
}
