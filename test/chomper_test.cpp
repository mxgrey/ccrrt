
#include "../ccrrt/Chomper.h"
#include "../ccrrt/CircleConstraint.h"
#include <iostream>

using namespace ccrrt;

int main(int argc, char* argv[])
{
    Trajectory traj;
    traj.state_space = 16;
    traj.waypoints = 4;

    CircleConstraint circle;

    Chomper chomp;
    chomp.initialize(traj, &circle);

    return 0;
}
