
#include "../ccrrt/MultiChomper.h"

using namespace ccrrt;

void MultiChomper::initialize(const Trajectory &trajectory, Constraint *constraint)
{
    _max_wps = pow(2,ceil(log(trajectory.waypoints)/log(2)));
    _waypoints.resize(_max_wps);
    
    
}
