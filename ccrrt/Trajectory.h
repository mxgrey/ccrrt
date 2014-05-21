#ifndef CCRRT_TRAJECTORY_H
#define CCRRT_TRAJECTORY_H

#include <Eigen/Geometry>

namespace ccrrt {

class Trajectory
{
public:

    Eigen::VectorXd start;
    Eigen::VectorXd xi;
    Eigen::VectorXd end;

    size_t state_space;
    size_t waypoints;

};

} // namespace ccrrt

#endif // CCRRT_TRAJECTORY_H
