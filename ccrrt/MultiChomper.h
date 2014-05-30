#ifndef MULTICHOMPER_H
#define MULTICHOMPER_H

#include "../ccrrt/Chomper.h"
#include <list>

typedef std::list<Eigen::VectorXd> WaypointList;
typedef std::list<Eigen::VectorXd>::iterator Waypoint;

namespace ccrrt {

class MultiChomper : public Chomper
{
public:
    
    MultiChomper();

    bool run(const Trajectory& trajectory,
                    Constraint* constraint,
                    const Eigen::VectorXd& min,
                    const Eigen::VectorXd& max,
                    double max_step_size);

    size_t max_attempts;
    
protected:

    bool _go();
    bool _check_step_sizes();
    Eigen::VectorXd _interpolate(const Eigen::VectorXd& start,
                                 const Eigen::VectorXd& end);
    
    Eigen::VectorXd _start;
    Eigen::VectorXd _end;
    WaypointList _waypoints;
    
    double _max_step;

};

} // namespace ccrrt

#endif // MULTICHOMPER_H
