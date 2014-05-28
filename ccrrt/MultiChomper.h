#ifndef MULTICHOMPER_H
#define MULTICHOMPER_H

#include "../ccrrt/Chomper.h"
#include <vector>



namespace ccrrt {

class MultiChomper : public Chomper
{
public:
    
    MultiChomper();
    
    void initialize(const Trajectory& trajectory, Constraint* constraint);
    
    
    
protected:
    
    std::vector<Eigen::VectorXd> _waypoints;
    size_t _max_wps;
    
};

} // namespace ccrrt

#endif // MULTICHOMPER_H
