
#include "../ccrrt/ConstrainedRRT.h"

using namespace ccrrt;

ConstrainedRRT::ConstrainedRRT(int maxTreeSize, 
                               double maxStepSize, 
                               double collisionCheckStepSize) :
    RRTManager(maxTreeSize, maxStepSize, collisionCheckStepSize)
{
    _constraint = NULL;
}

bool ConstrainedRRT::collisionChecker(JointConfig& config, const JointConfig& parentConfig)
{
    if(_constraint==NULL)
        return true;
    
    double dist = (parentConfig-config).norm();
    _col.state_space = config.size();
    _col.waypoints = ceil(dist/collisionCheckStepSize_);
    _col.start = parentConfig;
    _col.end = config;
    _col.xi.resize(_col.state_space*_col.waypoints);
    
    for(size_t i=0; i<_col.waypoints; ++i)
    {
        _col.xi.block(i*_col.state_space,0,_col.state_space,1) =
                (config-parentConfig)*(double)(i+1)/(double)(_col.waypoints)
                + parentConfig;
    }
    
    return _constraint->getCost(_cost, _col) != Constraint::INVALID;
}

void ConstrainedRRT::setConstraint(Constraint *constraint)
{
    _constraint = constraint;
}
