
#include "../ccrrt/ConstrainedRRT.h"

using namespace ccrrt;

ConstrainedRRT::ConstrainedRRT(int maxTreeSize, 
                               double maxStepSize, 
                               double collisionCheckStepSize) :
    RRTManager(maxTreeSize, maxStepSize, collisionCheckStepSize)
{
    _constraints = NULL;
}

bool ConstrainedRRT::collisionChecker(JointConfig& config, const JointConfig& parentConfig)
{
    if(_constraints==NULL)
        return true;
    
    assert( config.size() == parentConfig.size() 
            && "Collision checker detected a mismatch in configuration sizes!" );
    
    assert( config.size() > 0 
            && "Collision checker detected a size of 0 for the config" );
    
    double dist = (parentConfig-config).norm();
    _col.state_space = config.size();
    _col.waypoints = ceil(dist/collisionCheckStepSize_);
    _col.start = parentConfig;
    _col.end = config;
    _col.xi.resize(_col.state_space*_col.waypoints);
    
//    std::cout << " --- Rows:" << _col.xi.rows() << "\tCols:" << _col.xi.cols()
//              << "\tstate_space:" << _col.state_space << "\twaypoints:" << _col.waypoints
//              << "\tdist:" << dist
//              << "\tconfig.size:" << config.size()
//              << std::endl;
//    std::cout << "parent: " << parentConfig.transpose()
//              << "\t|\tchild: " << config.transpose() << std::endl;
    for(size_t i=0; i<_col.waypoints; ++i)
    {
//        std::cout << "StartRow:" << i*_col.state_space << "\t"
//                  << "BlockRows:" << _col.state_space << "\t"
//                  << "StartCol:" << 0 << "\t"
//                  << "BlockCols:" << 1 << "\t"
//                  << std::endl;
        _col.xi.block(i*_col.state_space,0,_col.state_space,1) =
                (config-parentConfig)*(double)(i+1)/(double)(_col.waypoints)
                + parentConfig;
    }
    
    if(_col.waypoints==0)
    {
        _col.waypoints = 1;
        _col.xi = config;
    }
    
    return _constraints->getCost(_cost, _col) != Constraint::INVALID;
}

void ConstrainedRRT::setConstraints(Constraint *constraints)
{
    _constraints = constraints;
}
