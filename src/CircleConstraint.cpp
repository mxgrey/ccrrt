
#include "../ccrrt/CircleConstraint.h"

using namespace ccrrt;

CircleConstraint::CircleConstraint(const Eigen::Vector2d &center, double radius) :
    _center(center),
    _radius(radius)
{
    
}

bool CircleConstraint::fillJacobian(Eigen::MatrixXd &H,
                                    const Eigen::VectorXd &xi, 
                                    size_t numWaypoints)
{
    assert( xi.size()%numWaypoints == 0 );
    
    size_t config_size = xi.size()/numWaypoints;
    assert( config_size == 2 );
    
    Eigen::Vector2d config;
    Eigen::Vector2d diff;
    for(size_t i=0; i<numWaypoints; ++i)
    {
        config[0] = xi[0+2*i];
        config[1] = xi[1+2*i];
        
        diff = config - _center;
//        if(diff.norm() > _radius)
    }
    
    return true;
}

Constraint::validity_t CircleConstraint::configValidity(Eigen::VectorXd& lambda,
                                                        const Eigen::VectorXd &xi, 
                                                        size_t numWaypoints)
{
    
    return VALID;
}

size_t CircleConstraint::constraintDimension() const
{
    return 1;
}
