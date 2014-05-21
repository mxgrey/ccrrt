#ifndef CONSTRAINT_H
#define CONSTRAINT_H

#include <Eigen/Geometry>

namespace ccrrt {

class Constraint
{
public:
    
    typedef enum {
        
        INVALID = 0,
        AT_RISK,
        VALID
        
    } validity_t;
    
    virtual bool fillJacobian(Eigen::MatrixXd& H, 
                              const Eigen::VectorXd& xi, 
                              size_t numWaypoints);
    
    virtual validity_t configValidity(Eigen::VectorXd& lambda, 
                                      const Eigen::VectorXd& xi, 
                                      size_t numWaypoints);
    
    virtual size_t constraintDimension() const = 0;
    
};

} // namespace ccrrt

#endif // CONSTRAINT_H
