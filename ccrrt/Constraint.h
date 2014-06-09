#ifndef CONSTRAINT_H
#define CONSTRAINT_H

#include <Eigen/Geometry>
#include <vector>

namespace ccrrt {

class Constraint
{
public:
    
    typedef enum {
        
        INVALID = 0,
        AT_RISK,
        VALID,
        STUCK
        
    } validity_t;
    
    virtual validity_t getCostGradient(Eigen::VectorXd& gradient, 
                                       const Eigen::VectorXd& parent,
                                       const Eigen::VectorXd& config,
                                       const Eigen::VectorXd& target) = 0;

    virtual validity_t getValidity(const Eigen::VectorXd& config) = 0;
    
    virtual size_t constraintDimension() const = 0;
    
    std::vector<size_t> elevatorDimensions;
    
};

} // namespace ccrrt

#endif // CONSTRAINT_H
