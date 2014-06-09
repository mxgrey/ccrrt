#ifndef BLOCKCONSTRAINT_H
#define BLOCKCONSTRAINT_H

#include "Constraint.h"

namespace ccrrt {

class BlockConstraint : public Constraint
{
public:
    
    BlockConstraint(const Eigen::Vector2d& mCenter, double mAngle,
                    double mWidth, double mHeight, double mBuffer=0.05);
    
    void setLocation(const Eigen::Vector2d& mCenter, double mAngle);
    
    Eigen::Vector2d scales;
    double buffer;
    
    validity_t getCostGradient(Eigen::VectorXd& gradient,
                               const Eigen::VectorXd& parent,
                               const Eigen::VectorXd& config,
                               const Eigen::VectorXd& target);
    
    validity_t getValidity(const Eigen::VectorXd &config);
    
    size_t constraintDimension() const;
    
    inline const Eigen::Isometry2d& getTf() const { return _tf; }
    
protected:
    
    double _basicCostGrad(Eigen::Vector2d& grad_c, const Eigen::Vector2d& config);
    
    validity_t _basicValidity(const Eigen::Vector2d& config);
    
    Eigen::Isometry2d _tf;
    Eigen::Isometry2d _tfinv;
    
    Eigen::Vector2d _center;
    double _angle;
    
};


} // namespace ccrrt


#endif // BLOCKCONSTRAINT_H
