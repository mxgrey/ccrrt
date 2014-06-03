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
    
    size_t getJacobian(Eigen::MatrixXd &J, const Trajectory &traj);
    
    validity_t getCost(Eigen::VectorXd &cost, const Trajectory &traj);
    
    validity_t getValidity(const Trajectory &traj);
    
    size_t constraintDimension() const;
    
    inline const Eigen::Isometry2d& getTf() const { return _tf; }
    
protected:
    
    size_t _basicJacobian(Eigen::Matrix<double,1,2>& J,
                          const Trajectory& traj,
                          size_t waypoint);
    
    double _basicCostGrad(Eigen::Vector2d& grad_c, const Eigen::Vector2d& config);
    
    validity_t _basicValidity(const Eigen::Vector2d& config);
    
    Eigen::Isometry2d _tf;
    Eigen::Isometry2d _tfinv;
    
    Eigen::Vector2d _center;
    double _angle;
    
};


} // namespace ccrrt


#endif // BLOCKCONSTRAINT_H
