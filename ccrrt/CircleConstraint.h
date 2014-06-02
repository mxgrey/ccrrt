#ifndef CIRCLECONSTRAINT_H
#define CIRCLECONSTRAINT_H

#include "Constraint.h"

namespace ccrrt {

class CircleConstraint : public Constraint
{
public:
    
    CircleConstraint(const Eigen::Vector2d& mCenter = Eigen::Vector2d::Zero(),
                     double mRadius = 0, double mBuffer = 0);
    
    size_t getJacobian(Eigen::MatrixXd& J, const ccrrt::Trajectory& traj);
    
    validity_t getCost(Eigen::VectorXd& cost, 
                       const ccrrt::Trajectory& traj);

    validity_t getValidity(const Trajectory &traj);
    
    size_t constraintDimension() const;
    
    Eigen::Vector2d center;
    double radius;
    double buffer;

protected:

    double _basicCost(const Eigen::Vector2d& config);
    void _basicCostGrad(Eigen::Vector2d& grad_c, const Eigen::Vector2d& config);
    size_t _basicJacobian(Eigen::Matrix<double,1,2>& J,
                          const ccrrt::Trajectory& traj,
                          size_t waypoint);
    validity_t _basicValidity(const Eigen::Vector2d& config);

};

} // namespace ccrrt

#endif // CIRCLECONSTRAINT_H
