#ifndef LINECONSTRAINT_H
#define LINECONSTRAINT_H

#include "Constraint.h"

namespace ccrrt {

class LineConstraint : public Constraint
{
public:

    LineConstraint(const Eigen::Vector2d& mStart, const Eigen::Vector2d& mEnd,
                   double mWidth, double mTolerance = 1e-4);

    Eigen::Vector2d start;
    Eigen::Vector2d end;
    double width;
    double tolerance;
    double parab_factor;

    size_t getJacobian(Eigen::MatrixXd &J, const Trajectory &traj);

    validity_t getCost(Eigen::VectorXd &cost, const Trajectory &traj);

    validity_t getValidity(const Trajectory &traj);

    size_t constraintDimension() const;

protected:

    size_t _basicJacobian(Eigen::Matrix<double,1,2>& J,
                          const Trajectory& traj,
                          size_t waypoint);

    size_t _basicCostGrad(Eigen::Vector2d& grad_c, const Eigen::Vector2d& config);

    double _basicCost(const Eigen::Vector2d& config);

};

} // namespace ccrrt


#endif // LINECONSTRAINT_H
