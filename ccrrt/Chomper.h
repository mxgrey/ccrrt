#ifndef CHOMPER_H
#define CHOMPER_H

#include "../ccrrt/Constraint.h"

namespace ccrrt {

class Chomper
{
public:
    
    Chomper();

    void initialize(const Trajectory& trajectory, Constraint* constraint);
    Constraint::validity_t iterate();
    
    const Trajectory& getTrajectory() const;
    const Constraint* const getConstraint() const; // Do we need so many consts?
    
protected:
    
    Trajectory _trajectory;
    Constraint* _constraint;
    
    Eigen::VectorXd Df;
    Eigen::VectorXd h;
    
    Eigen::MatrixXd A; // TODO: Make sparse matrix
    Eigen::MatrixXd Ainv;
    Eigen::MatrixXd H;
    Eigen::MatrixXd HAinvHt_inv;
    Eigen::MatrixXd Ainv_Ht_HAinvHt_inv;
};

} // namespace ccrrt

#endif // CHOMPER_H
