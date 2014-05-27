#ifndef CHOMPER_H
#define CHOMPER_H

//#define EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET

#include "../ccrrt/Constraint.h"
#include <Eigen/Sparse>

namespace ccrrt {

class Chomper
{
public:

    typedef enum {

        VELOCITY = 0,
        ACCELERATION

    } path_optimization_t;
    
    Chomper();

    void initialize(const Trajectory& trajectory, Constraint* constraint);
    Constraint::validity_t iterate();
    
    const Trajectory& getTrajectory() const;
    const Constraint* const getConstraint() const; // Do we need so many consts?

    path_optimization_t opt_type;
    
protected:
    
    Trajectory _trajectory;
    Constraint* _constraint;
    
    Eigen::VectorXd Df;
    Eigen::VectorXd h;
    
    Eigen::SparseMatrix<double> A; // TODO: Make sparse matrix
    Eigen::MatrixXd Ainv;
    Eigen::SimplicialLLT<Eigen::SparseMatrix<double>, Eigen::Lower > llt;
    Eigen::MatrixXd H;
    Eigen::MatrixXd HAinvHt_inv;
    Eigen::MatrixXd Ainv_Ht_HAinvHt_inv;

    void _generate_A(int state_space, int waypoints);
};

} // namespace ccrrt

#endif // CHOMPER_H
