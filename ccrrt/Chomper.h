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

    virtual void initialize(const Trajectory& trajectory, Constraint* constraint);
    double alpha;
    
    virtual Constraint::validity_t iterate(bool quit_if_valid=false);
    
    const Trajectory& getTrajectory() const;
    const Constraint* const getConstraint() const; // Do we need so many consts?

    path_optimization_t opt_type;
    
protected:
    
    Trajectory _trajectory;
    Constraint* _constraint;
    Constraint::validity_t _validity;
    
    Eigen::VectorXd Df;
    Eigen::VectorXd h;
    Eigen::VectorXd e;
    Eigen::VectorXd delta;
    
    Eigen::SparseMatrix<double> A;
    Eigen::MatrixXd Ainv;
    Eigen::SparseMatrix<double> K;
//    Eigen::SimplicialLLT<Eigen::SparseMatrix<double>, Eigen::Lower > llt;
    Eigen::MatrixXd H;
    Eigen::MatrixXd HAinvHt;
    Eigen::MatrixXd HAinvHt_inv;
    Eigen::MatrixXd Ainv_Ht_HAinvHt_inv;

    void _generate_A(int state_space, int waypoints);
    void _build_e();

    int _last_wp_size;
    int _last_ss_size;
    int _nm;
};

} // namespace ccrrt

#endif // CHOMPER_H
