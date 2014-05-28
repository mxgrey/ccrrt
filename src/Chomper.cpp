
#include "../ccrrt/Chomper.h"
#include <iostream>

using namespace ccrrt;
using namespace Eigen;

Chomper::Chomper()
{
    _constraint = NULL;
    opt_type = VELOCITY;
    _last_wp_size = 0;
    _last_ss_size = 0;
    alpha = 0.5;
}

const Trajectory& Chomper::getTrajectory() const
{
    return _trajectory;
}

Constraint* Chomper::getConstraint() const
{
    return _constraint;
}

void Chomper::initialize(const Trajectory& trajectory, Constraint* constraint)
{
    _trajectory = trajectory;
    _constraint = constraint;
    
    size_t nm = trajectory.state_space*trajectory.waypoints;
    size_t c = _constraint->constraintDimension();

    Df.resize(nm);
    h.resize(c);
    e.resize(trajectory.state_space*(trajectory.waypoints+1));
    delta.resize(nm);

    _generate_A(trajectory.state_space, trajectory.waypoints);
    _build_e();

    H.resize(c,nm);
    HAinvHt_inv.resize(c,c);
    HAinvHt.resize(c,c);
    Ainv_Ht_HAinvHt_inv.resize(nm,c);

    _validity = _constraint->getCost(h, _trajectory);
}

void Chomper::_generate_A(int state_space, int waypoints)
{
    if(_last_wp_size==waypoints && _last_ss_size==state_space)
        return;

    _last_wp_size = waypoints;
    _last_ss_size = state_space;

    int _nm = state_space*waypoints;
    A.resize(_nm,_nm);
    K.resize(state_space*(waypoints+1),_nm);

    assert( VELOCITY == opt_type && "Acceleration is not supported yet!");
    if(VELOCITY == opt_type)
    {
//        size_t entries = (3*waypoints-2)*state_space;
//        A.reserve(entries);
        K.reserve(VectorXd::Constant(_nm,2));
        A.reserve(VectorXd::Constant(_nm,3));

        int r_bottom, r_top;
        for(int i=0; i<waypoints; ++i)
        {
            r_bottom = -1;
            r_top = 1;

            if(i==0)
                r_bottom = 0;

            if(i==waypoints-1)
                r_top = 0;

            for(int j=0; j<state_space; ++j)
            {
                for(int r=r_bottom; r<=r_top; ++r)
                {
                    int x1 = j+state_space*r+state_space*i;
                    int x2 = j+state_space*i;
//                    std::cout << i << ", " << j << ", " << r << " -> ";
//                    std::cout << "(" << x1 << ", " << x2 << ")" << std::endl;
                    if(r==0)
                    {
                        A.insert(x1, x2) = 2;
                    }
                    else
                    {
                        A.insert(x1, x2) = -1;
                    }
                }

                for(int r=0; r<=1; ++r)
                {
                    int k1 = j+state_space*r+state_space*i;
                    int k2 = j+state_space*i;

                    if(r==0)
                    {
                        K.insert(k1,k2) = 1;
                    }
                    else if(r==1)
                    {
                        K.insert(k1,k2) = -1;
                    }
                }
            }
        }
    }

    A.makeCompressed();
//    std::cout << A << std::endl;

//    std::cout << "KtK:\n" << K.transpose()*K << std::endl;

//    std::cout << "\nK:\n" << K << std::endl;
//    llt.compute(A);
//    Eigen::MatrixXd lltInv = ((Eigen::MatrixXd)llt.matrixL()).inverse();
//    Ainv = lltInv.transpose()*lltInv;

//    std::cout << "\nL:\n" << llt.matrixL() << std::endl;

    Ainv = ((Eigen::MatrixXd)A).inverse();
//    std::cout << "\nInverse: \n" << Ainv << std::endl;
//    std::cout << "\nTimes each other:\n" << A*Ainv << std::endl;
}

Constraint::validity_t Chomper::iterate(bool quit_if_valid)
{
    if(quit_if_valid && _validity != Constraint::INVALID)
        return _validity;

    _constraint->getJacobian(H, _trajectory);
//    std::cout << "\nH:\n" << H << std::endl;
    HAinvHt = H*Ainv*H.transpose();
//    std::cout << "\nHAinvHt:\n" << HAinvHt << std::endl;
    if(fabs(HAinvHt.determinant()) > 1e-6)
    {
        HAinvHt_inv = HAinvHt.inverse();
        Ainv_Ht_HAinvHt_inv = Ainv*H.transpose()*HAinvHt_inv;
    }
    else
    {
        HAinvHt_inv.setZero();
        Ainv_Ht_HAinvHt_inv.setZero();
    }

//    std::cout << "\nHAinvHt_inv:\n" << HAinvHt_inv << std::endl;
//    std::cout << "\nAinv_Ht_HAinvHt_inv:\n" << Ainv_Ht_HAinvHt_inv.transpose() << std::endl;

//    std::cout << "A: " << A.rows() << "x" << A.cols() << std::endl;
//    std::cout << "xi:" << _trajectory.xi.rows() << std::endl;
//    std::cout << "Kt:" << K.transpose().rows() << "x" << K.transpose().cols() << std::endl;
//    std::cout << "e: " << e.rows() << std::endl;

//    std::cout << "A:\n" << A << std::endl;
//    std::cout << "\nxi:\n" << _trajectory.xi.transpose() << std::endl;
//    std::cout << "\nKt:\n" << K.transpose() << std::endl;
//    std::cout << "\ne:\n" << e.transpose() << std::endl;

    Df = A*_trajectory.xi+K.transpose()*e;
//    std::cout << "\nDf:\n" << Df.transpose() << std::endl;


    delta = -alpha*(Ainv - Ainv_Ht_HAinvHt_inv*H*Ainv)*Df
            -Ainv_Ht_HAinvHt_inv*h;

//    std::cout << "\ndelta:\n" << delta.transpose() << std::endl;

    _trajectory.xi += delta;
    _validity = _constraint->getCost(h, _trajectory);

//    return -Df;
    return _validity;

//    return delta;
}

void Chomper::_build_e()
{
    e.block(0,0,_trajectory.state_space,1) = -_trajectory.start;
    e.block(_last_wp_size*_last_ss_size,0,_trajectory.state_space,1) = _trajectory.end;
}


