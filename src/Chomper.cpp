
#include "../ccrrt/Chomper.h"
#include <iostream>

using namespace ccrrt;
using namespace Eigen;

Chomper::Chomper()
{
    _constraint = NULL;
    opt_type = VELOCITY;
}

const Trajectory& Chomper::getTrajectory() const
{
    return _trajectory;
}

const Constraint* const Chomper::getConstraint() const
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
    h.resize(_constraint->constraintDimension());

    _generate_A(trajectory.state_space, trajectory.waypoints);
    Ainv.resize(nm,nm);
    H.resize(c,nm);
    HAinvHt_inv.resize(c,c);
    Ainv_Ht_HAinvHt_inv.resize(nm,c);
}

void Chomper::_generate_A(int state_space, int waypoints)
{
    int nm = state_space*waypoints;
    A.resize(nm,nm);

    assert( VELOCITY == opt_type && "Acceleration is not supported yet!");
    if(VELOCITY == opt_type)
    {
//        size_t entries = (3*waypoints-2)*state_space;
//        A.reserve(entries);
        std::cout << "about to reserve" << std::endl;
        A.reserve(VectorXd::Constant(nm,3));

        int k_bottom, k_top;
        std::cout << "starting loop" << std::endl;
        for(int i=0; i<waypoints; ++i)
        {
            k_bottom = -1;
            k_top = 1;

            if(i==0)
                k_bottom = 0;

            if(i==waypoints-1)
                k_top = 0;

            for(int j=0; j<state_space; ++j)
            {
                for(int k=k_bottom; k<=k_top; ++k)
                {
                    std::cout << i << ", " << j << ", " << k << " -> ";
                    std::cout << "(" << j+state_space*k+state_space*i << ", " << j+state_space*i
                              << ")" << std::endl;
                    if(k==0)
                    {
                        A.insert(j+state_space*k+state_space*i, j+state_space*i) = 2;
                    }
                    else
                    {
                        A.insert(j+state_space*k+state_space*i, j+state_space*i) = -1;
                    }
                }
            }
        }
    }

    A.makeCompressed();
    std::cout << A << std::endl;
}

Constraint::validity_t Chomper::iterate()
{
    return Constraint::INVALID;
}


