
#include "../ccrrt/LineConstraint.h"

using namespace ccrrt;
using namespace Eigen;

LineConstraint::LineConstraint(const Vector2d &mStart,
                               const Vector2d &mEnd,
                               double mWidth,
                               double mTolerance) :
    start(mStart),
    end(mEnd),
    width(mWidth),
    tolerance(mTolerance)
{
    parab_factor = 0.1;
}


size_t LineConstraint::getJacobian(MatrixXd &J, const Trajectory &traj)
{
    assert(traj.state_space == 2);

    J.resize(constraintDimension(), traj.xi.size()); // 1 x nm

    Matrix<double, 1, 2> Jtemp;
    size_t rank_check=0;
    for(size_t i=0; i<traj.waypoints; ++i)
    {
        rank_check += _basicJacobian(Jtemp, traj, i);
        J.block(0,2*i,1,2) = Jtemp;
    }

    if(rank_check > 0)
        return 1;

    return 0;
}

size_t LineConstraint::_basicJacobian(Eigen::Matrix<double, 1, 2> &J,
                                      const Trajectory &traj,
                                      size_t waypoint)
{
    Vector2d config = Vector2d(traj.xi[2*waypoint],traj.xi[2*waypoint+1]);

    Vector2d grad_c;
    size_t result = _basicCostGrad(grad_c, config);

    J = grad_c.transpose();

    return result;
}

size_t LineConstraint::_basicCostGrad(Vector2d& grad_c, const Vector2d& config)
{
    Eigen::Vector2d p = config - start;
    Eigen::Vector2d v = end - start;
    double v_norm = v.norm();
    double v_dot_p = v.dot(p);
    Eigen::Vector2d c = v_dot_p/v_norm * v;
    double h = width/2;

    if( 0 < v_dot_p && v_dot_p < v_norm )
    {
        Eigen::Vector2d s = p - c;
        double s_norm = s.norm();
        if( fabs(tolerance) < s_norm &&  s_norm <= fabs(h) )
        {
            if( fabs(parab_factor*h) < s_norm )
            {
                grad_c = s.normalized();
                return 1;
            }
            else
            {
                grad_c = s/fabs(parab_factor*h);
                return 1;
            }
        }
        else
        {
            grad_c.setZero();
            return 0;
        }
    }
    else
    {
        grad_c.setZero();
        return 0;
    }

    return 0;
}

Constraint::validity_t LineConstraint::getCost(VectorXd &cost, const Trajectory &traj)
{
    cost.resize(1);

    cost[0] = 0;

    const VectorXd& xi = traj.xi;
    for(size_t i=0; i<traj.waypoints; ++i)
    {
        cost[0] += _basicCost(Vector2d(xi[2*i],xi[1+2*i]));
    }

    if(cost[0] > 0)
        return INVALID;

    return VALID;
}

double LineConstraint::_basicCost(const Vector2d &config)
{

    Eigen::Vector2d p = config - start;
    Eigen::Vector2d v = end - start;
    double v_norm = v.norm();
    double v_dot_p = v.dot(p);
    Eigen::Vector2d c = v_dot_p/v_norm * v;
    double h = width/2;

    if( 0 < v_dot_p && v_dot_p < v_norm )
    {
        Eigen::Vector2d s = p - c;
        double s_norm = s.norm();
        if( fabs(tolerance) < s_norm &&  s_norm <= fabs(h) )
        {
            if( fabs(parab_factor*h) < s_norm )
            {
                return s_norm - 0.5*fabs(parab_factor*h);
            }
            else
            {
                return 0.5/fabs(parab_factor*h)*s_norm*s_norm;
            }
        }
        else
        {
            return 0;
        }
    }
    else
    {
        return 0;
    }

    return 0;
}

Constraint::validity_t LineConstraint::getValidity(const Trajectory &traj)
{
    Eigen::VectorXd cost;
    return getCost(cost, traj);
}

size_t LineConstraint::constraintDimension() const
{
    return 1;
}


