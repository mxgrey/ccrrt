
#include "../ccrrt/BlockConstraint.h"
#include <iostream>

using namespace ccrrt;
using namespace Eigen;

static inline double sign(double x)
{
    return x == 0? 0 :
            x > 0? 1 : -1;
}

BlockConstraint::BlockConstraint(const Vector2d &mCenter, double mAngle, double mWidth, 
                                 double mHeight, double mBuffer) :
    scales(mWidth/2,mHeight/2),
    buffer(mBuffer)
{
    setLocation(mCenter, mAngle);
}

void BlockConstraint::setLocation(const Vector2d &mCenter, double mAngle)
{
    _tf = Isometry2d::Identity();
    _tf.translate(mCenter);
    _tf.rotate(mAngle);
    
    _tfinv = _tf.inverse();
}

Constraint::validity_t BlockConstraint::getCostGradient(VectorXd& gradient,
                                                        const VectorXd& ,
                                                        const VectorXd& config,
                                                        const VectorXd& )
{
    Vector2d grad2d;
    Vector2d config2d = config.block<2,1>(0,0);
    
    _basicCostGrad(grad2d, config2d);
    gradient = 0.1*VectorXd(grad2d);
//    std::cout << gradient.transpose() << std::endl;
    return _basicValidity(config2d);
}

Constraint::validity_t BlockConstraint::getValidity(const Eigen::VectorXd &config)
{
    return _basicValidity(Vector2d(config[0],config[1]));
}

Constraint::validity_t BlockConstraint::_basicValidity(const Vector2d &config)
{
    Vector2d p = _tfinv*(config-_center);
    validity_t result = INVALID;
    for(size_t i=0; i<2; ++i)
    {
        double v = fabs(p[i])-fabs(scales[i]);
        if( v >= buffer )
        {
            return VALID;
        }
        else if( v >= 0 )
        {
            result = AT_RISK;
        }
    }
    
    return result;
}

size_t BlockConstraint::constraintDimension() const
{
    return 1;
}

double BlockConstraint::_basicCostGrad(Eigen::Vector2d& grad_c, const Vector2d &config)
{
    Eigen::Vector2d p = _tfinv*(config-_center);
//    std::cout << p.transpose() << "\t|\t" << scales[0] << ", " << scales[1] << std::endl;
    
    assert( buffer > 0 && "We only support positive-valued buffer sizes!");
    
    double c = INFINITY;
    for(size_t i=0; i<2; ++i)
    {
        if(fabs(p[i]) >= fabs(scales[i])+fabs(buffer))
        {
            grad_c.setZero();
            return 0;
        }
        else if(fabs(p[i]) >= fabs(scales[i]))
        {
            double dist = buffer-(fabs(p[i])-fabs(scales[i]));
            double ctemp = dist*dist/(2*buffer);
            if(ctemp < c)
            {
                c = ctemp;
                grad_c.setZero();
                grad_c[i] = -(fabs(p[i])-fabs(scales[i])-buffer)/buffer*sign(p[i]);
            }
        }
        else
        {
            double ctemp = scales[i]-p[i] + buffer/2;
            if(ctemp < c)
            {
                c = ctemp;
                grad_c.setZero();
                grad_c[i] = -sign(p[i]);
            }
        }
    }
    
    return c;
}
