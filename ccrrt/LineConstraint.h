#ifndef LINECONSTRAINT_H
#define LINECONSTRAINT_H

#include "ConstraintGroup.h"
#include "BlockConstraint.h"

namespace ccrrt {

class LineConstraint : public ConstraintGroup
{
public:

    LineConstraint(const Eigen::Vector2d& mStart, const Eigen::Vector2d& mEnd,
                   double mWidth, double mTolerance = 1e-4);
    
    void setParameters(const Eigen::Vector2d& mStart, const Eigen::Vector2d& mEnd,
                       double mWidth, double mTolerance = 0.005);
    
    inline const Eigen::Vector2d& start() const {return _start;}
    inline const Eigen::Vector2d& end() const {return _end;}
    inline const double& width() const {return _width;}
    inline const double& tolerance() const {return _tol;}

protected:
    
    Eigen::Vector2d _start;
    Eigen::Vector2d _end;
    double _width;
    double _tol;

};

} // namespace ccrrt


#endif // LINECONSTRAINT_H
