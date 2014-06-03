
#include "../ccrrt/LineConstraint.h"

using namespace ccrrt;
using namespace Eigen;

LineConstraint::LineConstraint(const Vector2d &mStart,
                               const Vector2d &mEnd,
                               double mWidth,
                               double mTolerance)
{
    setParameters(mStart, mEnd, mWidth, mTolerance);
}

void LineConstraint::setParameters(const Vector2d &mStart, 
                                   const Vector2d &mEnd, 
                                   double mWidth, double mTolerance)
{
    
}


