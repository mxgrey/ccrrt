#ifndef CBIRRT_H
#define CBIRRT_H

#include "ConstrainedRRT.h"

namespace ccrrt {

class CBiRRT : public ConstrainedRRT
{
public:
    
    CBiRRT(int maxTreeSize=10000,
           double maxStepSize=0.1,
           double collisionCheckStepSize=0.1);
    
    double gamma;
    double stuck_distance;
    size_t max_projection_attempts;
    
    RRT_Result_t growTrees();
    
    bool constraintProjector(JointConfig &config, const JointConfig &parentConfig);
    
    void setProjectedConstraints(Constraint* constraints);
    
protected:
    
    Constraint* _projected_constraints;
    
};

} // namespace ccrrt


#endif // CBIRRT_H
