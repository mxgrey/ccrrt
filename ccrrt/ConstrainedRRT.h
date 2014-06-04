#ifndef CONSTRAINEDRRT_H
#define CONSTRAINEDRRT_H

#include "../ccrrt/RRTManager.h"
#include "../ccrrt/Constraint.h"

namespace ccrrt {

class ConstrainedRRT : public RRTManager
{
public:
    
    ConstrainedRRT(int maxTreeSize=10000, 
                   double maxStepSize=0.1,
                   double collisionCheckStepSize=0.1);
    
    bool collisionChecker(JointConfig& config, const JointConfig& parentConfig);
    
    void setConstraints(Constraint* constraints);
    
protected:
    
    Trajectory _col;
    Constraint* _constraints;
    Eigen::VectorXd _cost;
    
};

} // namespace ccrrt

#endif // CONSTRAINEDRRT_H
