#ifndef CBIRRT_H
#define CBIRRT_H

#include "RRTManager.h"
#include "Constraint.h"

namespace ccrrt {

class CBiRRT : public RRTManager
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
    bool collisionChecker(const JointConfig &config, const JointConfig &parentConfig);
    bool rejectionChecker(const JointConfig& config);
    bool projectionChecker(const JointConfig& config);
    
    void setProjectionConstraints(Constraint* constraints);
    void setRejectionConstraints(Constraint* constraints);
    
protected:
    
    Constraint* _projection_constraints;
    Constraint* _rejection_constraints;
    
    Eigen::VectorXd _col_step;
};

} // namespace ccrrt


#endif // CBIRRT_H
