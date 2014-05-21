#ifndef CONSTRAINT_H
#define CONSTRAINT_H

#include "../ccrrt/Trajectory.h"

namespace ccrrt {

class Constraint
{
public:
    
    typedef enum {
        
        INVALID = 0,
        AT_RISK,
        VALID
        
    } validity_t;
    
    virtual bool fillJacobian(Eigen::MatrixXd& H, const ccrrt::Trajectory& traj) = 0;
    
    virtual validity_t getCost(Eigen::VectorXd& cost, const ccrrt::Trajectory& traj) = 0;

    virtual validity_t getValidity(const ccrrt::Trajectory& traj) = 0;
    
    virtual size_t constraintDimension() const = 0;

    // TODO: Consider making this a function of Trajectory instead
    // Counter-argument: Maybe someone will want their own version of speed calculation for a
    //                   certain class of constraints?
    virtual double getSpeed(const ccrrt::Trajectory& traj, size_t waypoint);
    virtual void getVelocity(Eigen::VectorXd& vel,
                                    const ccrrt::Trajectory& traj,
                                    size_t waypoint);
    
};

} // namespace ccrrt

#endif // CONSTRAINT_H
