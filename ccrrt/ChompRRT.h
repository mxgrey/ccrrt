#ifndef CHOMPRRT_H
#define CHOMPRRT_H

#include "../ccrrt/ConstrainedRRT.h"
#include "../ccrrt/MultiChomper.h"

namespace ccrrt {

class ChompRRT : public ConstrainedRRT
{
public:

    ChompRRT(int maxTreeSize=10000,
             double maxStepSize=0.1,
             double collisionCheckStepSize=0.1);

    RRT_Result_t growTrees();
    RRTNode* attemptConnect(RRTNode *&node, const JointConfig &target, size_t treeID);

    MultiChomper multichomp;


protected:

    bool _firstAttempt;

    Trajectory _con;


};

} // namespace ccrrt

#endif // CHOMPRRT_H
