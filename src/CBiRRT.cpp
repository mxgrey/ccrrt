
#include "../ccrrt/CBiRRT.h"

using namespace ccrrt;
using namespace Eigen;

CBiRRT::CBiRRT(int maxTreeSize, double maxStepSize, double collisionCheckStepSize) :
    ConstrainedRRT(maxTreeSize, maxStepSize, collisionCheckStepSize)
{
    _projected_constraints = NULL;
    max_projection_attempts = 10;
    gamma = 0.05;
    stuck_distance = maxStepSize/5;
}

void CBiRRT::setProjectedConstraints(Constraint *constraints)
{
    _projected_constraints = constraints;
}

bool CBiRRT::constraintProjector(JointConfig &config, const JointConfig &parentConfig)
{
    if(_projected_constraints==NULL)
        return true;
    
    size_t count=0;
    VectorXd gradient(_domainSize);
    Constraint::validity_t result = _projected_constraints->getCostGradient(gradient, config);
    while(result == Constraint::INVALID)
    {
        if(!checkIfInRange(config, parentConfig))
            return false;
        
        ++count;
        if(count > max_projection_attempts)
            return false;
        
        config = config - gamma*gradient;
        result = _projected_constraints->getCostGradient(gradient, config);
        
        if( (config-parentConfig).norm() < stuck_distance )
            return false;
    }
    
    return true;
}

RRT_Result_t CBiRRT::growTrees()
{
    RRT_Result_t check = checkStatus();
    if(check != RRT_NOT_FINISHED)
        return check;
    
    JointConfig refConfig(_domainSize);
    
    for(size_t i=0; i<trees.size(); ++i)
    {
        if(checkIfMaxed(i))
            continue;
        
        randomizeConfig(refConfig);
        
        RRTNode* node;
        if( trees[i]->getClosestNode(node, refConfig) < _numPrecThresh )
            continue;
        
        attemptConnect(node, refConfig, i);
        if(node == NULL)
            continue;
        
        if(treeTypeTracker[i] == RRT_START_TREE)
        {
            RRTNode* connection = lookForTreeConnection(node, RRT_GOAL_TREE);
            if(connection != NULL)
            {
                constructSolution(node, connection);
                return RRT_SOLVED;
            }
        }
        else if(treeTypeTracker[i] == RRT_GOAL_TREE)
        {
            RRTNode* connection = lookForTreeConnection(node, RRT_START_TREE);
            if(connection != NULL)
            {
                constructSolution(connection, node);
                return RRT_SOLVED;
            }
        }
    }
    
    ++_iterations;
    return RRT_NOT_FINISHED;
}
