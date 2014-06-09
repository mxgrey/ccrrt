
#include "../ccrrt/CBiRRT.h"

using namespace ccrrt;
using namespace Eigen;

CBiRRT::CBiRRT(int maxTreeSize, double maxStepSize, double collisionCheckStepSize) :
    RRTManager(maxTreeSize, maxStepSize, collisionCheckStepSize)
{
    _projection_constraints = NULL;
    _rejection_constraints = NULL;
    max_projection_attempts = 100;
    gamma = 1;
    stuck_distance = maxStepSize/10;
}

void CBiRRT::setRejectionConstraints(Constraint *constraints)
{
    _rejection_constraints = constraints;
}

void CBiRRT::setProjectionConstraints(Constraint *constraints)
{
    _projection_constraints = constraints;
}

bool CBiRRT::collisionChecker(const JointConfig &config, const JointConfig &parentConfig)
{
    if(_rejection_constraints==NULL)
        return true;
    
    size_t c = ceil((parentConfig-config).norm()/collisionCheckStepSize_);
    if( c == 0 )
        return (_rejection_constraints->getValidity(config) != Constraint::INVALID)
            && (_projection_constraints->getValidity(config) != Constraint::INVALID);
    
    for(size_t i=0; i<c; ++i)
    {
        if(_rejection_constraints->getValidity(config + 
                    (parentConfig-config).normalized()*i/c) == Constraint::INVALID)
            return false;
        
        if(_projection_constraints->getValidity(config +
                    (parentConfig-config).normalized()*i/c) == Constraint::INVALID)
            return false;
    }
    
    return true;
}

bool CBiRRT::constraintProjector(JointConfig &config, const JointConfig &parentConfig)
{
    if(_projection_constraints==NULL)
        return true;
    
    JointConfig target = config;
    config = parentConfig;
    stepConfigTowards(config, target);
    
    size_t count=0;
    VectorXd gradient(_domainSize);
    Constraint::validity_t result = _projection_constraints->getCostGradient(gradient, parentConfig,
                                                                             config, target);

    while(result == Constraint::INVALID)
    {
        if(!checkIfInRange(config, parentConfig))
            return false;

        if(count > max_projection_attempts)
            return false;
        
        config = config - gamma*gradient;
        for(int i=0; i<_domainSize; ++i)
        {
            if( config[i] < minConfig[i] || maxConfig[i] < config[i] )
                return false;
        }
        
        result = _projection_constraints->getCostGradient(gradient, parentConfig, config, target);
        
        if( (config-parentConfig).norm() < stuck_distance )
            return false;

        ++count;
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
