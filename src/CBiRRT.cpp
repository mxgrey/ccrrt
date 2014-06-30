
#include "../ccrrt/CBiRRT.h"

using namespace ccrrt;
using namespace Eigen;

CBiRRT::CBiRRT(int maxTreeSize, double maxStepSize, double collisionCheckStepSize) :
    RRTManager(maxTreeSize, maxStepSize, collisionCheckStepSize)
{
    _projection_constraints = NULL;
    _rejection_constraints = NULL;
    max_projection_attempts = 200;
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

bool CBiRRT::rejectionChecker(const JointConfig &config)
{
    if(NULL == _rejection_constraints)
        return true;
    
    return _rejection_constraints->getValidity(config) != Constraint::INVALID;
}

bool CBiRRT::projectionChecker(const JointConfig &config)
{
    if(NULL == _projection_constraints)
        return true;
    
    return _projection_constraints->getValidity(config) != Constraint::INVALID;
}

void dummyfunction(JointConfig col, JointConfig config, JointConfig parentConfig, size_t i, size_t c)
{
    std::cout << "col: " << col.transpose() 
              << "\ncon: " << config.transpose()
              << "\npar: " << parentConfig.transpose() 
              << "\n i,c: " << i << " | " << c << std::endl;
}

bool CBiRRT::collisionChecker(const JointConfig &config, const JointConfig &parentConfig)
{
    if(NULL==_rejection_constraints && NULL==_projection_constraints)
        return true;
    
    size_t c = ceil((parentConfig-config).norm()/collisionCheckStepSize_);
    if( c == 0 )
        return rejectionChecker(config) && projectionChecker(config);
    
    for(size_t i=0; i<c; ++i)
    {
        _col_step = config + (parentConfig-config)*(double)(i)/(double)(c);
        
//        std::cout << " C: " << ceil((parentConfig-config).norm()/collisionCheckStepSize_)
//                     << std::endl;
//        dummyfunction(_col_step, config, parentConfig, i, c);
        
        if(!projectionChecker(_col_step))
            return false;
        
        if(!rejectionChecker(_col_step))
            return false;
    }
    
    return true;
}

bool CBiRRT::constraintProjector(JointConfig &config, const JointConfig &parentConfig)
{
    if(_projection_constraints==NULL)
    {
        JointConfig target = config;
        config = parentConfig;
        stepConfigTowards(config, target);
        return true;
    }
    
    JointConfig target = config;
    config = parentConfig;
    stepConfigTowards(config, target);
    
    size_t count=0;
    VectorXd gradient(_domainSize);
    Constraint::validity_t result = _projection_constraints->getCostGradient(gradient, parentConfig,
                                                                             config, target);
    if(result == Constraint::STUCK)
        return false;

    while(result == Constraint::INVALID)
    {
        if(result == Constraint::STUCK)
        {
            std::cout << "stuck" << std::endl;
            return false;
        }
        
        if(!checkIfInRange(config, parentConfig))
        {
            std::cout << "OUT OF REACH (" << count << ")" << std::endl;
            return false;
        }

        if(count > max_projection_attempts)
        {
//            std::cout << "too many attempts" << std::endl;
            return false;
        }
        
        if(!checkForNan(gradient))
        {
            std::cout << "NANS IN GRADIENT: " << gradient.transpose() << std::endl;
            std::cout << "config: " << config.transpose() << std::endl;
            return false;
        }
        
        config = config - gamma*gradient;
        scaleConfig(config, parentConfig);
        if(!checkForNan(config))
        {
            std::cout << "NANS NANS NANS!!!: " << config.transpose() << std::endl;
            return false;
        }
        for(int i=0; i<_domainSize; ++i)
        {
            if( config[i] < minConfig[i] || maxConfig[i] < config[i] )
            {
                std::cout << "outside joint limits" << std::endl;
                return false;
            }
        }
        
        result = _projection_constraints->getCostGradient(gradient, parentConfig, config, target);
        
        
        if( (config-parentConfig).norm() < stuck_distance )
        {
            std::cout << "within stuck distance" << std::endl;
            return false;
        }

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
