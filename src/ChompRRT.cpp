
#include "../ccrrt/ChompRRT.h"

using namespace ccrrt;
using namespace Eigen;

ChompRRT::ChompRRT(int maxTreeSize, double maxStepSize, double collisionCheckStepSize) :
    ConstrainedRRT(maxTreeSize, maxStepSize, collisionCheckStepSize)
{
    _firstAttempt = true;
}

RRT_Result_t ChompRRT::growTrees()
{
    assert( trees.size() == 2
            && "ChompRRT is only designed to handle 1 start tree and 1 goal tree!");
    
    _con.state_space = _domainSize;
    _con.waypoints = 1;

//    std::cout << " ---- grow tree ---- " << std::endl;

    RRT_Result_t check = checkStatus();
    if(check != RRT_NOT_FINISHED)
        return check;

    JointConfig refConfig(_domainSize);

    if(_firstAttempt)
    {
        for(size_t i=0; i<trees.size(); ++i)
        {
            if(treeTypeTracker[i] == RRT_START_TREE)
            {
                RRTNode* connection = lookForTreeConnection(trees[i], RRT_GOAL_TREE);
                if(connection != NULL)
                {
                    constructSolution(trees[0], connection);
                    return RRT_SOLVED;
                }
            }
        }
        _firstAttempt = false;
    }
//    else
//    {
//    }


//    RRTNode* startNode;
//    RRTNode* endNode;
//    trees[0]->getClosestNode(startNode, refConfig);
//    trees[1]->getClosestNode(endNode, refConfig);

//    _con.start = startNode->getConfig();
//    _con.end = endNode->getConfig();
//    _con.xi = refConfig;
    
//    multichomp.initialize(_con, _constraints, minConfig, maxConfig);
//    size_t quit = 0;
//    while(multichomp.iterate(true) == Constraint::INVALID)
//    {
//        ++quit;
//        if(quit > multichomp.max_attempts)
//        {
//            break;
//        }
//    }
    
//    refConfig = multichomp.getTrajectory().xi;
    
    
//    bool success = true;
//    for(int i=trees.size()-1; i>=0; --i)
    for(size_t i=0; i<trees.size(); ++i)
    {
        if(checkIfMaxed(i))
            continue;
        
        randomizeConfig(refConfig);
        
        RRTNode* node;
        if(trees[i]->getClosestNode(node, refConfig) < _numPrecThresh)
            continue;
        
        attemptConnect(node, refConfig, i);
        
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
                constructSolution(node, connection);
                return RRT_SOLVED;
            }
        }
        
//        trees[i]->getClosestNode(startNode, refConfig);
//        _con.start = startNode->getConfig();
//        _con.end = refConfig;
//        _con.xi = (_con.start+_con.end)/2.0;
        
//        success = multichomp.run(_con, _constraints, minConfig, maxConfig, _maxStepSize);
//        const Trajectory& path = multichomp.getTrajectory();
        
//        RRTNode* node = startNode;
//        JointConfig config = node->getConfig();
//        size_t counter=0;
//        while( counter < path.waypoints+1 )
//        {
////            std::cout << "Counter " << counter << std::endl;
//            const JointConfig& target = counter < path.waypoints?
//                                            path.xi.block(counter*_domainSize,0,_domainSize,1) :
//                                            path.end;
            
//            stepConfigTowards(config, target);
//            if(config == target)
//            {
//                ++counter;
//            }
            
//            if(!collisionChecker(config, node->getConfig()))
//            {
//                if(success)
//                    std::cout << "Collision checking failure!" << std::endl;
//                break;
//            }
            
//            node = node->attemptAddChild(config);
//            ++treeSizeCounter[i];
            
//            assert(node != NULL && "Wtf... big bug in the CHOMP connector");
//        }
        
//        refConfig = node->getConfig();
        
//        lastNode[i] = node;
//        if(success)
//            std::cout << " | success | ";
//        else
//            std::cout << " | fail | ";
    }

//    std::cout << std::endl;
    
//    if(success)
//    {
//        std::cout << "Constructing" << std::endl;
//        constructSolution(lastNode[0], lastNode[1]);
//        return RRT_SOLVED;
//    }
    
    
//    trees[0]->getClosestNode(startNode, _con.xi);
//    trees[1]->getClosestNode(endNode, _con.xi);
//    _con.start = startNode->getConfig();
//    _con.end = endNode->getConfig();
    
//    bool success = false;
    
//    if(quit <= multichomp.max_attempts)
//    {
//        success = multichomp.run(_con, _constraints, minConfig, maxConfig, _maxStepSize);
//        _con = multichomp.getTrajectory();
//    }

//    if(success)
//    {
        
//        RRTNode* node = startNode;
        
//        JointConfig config = node->getConfig();
//        size_t counter=0;
//        while(counter < _con.waypoints)
//        {
//            const JointConfig& target = _con.xi.block(counter*_domainSize,0,_domainSize,1);
//            stepConfigTowards(config, target);
//            if( config == target )
//            {
//                ++counter;
//            }
            
//            if(!collisionChecker(config, node->getConfig()))
//            {
//                std::cout << "STEP SIZE ASSUMPTION FAILURE" << std::endl;
//                return RRT_NOT_FINISHED;
//            }
            
//            node = node->attemptAddChild(config);
//            assert(node != NULL && "Uh oh... big bug in the CHOMP connection attempt!");
//        }
        
//        if(success)
//        {
//            constructSolution(node, endNode);
//            return RRT_SOLVED;
//        }
//    }
//    else
//    {
//        RRTNode* node = startNode;
//        JointConfig config = startNode->getConfig();
//        size_t counter=0;
//        while( (refConfig-config).norm() > fabs(_maxStepSize)/2 )
//        {
//            const JointConfig& target = _con.xi.block(counter*_domainSize,0,_domainSize,1);
//            stepConfigTowards(config, target);
//            if( config == target )
//            {
//                ++counter;
//            }

//            if(!collisionChecker(config, node->getConfig()))
//                break;

//            node = node->attemptAddChild(config);

//            assert(node != NULL && "Wtf... big bug in the CHOMP connector");
//        }

//        counter=_con.waypoints-1;
//        node = endNode;
//        config = endNode->getConfig();
//        while( (refConfig-config).norm() > fabs(_maxStepSize)/2 )
//        {
//            const JointConfig& target = _con.xi.block(counter*_domainSize,0,_domainSize,1);
//            stepConfigTowards(config, target);
//            if( config == target )
//            {
//                --counter;
//            }

//            if(!collisionChecker(config, node->getConfig()))
//                break;

//            node = node->attemptAddChild(config);

//            assert(node != NULL && "Wtf... big bug in the CHOMP connector");
//        }
//    }
    
    ++_iterations;
    return RRT_NOT_FINISHED;
}

RRTNode* ChompRRT::attemptConnect(RRTNode *&node, const JointConfig &target, size_t treeID)
{
    _con.start = node->getConfig();
    _con.end = target;
    _con.xi = (_con.start+_con.end)/2.0;
    
    multichomp.run(_con, _constraints, minConfig, maxConfig, _maxStepSize);
    const Trajectory& path = multichomp.getTrajectory();
    
    JointConfig config = node->getConfig();
    size_t counter = 0;
    while( counter < path.waypoints+1 )
    {
        const JointConfig& target = counter < path.waypoints?
                                        path.xi.block(counter*_domainSize,0,_domainSize,1) :
                                        path.end;
        
        stepConfigTowards(config, target);
        if(config == target)
            ++counter;
        
        if(!collisionChecker(config, node->getConfig()))
        {
            node->type = RRTNode::KEY;
            return NULL;
        }
        
        node = node->attemptAddChild(config);
        ++treeSizeCounter[treeID];
        
        assert( node != NULL && "Wtf... big bug in the CHOMP connector");
    }
    
    node->type = RRTNode::KEY;
    return node;
}



