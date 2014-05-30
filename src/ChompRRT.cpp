
#include "../ccrrt/ChompRRT.h"

using namespace ccrrt;
using namespace Eigen;

ChompRRT::ChompRRT(int maxTreeSize, double maxStepSize, double collisionCheckStepSize) :
    ConstrainedRRT(maxTreeSize, maxStepSize, collisionCheckStepSize)
{
    _firstAttempt = true;
}

RRT_Result_t ChompRRT::growTrees(Trajectory& referenceTraj)
{
    assert( trees.size() == 2
            && "ChompRRT is only designed to handle 1 start tree and 1 goal tree!");

    RRT_Result_t check = checkStatus();
    if(check != RRT_NOT_FINISHED)
        return check;

    JointConfig refConfig(_domainSize);

    if(_firstAttempt)
    {
        refConfig = (trees[0]->getConfig()+trees[1]->getConfig())/2;
        _firstAttempt = false;
    }
    else
    {
        randomizeConfig(refConfig);
    }

    _con.state_space = _domainSize;
    _con.waypoints = 1;

    RRTNode* startNode;
    RRTNode* endNode;
    trees[0]->getClosestNode(startNode, refConfig);
    trees[1]->getClosestNode(endNode, refConfig);

    _con.start = startNode->getConfig();
    _con.end = endNode->getConfig();
    _con.xi = refConfig;
//    std::cout << "xi: " << _con.xi.transpose() << std::endl;

    referenceTraj = _con;

    bool success = multichomp.run(_con, _constraint, minConfig, maxConfig, _maxStepSize);
    _con = multichomp.getTrajectory();
//    if(success)
//        std::cout << "Connect Succeeded" << std::endl;
//    else
//        std::cout << "Connect Failed" << std::endl;
//    std::cout << "T: " << _con.xi.transpose() << std::endl;

    if(success)
    {
        RRTNode* node = startNode;
        for(size_t i=0; i<_con.waypoints; ++i)
        {
            node = node->attemptAddChild(_con.xi.block(i*_domainSize,0,_domainSize,1));
            assert(node != NULL && "Uh oh... big bug in the CHOMP connection attempt!");
        }
        constructSolution(node, endNode);

        return RRT_SOLVED;
    }
    else
    {
        RRTNode* node = startNode;
        JointConfig config = startNode->getConfig();
        size_t counter=0;
        while(counter < _con.waypoints)
        {
            const JointConfig& target = _con.xi.block(counter*_domainSize,0,_domainSize,1);
            stepConfigTowards(config, target);
            if( config == target )
            {
                ++counter;
            }

            if(!collisionChecker(config, node->getConfig()))
                break;

            node = node->attemptAddChild(config);

            assert(node != NULL && "Wtf... big bug in the CHOMP connector");
        }

        counter=_con.waypoints-1;
        node = endNode;
        config = endNode->getConfig();
        while(counter > 0)
        {
            const JointConfig& target = _con.xi.block(counter*_domainSize,0,_domainSize,1);
            stepConfigTowards(config, target);
            if( config == target )
            {
                --counter;
            }

            if(!collisionChecker(config, node->getConfig()))
                break;

            node = node->attemptAddChild(config);

            assert(node != NULL && "Wtf... big bug in the CHOMP connector");
        }
    }

    return RRT_NOT_FINISHED;
}
