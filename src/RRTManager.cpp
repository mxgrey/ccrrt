
/*
 * Copyright (c) 2013, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author: Michael X. Grey <mxgrey@gatech.edu>
 * Date: Nov 13, 2013
 *
 * Humanoid Robotics Lab      Georgia Institute of Technology
 * Director: Mike Stilman     http://www.golems.org
 *
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */


#include "../ccrrt/RRTManager.h"
#include <ctime>

bool RRTManager::collisionChecker(JointConfig &config, const JointConfig &parentConfig)
{
    // THIS IS WHERE YOU SHOULD APPLY COLLISION CHECKING. IF THE CONFIGURATION
    // IS ACCEPTABLE, YOU SHOULD RETURN true. IF THE CONFIGURATION IS IN A
    // COLLISION, YOU SHOULD RETURN FALSE.

    // THIS FUNCTION GRANTS YOU THE RIGHT TO MODIFY CONFIG IN CASE YOU WANT TO
    // NUDGE IT AROUND TO AVOID A COLLISION, OR MAYBE MOVE AWAY FROM A NEAR
    // COLLISION. HOWEVER, THE PLANNER WILL YELL AT YOU IF YOUR MODIFIED
    // CONFIGURATION VIOLATES THE STEP SIZE LIMIT, SO BE SURE NOT TO DO THAT
    // BY USING checkIfInRange(config, parentConfig) OR ALSO scaleConfig(config, parentConfig)
    return true;
}


bool RRTManager::constraintProjector(JointConfig &config, const JointConfig &parentConfig)
{
    // THIS IS WHERE YOU SHOULD PROJECT ARBITRARY JOINT CONFIGURATIONS ONTO
    // YOUR CONSTRAINT MANIFOLD (i.e. move the configuration so that it
    // satisfies your constraints). THIS FUNCTION SHOULD RETURN true IF YOU
    // WANT THE RRT PLANNER TO CONTINUE USING THIS CONFIGURATION OR IT SHOULD
    // RETURN FALSE IF YOU WANT THE PLANNER TO ABANDON THIS CONFIGURATION.
    return true;
}

RRT_Result_t RRTManager::growTrees()
{
    // THIS IS THE DEFUALT VERSION OF growTrees(). YOU SHOULD INHERIT RRTManager
    // AND EITHER WRITE OVER THIS VIRTUAL FUNCTION WITH YOUR OWN OR SIMPLY
    // OVERWRITE THE VIRTUAL FUNCTIONS collisionChecker(~) AND constraintProjector(~)
    // (instructions will be in capital letters, explanations are in lowercase)
    
    
    if(!_hasDomain) // Make sure we have a domain
        return RRT_NO_DOMAIN;
    
    if(_hasSolution) // (Optional)
        return RRT_SOLVED; // Don't waste our time!
    
    if(maxIterations_ >= 0 && _iterations >= maxIterations_)
        return RRT_REACHED_ITER_LIMIT;
    
    if(maxTotalNodes_ >= 0 && getTotalNodes() >= maxTotalNodes_)
        return RRT_REACHED_MAX_NODES;
    
    if(checkIfAllMaxed())
        return RRT_TREES_MAXED;
    
    // Grab a random reference configuration for the trees to move towards
    JointConfig refConfig(_domainSize);
    randomizeConfig(refConfig);
    
    // This variable will be altered by each tree and reset to refConfig
    // with each iteration through the trees
    JointConfig config;
    
    for(size_t i=0; i<trees.size(); i++)
    {
        if(checkIfMaxed(i)) // Don't waste our time on a full tree
            continue;
        
        config = refConfig; // reset our config to the refConfig
        
        // Using the reference (&) is VERY important here
        RRTNode& newParent = trees[i]->getClosestNodeAndScaleConfig(config);
        
        //{
        //   THIS IS WHERE YOU SHOULD PROJECT TO THE CONSTRAINT MANIFOLD
        //   (IF YOU ARE DOING CONSTRAINED PLANNING)
        //}
        if(!constraintProjector(config, newParent.getConfig()))
            continue;
        
        // Note: the following function does not actually accomplish anything
        // in this particular code example, but it would be an important step
        // if you followed the above instruction and projected the config onto
        // the constraint manifold
        newParent.scaleJointConfig(config);
        //{
        //   NOW YOU SHOULD CHECK IF THE CONFIG IS STILL INSIDE YOUR CONSTRAINT
        //   MANIFOLD. IF NOT, YOU SHOULD PROBABLY >> continue; BUT THIS IS UP
        //   TO YOUR DISCRETION
        //}
        
        //{
        //   AND THEN THIS IS WHERE YOU SHOULD PERFORM COLLISION CHECKING.
        //   IF COLLISION CHECKING FAILS, YOU SHOULD >> continue;
        //}
        if(!collisionChecker(config, newParent.getConfig()))
            continue;

        
        // Assuming config has satisfied everything, we should add it to the tree
        RRTNode* newChild = newParent.attemptAddChild(config);
        if(newChild==NULL)
        {
            // Since the last thing we did to config is make it a suitable child
            // by scaling it << newParent.scaleJointConfig(config) >>, newChild
            // should not be null, and something is very wrong in the code somewhere.
            // Most likely your collision checker modified the config in an
            // unacceptable way.
            std::cerr << "Your code is broken!" << std::endl;
            continue;
        }
        treeSizeCounter[i]++;
        
        // Now we attempt to connect the new child to one of the other trees
        if(treeTypeTracker[i] == RRT_START_TREE)
        {
            // If our tree is a start tree, we should try to connect it to a goal tree
            RRTNode* connection = lookForTreeConnection(newChild, RRT_GOAL_TREE);
            if(connection != NULL)
            {
                constructSolution(newChild, connection);
                return RRT_SOLVED;
            }
        }
        else if(treeTypeTracker[i] == RRT_GOAL_TREE)
        {
            // If our tree is a goal tree, we should try to connect it to a start tree
            RRTNode* connection = lookForTreeConnection(newChild, RRT_START_TREE);
            if(connection != NULL)
            {
                constructSolution(connection, newChild);
                return RRT_SOLVED;
            }
        }
        // You could also handle other tree types here like RRT_AMBI_TREE, but it's
        // probably a silly idea. It might be best to only handle START and GOAL trees.
    }
    
    _iterations++;
    return RRT_NOT_FINISHED;
}

void RRTManager::shortenSolution()
{
    // JUST LIKE growTree() YOU WILL NEED TO OVERWRITE THIS FUNCTION AFTER YOU HAVE
    // INHERITED THE CLASS. SIMPLY PLUG IN YOUR CONSTRAINER AND COLLISION CHECKER
    // BELOW WHERE INSTRUCTED.
    
    if(!_hasSolution)
    {
        std::cerr << "I will not shorten a solution that doesn't exist!" << std::endl;
        return;
    }
    
    ConfigPath testPlan;
    ConfigPath bufferPlan; bufferPlan.resize(0);
    bufferPlan.push_back(solvedPlan[0]);
    
    JointConfig startConfig = solvedPlan[0];
    JointConfig finalConfig = solvedPlan[solvedPlan.size()-1];
    JointConfig testConfig, lastTestConfig;
    int startIndex = 0;
    while((startConfig-finalConfig).norm() > _numPrecThresh
    && (bufferPlan[bufferPlan.size()-1]-finalConfig).norm() > _numPrecThresh)
    {
        bool valid;
        int targetIndex = solvedPlan.size()-1;
        JointConfig targetConfig = finalConfig;
        double remainingNorm = (startConfig-targetConfig).norm();
        while(remainingNorm > _numPrecThresh)
        {
            testPlan.resize(0);
            
            valid = true;
            
            testConfig = startConfig;
            lastTestConfig = startConfig;
            while((testConfig-targetConfig).norm() > _numPrecThresh)
            {
                stepConfigTowards(testConfig, targetConfig);
                
                //{
                //   THIS IS WHERE YOU SHOULD PROJECT testConfig TO THE CONSTRAINT
                //   MANIFOLD (IF YOU ARE DOING CONSTRAINED PLANNING).
                //   YOU SHOULD SET valid TO FALSE IF YOUR CONSTRAINTS CANNOT
                //   BE SATISFIED
                //}
                valid = valid && constraintProjector(testConfig, lastTestConfig);

                if(!valid)
                    break;
                
                valid = valid && checkIfInRange(testConfig, lastTestConfig);
                
                if(!valid)
                    break;
                
                //{
                //   THIS IS WHERE YOU SHOULD COLLISION CHECK testConfig
                //
                //
                //}
                valid = valid && collisionChecker(testConfig, lastTestConfig);
                
                if(!valid)
                    break;
                
                testPlan.push_back(testConfig);
                lastTestConfig = testConfig;
            }
            
            if(valid)
            {
                bufferPlan.resize(startIndex+1);
                for(size_t i=0; i<testPlan.size(); i++)
                    bufferPlan.push_back(testPlan[i]);
                
                break;
            }
            
            targetIndex--;
            targetConfig = solvedPlan[targetIndex];
            remainingNorm = (startConfig-targetConfig).norm();
        }
        
        if(!valid)
        {
            bufferPlan.push_back(solvedPlan[targetIndex+1]);
        }
        
        startIndex++;
        startConfig = bufferPlan[startIndex];
    }
    
    solvedPlan = bufferPlan;
}



RRTNode::RRTNode(JointConfig mConfig, double maxStepSize, RRTNode *mParent)
    : config(mConfig)
{
    giveParent(mParent);
    maxStepSize_ = maxStepSize;
    hasChildren_ = false;
    numPrecThresh = 1e-10;
}

const JointConfig& RRTNode::getConfig() const
{
    return config;
}


void RRTNode::giveParent(RRTNode *newParent)
{
    if(newParent==NULL)
        hasParent_ = false;
    else
        hasParent_ = true;

    parent = newParent;
}

const RRTNode *RRTNode::getParent() const
{
    if(hasParent_)
        return parent;
    else
        return this;
}
bool RRTNode::hasParent() const { return hasParent_; }


const RRTNode *RRTNode::getChild(size_t child) const
{
    if(child < children.size())
        return children[child];
    else
        return this;
}
bool RRTNode::hasChild() const { return hasChildren_; }
size_t RRTNode::numChildren() const { return children.size(); }


RRTNode& RRTNode::getClosestNodeAndScaleConfig(JointConfig &newConfig)
{
    RRTNode* closest = NULL;
    double scale = getClosestNode(closest, newConfig);
    
    scale = fabs(scale) < fabs(maxStepSize_) ? fabs(scale) : fabs(maxStepSize_);
    newConfig = closest->config + scale*(newConfig-closest->config).normalized();

    return *closest;
}


double RRTNode::getClosestNode(RRTNode* &closest, const JointConfig &newConfig)
{
    double eval = (newConfig-config).norm();
    closest = this;

    for(size_t i=0; i<children.size(); i++)
    {
        RRTNode* closerCheck;
        double evalCheck = children[i]->getClosestNode(closerCheck, newConfig);

        if(evalCheck < eval)
        {
            closest = closerCheck;
            eval = evalCheck;
        }
    }

    return eval;
}


RRTNode *RRTNode::attemptAddChild(const JointConfig &childConfig)
{
    if((childConfig-config).norm() > maxStepSize_+numPrecThresh)
    {
//        childConfig = config + maxStepSize_*(childConfig-config).normalized();
        return NULL;
    }
    else
    {
        return addChild(childConfig);
    }
}

bool RRTNode::scaleJointConfig(JointConfig &childConfig)
{
    if((childConfig-config).norm() > maxStepSize_+numPrecThresh)
    {
        childConfig = config + maxStepSize_*(childConfig-config).normalized();
        return false;
    }
    else
    {
        return true;
    }
}


void RRTNode::setMaxStepSize(double newMaxStepSize)
{
    maxStepSize_ = newMaxStepSize;
    for(size_t i=0; i<children.size(); i++)
        children[i]->setMaxStepSize(newMaxStepSize);
}


RRTNode *RRTNode::addChild(const JointConfig &childConfig)
{
    hasChildren_ = true;
    RRTNode* newChild = new RRTNode(childConfig, maxStepSize_, this);
    children.push_back(newChild);
    return newChild;
}













RRTManager::RRTManager(int maxTreeSize, double maxStepSize, double collisionCheckStepSize)
    : maxTreeSize_(maxTreeSize),
      collisionCheckStepSize_(collisionCheckStepSize),
      _maxStepSize(maxStepSize)
{
    trees.resize(0);
    currentTreeSize.resize(0);
    _hasSolution = false;
    _hasDomain = false;
    srand(time(NULL));
    maxTotalNodes_ = -1;
    maxIterations_ = -1;
    _iterations = 0;
    _numPrecThresh = 1e-10;
}

void RRTManager::setMaxStepSize(double newMaxStepSize)
{
    _maxStepSize = newMaxStepSize;
    for(size_t i=0; i<trees.size(); i++)
        trees[i]->setMaxStepSize(newMaxStepSize);
}


int RRTManager::addStartTree(JointConfig startConfiguration)
{
    return addTree(startConfiguration, RRT_START_TREE);
}

int RRTManager::addGoalTree(JointConfig goalConfiguration)
{
    return addTree(goalConfiguration, RRT_GOAL_TREE);
}

int RRTManager::addTree(JointConfig rootNodeConfiguration, RRT_Tree_t treeType)
{
    if(!checkIfInDomain(rootNodeConfiguration))
        return -1;

    if(!collisionChecker(rootNodeConfiguration, rootNodeConfiguration))
        return -2;

    if(!constraintProjector(rootNodeConfiguration, rootNodeConfiguration))
        return -3;

    RRTNode* rootNode = new RRTNode(rootNodeConfiguration, _maxStepSize);
    trees.push_back(rootNode);
    treeSizeCounter.push_back(1);
    treeTypeTracker.push_back(treeType);
    
    return trees.size()-1;
}

int RRTManager::getTreeSize(size_t tree) const
{
    if(tree < treeSizeCounter.size())
        return treeSizeCounter[tree];
    else
        return 0;
}

int RRTManager::getTotalNodes() const
{
    int result = 0;
    for(size_t i=0; i<treeSizeCounter.size(); i++)
        result += getTreeSize(i);
    return result;
}

int RRTManager::getNumIterations() const { return _iterations; }

RRT_Tree_t RRTManager::getTreeType(size_t tree) const
{
    if(tree < treeTypeTracker.size())
        return treeTypeTracker[tree];
    else
        return RRT_INVALID_TREE;
}

size_t RRTManager::getNumTrees() const
{
    return trees.size();
}

const RRTNode* RRTManager::getTree(size_t tree) const
{
    if(tree < trees.size())
        return trees[tree];
    else
        return NULL;
}

bool RRTManager::checkIfMaxed(int tree) const
{
    return maxTreeSize_ <= getTreeSize(tree);
}

bool RRTManager::checkIfAllMaxed() const
{
    for(size_t i=0; i<treeSizeCounter.size(); i++)
        if(!checkIfMaxed(i))
            return false;
    
    return true;
}

bool RRTManager::checkIfAnyMaxed() const
{
    for(size_t i=0; i<treeSizeCounter.size(); i++)
        if(checkIfMaxed(i))
            return true;
    
    return false;
}

bool RRTManager::checkIfSolved() const { return _hasSolution; }

RRTNode* RRTManager::lookForTreeConnection(const RRTNode* targetNode, RRT_Tree_t connectTargetType)
{
    RRTNode* goalCandidate = NULL;
    double eval = INFINITY;
    for(size_t i=0; i<trees.size(); i++)
    {
        if(treeTypeTracker[i] == connectTargetType)
        {
            RRTNode* goalCheck;
            double evalCheck = trees[i]->getClosestNode(goalCheck, targetNode->getConfig());
            
            if(evalCheck < eval)
            {
                eval = evalCheck;
                goalCandidate = goalCheck;
            }
        }
    }
    
    return attemptConnect(goalCandidate, targetNode);
}

RRTNode* RRTManager::attemptConnect(RRTNode *begin, const RRTNode *target)
{
    RRTNode* currentNode = begin;
    JointConfig currentConfig = begin->getConfig();

    while( (currentNode->getConfig()-target->getConfig()).norm() > _maxStepSize )
    {
        stepConfigTowards(currentConfig, target->getConfig());

        if(!constraintProjector(currentConfig, currentNode->getConfig()))
            return NULL;

        currentNode->scaleJointConfig(currentConfig);

        if(!collisionChecker(currentConfig, currentNode->getConfig()))
            return NULL;

        currentNode = currentNode->attemptAddChild(currentConfig);
        if(currentNode == NULL)
        {
            std::cerr << "Attempt connect code is broken!" << std::endl;
            return NULL;
        }
    }

    return currentNode;
}

void RRTManager::constructSolution(const RRTNode *beginTree, const RRTNode *endTree)
{
    _hasSolution = true;
    solvedPlan.resize(0);
    
    ConfigPath beginPortion; beginPortion.resize(0);
    ConfigPath endPortion; endPortion.resize(0);
    
    const RRTNode* buildingNode = beginTree;
    beginPortion.push_back(buildingNode->getConfig());
    while(buildingNode->hasParent())
    {
        buildingNode = buildingNode->getParent();
        beginPortion.push_back(buildingNode->getConfig());
    }
    
    for(size_t i=0; i<beginPortion.size(); i++)
        solvedPlan.push_back(beginPortion[beginPortion.size()-1-i]);
    
    buildingNode = endTree;
    solvedPlan.push_back(buildingNode->config);
    while(buildingNode->hasParent())
    {
        buildingNode = buildingNode->getParent();
        solvedPlan.push_back(buildingNode->config);
    }
}

void RRTManager::setDomain(const JointConfig &minJointConfig, const JointConfig &maxJointConfig, int resolution)
{
    if(minJointConfig.size() != maxJointConfig.size())
    {
        std::cerr << std::endl << "Your domain is not consistent!" << std::endl
                  << " -- min dimensionality: " << minJointConfig.size() << std::endl
                  << " -- max dimensionality: " << maxJointConfig.size() << std::endl
                     << std::endl;
        
        return;
    }
    
    minConfig = minJointConfig;
    maxConfig = maxJointConfig;
    dResolution = resolution;
    _domainSize = minConfig.size();
    _hasDomain = true;
}

void RRTManager::randomizeConfig(JointConfig &config)
{
    if(!_hasDomain)
    {
        std::cerr << "I will not generate a random configuration until you use setDomain(~)!" << std::endl;
        return;
    }
    
    if(!checkDimensionality(config))
        return;
    
    for(int i=0; i<config.size(); i++)
        config(i) = ((double)(rand()%dResolution))/(double)(dResolution-1)
                * (maxConfig(i) - minConfig(i)) + minConfig(i);
}

void RRTManager::scaleConfig(JointConfig &config, const JointConfig &parentConfig)
{
    if((config-parentConfig).norm() > _maxStepSize+_numPrecThresh)
    {
        config = parentConfig + _maxStepSize*(config-parentConfig).normalized();
    }
}


void RRTManager::stepConfigTowards(JointConfig &config, const JointConfig &targetConfig)
{
    if((targetConfig-config).norm() > _maxStepSize+_numPrecThresh)
    {
        config = config + _maxStepSize*(targetConfig-config).normalized();
    }
    else
    {
        config = targetConfig;
    }
}


bool RRTManager::checkIfInRange(const JointConfig &configA, const JointConfig &configB)
{
    if((configA-configB).norm() > _maxStepSize+_numPrecThresh)
        return false;
    else
        return true;
}

bool RRTManager::checkIfInDomain(const JointConfig &config, bool verbose)
{
    if(!checkDimensionality(config, verbose))
        return false;

    for(int i=0; i<config.size(); i++)
    {
        if(config[i] < minConfig[i] || maxConfig[i] < config[i])
        {
            if(verbose)
            {
                std::cerr << "Outside Limit (" << i << "): " << config[i] << std::endl
                          << "Min: " << minConfig[i] << std::endl
                          << "Max: " << maxConfig[i] << std::endl;
            }
            return false;
        }
    }

    return true;
}

bool RRTManager::checkDimensionality(const JointConfig &config, bool verbose)
{
    if(config.size() != minConfig.size() || config.size() != maxConfig.size())
    {
        if(verbose)
        {
            std::cerr << "Configuration dimensionality does not match the domain!" << std::endl
                      << " -- Config:" << config.size()
                      << ", min:" << minConfig.size()
                      << ", max:" << maxConfig.size()
                      << std::endl;
        }
        return false;
    }
    return true;
}
