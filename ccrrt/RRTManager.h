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


#ifndef RRTMANAGER_H
#define RRTMANAGER_H

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <iostream>
#include <vector>

#include <math.h>

typedef Eigen::VectorXd JointConfig;
typedef std::vector<JointConfig, Eigen::aligned_allocator<JointConfig> > ConfigPath;

typedef enum {

    RRT_NOT_FINISHED=0,     ///> A solution has not been found, and the trees have room to grow
    RRT_SOLVED,             ///> A solution has been found and can be accessed in the solvedPlan member
    RRT_TREES_MAXED,        ///> All the trees have reached their limit, and no plan has been found
    RRT_REACHED_MAX_NODES,  ///> The maximum total number of nodes has been reached, and no plan has been found
    RRT_NO_DOMAIN,          ///> The user has neglected to define a domain for the search
    RRT_REACHED_ITER_LIMIT, ///> The growTree() has reached its iteration limit
    RRT_INVALID_ROOT,       ///> The root of one of your trees is in an invalid configuration

    RRT_RESULT_SIZE

} RRT_Result_t;

static const char *RRT_Result_string[RRT_RESULT_SIZE] =
{
    "RRT_NOT_FINISHED",
    "RRT_SOLVED",
    "RRT_TREES_MAXED",
    "RRT_REACHED_MAX_NODES",
    "RRT_NO_DOMAIN",
    "RRT_REACHED_ITER_LIMIT",
    "RRT_INVALID_ROOT"
};

inline std::string RRT_Result_to_string(RRT_Result_t result)
{
    if( 0 <= result && result < RRT_RESULT_SIZE)
        return RRT_Result_string[result];
    else
        return "Unknown Result";
}

inline std::ostream& operator<<(std::ostream& oStrStream, const RRT_Result_t result)
{
    oStrStream << RRT_Result_to_string(result);
    return oStrStream;
}

// TODO: Consider using bit-logic and making these parameters
// instead of a single setting
typedef enum {
    
    RRT_INVALID_TREE=0, ///> Means that this tree was not initialized correctly
    RRT_START_TREE,     ///> Will attempt to connect itself to goal trees
    RRT_GOAL_TREE,      ///> Will attempt to connect itself to start trees
    RRT_AMBI_TREE,      ///> Will attempt to connect itself to any tree type
                        // The last one is probably worthless, but seems like a fun idea
    
    RRT_TREE_TYPE_SIZE
    
} RRT_Tree_t;

static const char *RRT_Tree_type_string[RRT_TREE_TYPE_SIZE] =
{
    "RRT_INVALID_TREE",
    "RRT_START_TREE",
    "RRT_GOAL_TREE",
    "RRT_AMBI_TREE",
};

static std::string RRT_Tree_type_to_string(RRT_Tree_t type)
{
    if( 0 <= type && type < RRT_TREE_TYPE_SIZE )
        return RRT_Tree_type_string[type];
    else
        return "Unknown Type";
}

inline std::ostream& operator<<(std::ostream& oStrStream, const RRT_Tree_t type)
{
    oStrStream << RRT_Tree_type_to_string(type);
    return oStrStream;
}



class RRTNode
{
public:
    
    typedef enum {
        
        NORMAL=0,
        KEY
        
    } point_t;
    
    point_t type;

    JointConfig config;
    const JointConfig& getConfig() const;
    
    RRTNode(JointConfig mConfig, double maxStepSize=0.3, RRTNode* mParent = NULL);

    void giveParent(RRTNode* newParent);

    // If the node does not have a parent, this will return itself
    const RRTNode* getParent() const;
    // Always check for hasParent() before calling getParent()
    bool hasParent() const;

    // If the node does not have a child, this will return itself
    const RRTNode* getChild(size_t child) const;
    // Always check for hasChild() before calling getChild(~)
    bool hasChild() const;
    size_t numChildren() const;

    // Identify the node that's closest and scale down newConfig to be
    // an acceptable child to that node
    RRTNode& getClosestNodeAndScaleConfig(JointConfig& newConfig);
    
    // Identify the node that's closest in this subtree and return the rating
    // of that node
    double getClosestNode(RRTNode *&closest, const JointConfig& newConfig);


    // Add the new config as this node's child. Returns false if
    // the child configuration needed to be changed in order to respect
    // the maximum step size. If false is returned, the child was not
    // added and childConfig has been modified so that it is an
    // acceptable child. childConfig should then be collision tested
    // again.
    RRTNode* attemptAddChild(const JointConfig& childConfig);
    
    // This will scale the childConfig to be a suitable child but will not
    // attempt to add it as a child.
    // Returns true if childConfig was already suitable, false otherwise
    bool scaleJointConfig(JointConfig& childConfig);

    // Sets the max step size of this node and all its children to
    // the new value so it will be applied to all future iterations
    void setMaxStepSize(double newMaxStepSize);
    
protected:

    // Used to add a verified config as a child to this node
    RRTNode* addChild(const JointConfig& childConfig);
    double maxStepSize_;

    bool hasParent_;
    bool hasChildren_;
    std::vector<RRTNode*> children;
    RRTNode* parent;
    
    double numPrecThresh;
};


class RRTManager
{
public:
    
    RRTManager(int maxTreeSize=10000,
               double maxStepSize=0.3,
               double collisionCheckStepSize=0.3);

    void setDomain(const JointConfig& minJointConfig,
                   const JointConfig& maxJointConfig,
                   unsigned long resolution=10000000);
    
    void getDomain(JointConfig& minJointConfig,
                   JointConfig& maxJointConfig) const;
    
    // Add a desired start configuration for a plan
    // Returns the tree index of this new tree
    int addStartTree(JointConfig startConfiguration);

    // Add a desired goal configuration for a plan
    // Returns the tree index of this new tree
    int addGoalTree(JointConfig goalConfiguration);
    
    // Add any kind of tree you would like
    // Returns the tree index of this new tree
    // Or it returns -1 if it was outside the domain
    // Or it returns -2 if a collision was detected
    // Or it returns -3 if the constraintProjector was unsatisfied
    int addTree(JointConfig rootNodeConfiguration, RRT_Tree_t treeType);

    virtual RRT_Result_t checkStatus() const;
    // Add a node to each tree while checking for connections between trees
    virtual RRT_Result_t growTrees();
    
    // Once a solution is found, you will want to apply shortening to it
    virtual void shortenSolution();

    // You should plug a collision checker in here
    virtual bool collisionChecker(JointConfig& config, const JointConfig& parentConfig);

    // You should use this to impose constraints if desired
    virtual bool constraintProjector(JointConfig& config, const JointConfig& parentConfig);

    // This vector will be non-zero once a complete solved plan has been found
    ConfigPath solvedPlan;

    
    // Change the max step size for future steps
    void setMaxStepSize(double newMaxStepSize);
    double getMaxStepSize() const;
    // Maximum size that a tree may grow to
    int maxTreeSize_;
    // Number of total nodes allowed across all the trees. -1 means disabled
    int maxTotalNodes_;
    // Minimum step size of required collision checks between two nodes
    double collisionCheckStepSize_;
    // Number of times that growTree() can be called before giving up. -1 means disabled
    int maxIterations_;
    // Resolution that should be used for partitioning the domain
    unsigned long int dResolution;
    
    
    int getTreeSize(size_t tree) const;
    RRT_Tree_t getTreeType(size_t tree) const;
    int getTotalNodes() const;
    int getNumIterations() const;
    size_t getNumTrees() const;
    const RRTNode* getTree(size_t tree) const;
    
    // Check if a particular tree is maxed out
    bool checkIfMaxed(int tree) const;
    // Check if all the trees are maxed out
    bool checkIfAllMaxed() const;
    // Check if any of the trees are maxed out
    bool checkIfAnyMaxed() const;
    // Check if the manager has identified a solution yet
    bool checkIfSolved() const;
    
    void randomizeConfig(JointConfig& config);
    void scaleConfig(JointConfig& config, const JointConfig& parentConfig);
    void stepConfigTowards(JointConfig& config, const JointConfig& targetConfig);
    bool checkIfInRange(const JointConfig& configA, const JointConfig& configB);
    bool checkIfInDomain(const JointConfig& config, bool verbose=true);
    bool checkDimensionality(const JointConfig& config, bool verbose=true);

protected:

    std::vector<RRTNode*> trees;
    std::vector<int> treeSizeCounter;
    std::vector<RRT_Tree_t> treeTypeTracker;
    

    RRTNode* lookForTreeConnection(const RRTNode* targetNode, RRT_Tree_t connectTargetType);
    virtual RRTNode* attemptConnect(RRTNode*& node, const JointConfig& target, size_t treeID);
    void constructSolution(const RRTNode* beginTree, const RRTNode* endTree);
    bool _hasSolution;
    bool _invalidRoot;

    JointConfig minConfig;
    JointConfig maxConfig;
    int _domainSize;
    bool _hasDomain;
    int _iterations;

    double _numPrecThresh;

    // Maximum step size through the jointspace
    double _maxStepSize;

    std::vector<int> currentTreeSize;
    
};


#endif // RRTMANAGER_H
