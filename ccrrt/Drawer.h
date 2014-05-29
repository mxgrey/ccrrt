#ifndef CCRRT_DRAWER_H
#define CCRRT_DRAWER_H

#include "osgAkin/Line.h"
#include "../ccrrt/Trajectory.h"
#include "../ccrrt/CircleConstraint.h"
#include "../ccrrt/RRTManager.h"

namespace ccrrt {

class Drawer
{
public:
    
    Drawer();
    
    void draw_trajectory(const Trajectory& traj, 
                         const osg::Vec4& color=osg::Vec4(0.1f,0.1f,0.1f,1.0f));
    
    void draw_path(const ConfigPath& path, 
                   const osg::Vec4& color=osg::Vec4(0.2,0.4,0.2,1.0));
    
    void draw_rrts(const RRTManager& mgr,
                   const osg::Vec4& tree_color=osg::Vec4(0.2,0.7,0.2,1.0),
                   const osg::Vec4& path_color=osg::Vec4(0.2,0.4,0.2,1.0));
    
    void draw_tree(const RRTNode& root_node,
                   const osg::Vec4& color=osg::Vec4(0.2,0.7,0.2,1.0));
    
    void draw_circle(const CircleConstraint& circle);
    
    void draw_vector(const Eigen::Vector2d& vec, const Eigen::Vector2d& origin);
    
    
    void run();
    
protected:
    
    osgViewer::Viewer viewer;
    osg::Group* _root;
    osg::Geode* _geode;
    
    std::vector<osgAkin::Line*> _lines;
};


} // namespace ccrrt

#endif // CCRRT_DRAWER_H
