#ifndef CCRRT_DRAWER_H
#define CCRRT_DRAWER_H

#include "osgAkin/Line.h"
#include "../ccrrt/Trajectory.h"
#include "../ccrrt/CircleConstraint.h"

namespace ccrrt {

class Drawer
{
public:
    
    Drawer();
//    ~Drawer();
    
    void draw_trajectory(const Trajectory& traj);
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
