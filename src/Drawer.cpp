
#include "../ccrrt/Drawer.h"

using namespace ccrrt;

Drawer::Drawer()
{
    _root = new osg::Group;
    _geode = new osg::Geode;
    _root->addChild(_geode);
    _geode->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    
    osg::LineWidth* lineWidth = new osg::LineWidth(3.0f);
    _geode->getOrCreateStateSet()->setAttributeAndModes(lineWidth);
    
    viewer.getCamera()->setClearColor(osg::Vec4(0.9f,0.9f,0.9f,1.0f));
    viewer.setSceneData(_root);
}

void Drawer::draw_trajectory(const Trajectory &traj)
{
    osgAkin::Line* line = new osgAkin::Line(
                              akin::Translation(traj.start[0],0,traj.start[1]));
    line->setColor(osg::Vec4(0.1f,0.1f,0.1f,1.0f));
    
    for(size_t i=0; i < traj.waypoints; ++i)
    {
        line->addVertex(akin::Translation(traj.xi[2*i],0,traj.xi[2*i+1]));
    }
    
    line->addVertex(akin::Translation(traj.end[0],0,traj.end[1]));
    
    line->updateVertices();
    _geode->addDrawable(line);
}

void Drawer::draw_circle(const CircleConstraint &circle)
{
    osgAkin::Line* line = new osgAkin::Line;
    
    size_t res = 1000;
    for(size_t i=0; i<=res; ++i)
    {
        double x = circle.center.x() + circle.radius*cos(2*M_PI*i/res);
        double y = 0;
        double z = circle.center.y() + circle.radius*sin(2*M_PI*i/res);
        line->addVertex(akin::Translation(x,y,z));
    }
    
    line->setColor(osg::Vec4(0.8f, 0.1f, 0.1f, 1.0f));
    
    line->updateVertices();
    _geode->addDrawable(line);
}

void Drawer::draw_vector(const Eigen::Vector2d &vec, const Eigen::Vector2d &origin)
{
    osgAkin::LineTree* vector = new osgAkin::LineTree;
    
    Eigen::Vector2d tip = vec + origin;
    vector->addVertex(akin::Translation(tip[0],0,tip[1]),-1);
    vector->addVertex(akin::Translation(origin[0],0,origin[1]),0);
    
    double norm = vec.norm();
    Eigen::Vector2d topfeather(-norm/8,norm/8);
    Eigen::Vector2d botfeather(-norm/8,-norm/8);
    
    double theta = atan2(vec[1],vec[0]);
    
    topfeather = Eigen::Rotation2D<double>(theta)*topfeather;
    botfeather = Eigen::Rotation2D<double>(theta)*botfeather;
    
    vector->addVertex(akin::Translation(tip[0]+topfeather[0],0,tip[1]+topfeather[1]),0);
    vector->addVertex(akin::Translation(tip[0]+botfeather[0],0,tip[1]+botfeather[1]),0);
    
    vector->setColor(osg::Vec4(0.1,0.1,0.8,1.0));
    
    vector->updateVertices();
    _geode->addDrawable(vector);
}

void Drawer::run()
{
    viewer.run();
}
