
#include "../ccrrt/Drawer.h"
#include "osg/Point"

using namespace ccrrt;
using namespace Eigen;

Drawer::Drawer()
{
    _root = new osg::Group;
    _geode = new osg::Geode;
    _root->addChild(_geode);
    _geode->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    
    _geode->getOrCreateStateSet()->setAttributeAndModes(new osg::LineWidth(3.0f));
    _geode->getOrCreateStateSet()->setAttributeAndModes(new osg::Point(5.0f));
    
    
    viewer.getCamera()->setClearColor(osg::Vec4(0.9f,0.9f,0.9f,1.0f));
    viewer.setSceneData(_root);
    
    _markers = new osg::Geometry;
    _points = new osg::Vec3Array;
    _pointColors = new osg::Vec4Array;
    _geode->addDrawable(_markers);
}

void Drawer::draw_trajectory(const Trajectory &traj, const osg::Vec4 &color)
{
    osgAkin::Line* line = new osgAkin::Line(
                              akin::Translation(traj.start[0],0,traj.start[1]));
    line->setColor(color);

    for(size_t i=0; i < traj.waypoints; ++i)
    {
        line->addVertex(akin::Translation(traj.xi[2*i],0,traj.xi[2*i+1]));
    }

    line->addVertex(akin::Translation(traj.end[0],0,traj.end[1]));

    line->updateVertices();
    _geode->addDrawable(line);
}

void Drawer::draw_path(const ConfigPath &path, const osg::Vec4 &color)
{
    osgAkin::Line* line = new osgAkin::Line;
    
    line->setColor(color);
    
    for(size_t i=0; i < path.size(); ++i)
    {
        line->addVertex(akin::Translation(path[i][0],0,path[i][1]));
    }
    
    line->updateVertices();
    _geode->addDrawable(line);
}

void Drawer::draw_tree(const RRTNode &root_node, const osg::Vec4 &color)
{
    std::map<const RRTNode*,ushort> node_map;
    osgAkin::LineTree* tree = new osgAkin::LineTree;
    tree->setColor(color);
    std::vector<size_t> tracker;
    tracker.push_back(0);
    size_t depth = 0;
    const RRTNode* current_node = &root_node;
    
    node_map[&root_node] = tree->addVertex(akin::Translation(
                                               root_node.getConfig()[0],0,
                                           root_node.getConfig()[1]),-1);
    
    while(tracker.size() > 0)
    {
        if(tracker[depth] < current_node->numChildren())
        {
            current_node = current_node->getChild(tracker[depth]);
            node_map[current_node] = tree->addVertex(akin::Translation(
                                        current_node->getConfig()[0],0,
                                        current_node->getConfig()[1]),
                                     node_map[current_node->getParent()]);
            
//            std::cout << "D:" << depth << " T:" << tracker[depth] 
//                      << " C:" << current_node->numChildren() << " | " 
//                      << current_node->getConfig().transpose() << std::endl;
            
            osg::Vec4 mColor = color;
            if(current_node->type == RRTNode::NORMAL)
                mColor = osg::Vec4(0.1,0.7,0.7,1.0);
            
            if(current_node->type == RRTNode::KEY)
                mColor = osg::Vec4(0.8,0.1,0.1,1.0);
            
            draw_marker(current_node->getConfig(), 0.025, mColor);
            
            ++tracker[depth];
            tracker.push_back(0);
            ++depth;
        }
        else
        {
            current_node = current_node->getParent();
            tracker.pop_back();
            --depth;
        }
    }
    
    tree->updateVertices();
    _geode->addDrawable(tree);
}

void Drawer::draw_rrts(const RRTManager& mgr,
                       const osg::Vec4& tree_color,
                       const osg::Vec4& path_color)
{
    if(mgr.checkIfSolved())
        draw_path(mgr.solvedPlan, path_color);
    
    for(size_t i=0; i<mgr.getNumTrees(); ++i)
    {
        osg::Vec4 treeColor = tree_color;
        if(mgr.getTreeType(i) == RRT_START_TREE)
            treeColor = osg::Vec4(0.8,0.8,0.1,1.0);
        else if(mgr.getTreeType(i) == RRT_GOAL_TREE)
            treeColor = osg::Vec4(0.1,1.0,0.1,1.0);
        
        draw_tree(*mgr.getTree(i),treeColor);
    }
    
    osgAkin::Line* domain = new osgAkin::Line;
    Eigen::VectorXd min;
    Eigen::VectorXd max;
    
    mgr.getDomain(min, max);
    domain->addVertex(akin::Translation(min[0],0,min[1]));
    domain->addVertex(akin::Translation(min[0],0,max[1]));
    domain->addVertex(akin::Translation(max[0],0,max[1]));
    domain->addVertex(akin::Translation(max[0],0,min[1]));
    domain->addVertex(akin::Translation(min[0],0,min[1]));
    domain->setColor(osg::Vec4(0.8,0.1,0.8,1.0));
    domain->updateVertices();
    _geode->addDrawable(domain);
    
    for(size_t i=0; i<mgr.getNumTrees(); ++i)
    {
        const RRTNode* tree = mgr.getTree(i);
        osg::Vec4 color = osg::Vec4(0.8f, 0.1f, 0.8f, 1.0f);
        if(mgr.getTreeType(i) == RRT_START_TREE)
        {
            color = osg::Vec4(0.8,0.8,0.1,1.0);
        }
        else if(mgr.getTreeType(i) == RRT_GOAL_TREE)
        {
            color = osg::Vec4(0.1,1.0,0.1,1.0);
        }
            
        draw_marker(Eigen::Vector2d(tree->getConfig()[0],tree->getConfig()[1]),
                mgr.getMaxStepSize(), color);
    }
    
}

void Drawer::draw_marker(const Eigen::Vector2d &position, double , const osg::Vec4 &color)
{
    _points->push_back(osg::Vec3(position[0],0,position[1]));
    _pointColors->push_back(color);
    
    osg::DrawElementsUInt* pointPrim = new osg::DrawElementsUInt(osg::PrimitiveSet::POINTS, 0);
    pointPrim->push_back(_points->size()-1);
    
    _markers->setVertexArray(_points);
    _markers->setColorArray(_pointColors);
    _markers->setColorBinding(osg::Geometry::BIND_PER_PRIMITIVE_SET);
    _markers->addPrimitiveSet(pointPrim);
}

void Drawer::draw_circle(const CircleConstraint& circle, const osg::Vec4& color)
{   
    osg::Geometry* geom = new osg::Geometry;
    osg::Vec3Array* verts = new osg::Vec3Array;
    verts->push_back(osg::Vec3(circle.center.x(),0,circle.center.y()));
    
    size_t res = 1000;
    for(size_t i=0; i<=res; ++i)
    {
        double x = circle.center.x() + circle.radius*cos(2*M_PI*i/res);
        double y = 0;
        double z = circle.center.y() + circle.radius*sin(2*M_PI*i/res);
        verts->push_back(osg::Vec3(x,y,z));
    }
    geom->setVertexArray(verts);
    
    osg::DrawElementsUShort* faces = new osg::DrawElementsUShort(osg::PrimitiveSet::TRIANGLES, 0);
    for(size_t i=1; i<verts->size(); ++i)
    {
        faces->push_back(0);
        faces->push_back(i);
        
        if(i < verts->size()-1)
            faces->push_back(i+1);
        else
            faces->push_back(1);
    }
    geom->addPrimitiveSet(faces);
    
    osg::Vec4Array* color_array = new osg::Vec4Array;
    color_array->push_back(color);
    geom->setColorArray(color_array);
    geom->setColorBinding(osg::Geometry::BIND_PER_PRIMITIVE_SET);
    
    _geode->addDrawable(geom);
}

void Drawer::draw_block(const BlockConstraint &block, const osg::Vec4 &color)
{
    osg::Geometry* geom = new osg::Geometry;
    osg::Vec3Array* verts = new osg::Vec3Array;
    osg::Vec4Array* colors = new osg::Vec4Array;
    colors->push_back(color);
    
    Vector2d v = block.scales;
    Vector2d vt = block.getTf()*v;
    verts->push_back(osg::Vec3(vt[0],0,vt[1]));
    vt = block.getTf()*Vector2d(-v[0],v[1]);
    verts->push_back(osg::Vec3(vt[0],0,vt[1]));
    vt = block.getTf()*Vector2d(-v[0],-v[1]);
    verts->push_back(osg::Vec3(vt[0],0,vt[1]));
    vt = block.getTf()*Vector2d(v[0],-v[1]);
    verts->push_back(osg::Vec3(vt[0],0,vt[1]));
    
    geom->setVertexArray(verts);

    osg::DrawElementsUShort* faces = new osg::DrawElementsUShort(osg::PrimitiveSet::QUADS, 0);
    faces->push_back(0);
    faces->push_back(1);
    faces->push_back(2);
    faces->push_back(3);

    geom->addPrimitiveSet(faces);

    geom->setColorArray(colors);
    geom->setColorBinding(osg::Geometry::BIND_PER_PRIMITIVE_SET);

    _geode->addDrawable(geom);
}

void Drawer::draw_line_constraint(const LineConstraint &line, const osg::Vec4 &color)
{
    osg::Geometry* geom = new osg::Geometry;
    osg::Vec3Array* verts = new osg::Vec3Array;
    osg::Vec4Array* colors = new osg::Vec4Array;

    verts->push_back(osg::Vec3(line.start()[0],0,line.start()[1]));                 // 0
    colors->push_back(osg::Vec4(1,1,1,1));

//    verts->push_back(osg::Vec3(line.end[0],0,line.end[1]));                     // 1
//    colors->push_back(osg::Vec4(1,1,1,1));

//    Eigen::Vector2d v = line.end() - line.start();
//    Eigen::Vector2d perp(-v[1],v[0]);
//    perp = perp.normalized()*line.width()/2;
//    Eigen::Vector2d w = perp.normalized()*line.width()*line.parab_factor/2;

//    osg::Vec4 midcolor = (osg::Vec4(1,1,1,1)-color)/2 + color;
//    verts->push_back(osg::Vec3(line.start[0]+w[0],0,line.start[1]+w[1]));       // 2
//    colors->push_back(midcolor);

//    verts->push_back(osg::Vec3(line.end[0]+w[0],0,line.end[1]+w[1]));           // 3
//    colors->push_back(midcolor);

//    verts->push_back(osg::Vec3(line.start[0]-w[0],0,line.start[1]-w[1]));       // 4
//    colors->push_back(midcolor);

//    verts->push_back(osg::Vec3(line.end[0]-w[0],0,line.end[1]-w[1]));           // 5
//    colors->push_back(midcolor);

//    verts->push_back(osg::Vec3(line.start[0]+perp[0],0,line.start[1]+perp[1])); // 6
//    colors->push_back(color);

//    verts->push_back(osg::Vec3(line.end[0]+perp[0],0,line.end[1]+perp[1]));     // 7
//    colors->push_back(color);

//    verts->push_back(osg::Vec3(line.start[0]-perp[0],0,line.start[1]-perp[1])); // 8
//    colors->push_back(color);

//    verts->push_back(osg::Vec3(line.end[0]-perp[0],0,line.end[1]-perp[1]));     // 9
    colors->push_back(color);

    geom->setVertexArray(verts);

    osg::DrawElementsUShort* faces = new osg::DrawElementsUShort(osg::PrimitiveSet::QUADS, 0);
    faces->push_back(0);
    faces->push_back(1);
    faces->push_back(3);
    faces->push_back(2);

    faces->push_back(0);
    faces->push_back(1);
    faces->push_back(5);
    faces->push_back(4);

    faces->push_back(2);
    faces->push_back(3);
    faces->push_back(7);
    faces->push_back(6);

    faces->push_back(4);
    faces->push_back(5);
    faces->push_back(9);
    faces->push_back(8);

    geom->addPrimitiveSet(faces);

    geom->setColorArray(colors);
    geom->setColorBinding(osg::Geometry::BIND_PER_VERTEX);

    _geode->addDrawable(geom);
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
