
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
            
//            draw_marker(current_node->getConfig(), 0.025, color);
            
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
    draw_path(mgr.solvedPlan, path_color);
    for(size_t i=0; i<mgr.getNumTrees(); ++i)
    {
        draw_tree(*mgr.getTree(i),tree_color);
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

void Drawer::draw_marker(const Eigen::Vector2d &position, double radius, const osg::Vec4 &color)
{
    CircleConstraint marker(position,radius);
    draw_circle(marker, color);
}

void Drawer::draw_circle(const CircleConstraint& circle, const osg::Vec4& color)
{
//    osgAkin::Line* line = new osgAkin::Line;
    
//    size_t res = 1000;
//    for(size_t i=0; i<=res; ++i)
//    {
//        double x = circle.center.x() + circle.radius*cos(2*M_PI*i/res);
//        double y = 0;
//        double z = circle.center.y() + circle.radius*sin(2*M_PI*i/res);
//        line->addVertex(akin::Translation(x,y,z));
//    }
    
//    line->setColor(color);
    
//    line->updateVertices();
//    _geode->addDrawable(line);
    
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
