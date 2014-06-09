
#include "../ccrrt/ConstraintGroup.h"
#include "../ccrrt/BlockConstraint.h"

#include "../ccrrt/CBiRRT.h"

#include "../ccrrt/Drawer.h"

using namespace ccrrt;
using namespace Eigen;

int main(int argc, char* argv[])
{
//    ChompRRT rrt;
    CBiRRT rrt;
//    ConstrainedRRT rrt;
    
    ConstraintGroup proj_group;
    ConstraintGroup obst_group;
    
    std::vector<BlockConstraint*> proj_constraints;
    std::vector<BlockConstraint*> obst_constraints;
    
    proj_constraints.push_back(new BlockConstraint(Vector2d(0,-0.1), 0, 1, 0.5, 0.1));
    proj_constraints.push_back(new BlockConstraint(Vector2d(0, 0.41), 0, 1, 0.5, 0.1));
    proj_constraints.push_back(new BlockConstraint(Vector2d(0.77,0.2), 0, 0.5, 1, 0.1));
    proj_constraints.push_back(new BlockConstraint(Vector2d(-0.77,0.2), 0, 0.5, 1, 0.1));
    
//    obst_constraints.push_back(new BlockConstraint(Vector2d(0, 3), 0, 1, 5, 0.1));
//    obst_constraints.push_back(new BlockConstraint(Vector2d(0, -2.6), 0, 1, 5, 0.1));
    proj_constraints.push_back(new BlockConstraint(Vector2d(0, 3), 0, 1, 5, 0.1));
    proj_constraints.push_back(new BlockConstraint(Vector2d(0, -2.6), 0, 1, 5, 0.1));
    
    
//    rrt.multichomp.max_attempts = 10;
    
    for(size_t i=0; i<proj_constraints.size(); ++i)
    {
        proj_group.addConstraint(proj_constraints[i]);
    }
    
    for(size_t i=0; i<obst_constraints.size(); ++i)
    {
        obst_group.addConstraint(obst_constraints[i]);
    }
    
    Eigen::VectorXd limits(2);
    limits << 5,5;
    rrt.setDomain(-limits,limits);
    rrt.setRejectionConstraints(&obst_group);
    rrt.setProjectionConstraints(&proj_group);
    
    
    Eigen::VectorXd p(2);
    p << -3, 0;
    std::cout << "Start: " << rrt.addStartTree(p) << std::endl;
    p << 3, 0;
    std::cout << "Goal: " << rrt.addGoalTree(p) << std::endl;
    
    Trajectory vis;
    
    Drawer draw;
    
    std::cout << "Start" << std::endl;
    RRT_Result_t result = RRT_NOT_FINISHED;
    size_t counter=0;

    clock_t start_time;
    start_time = clock();


    while(RRT_NOT_FINISHED == result)
    {
        ++counter;
//        result = rrt.growTrees(vis);
        result = rrt.growTrees();
    }

    clock_t end_time;
    end_time = clock();

    double total_time = (end_time-start_time)/(double)(CLOCKS_PER_SEC);
    if(result != RRT_SOLVED)
        std::cout << "Failure: " << result << " (" << total_time << ")" <<  std::endl;
    else
        std::cout << "Time: " << total_time << std::endl;

    std::cout << "Steps: " << counter << std::endl;

    draw.draw_rrts(rrt);

    for(size_t i=0; i<proj_constraints.size(); ++i)
    {
        draw.draw_block(*proj_constraints[i]);
    }
    
    for(size_t i=0; i<obst_constraints.size(); ++i)
    {
        draw.draw_block(*obst_constraints[i], osg::Vec4(0.8,0.1,0.1,1.0));
    }

    draw.run();
    
    return 0;
}
