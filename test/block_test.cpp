
#include "../ccrrt/ConstraintGroup.h"
#include "../ccrrt/BlockConstraint.h"

#include "../ccrrt/ChompRRT.h"
#include "../ccrrt/ConstrainedRRT.h"
#include "../ccrrt/CBiRRT.h"

#include "../ccrrt/Drawer.h"

using namespace ccrrt;
using namespace Eigen;

int main(int argc, char* argv[])
{
    ChompRRT rrt;
//    CBiRRT rrt;
//    ConstrainedRRT rrt;
    
    ConstraintGroup group;
    
    std::vector<BlockConstraint*> block_constraints;
    
    block_constraints.push_back(new BlockConstraint(Vector2d(0,-0.1), 0, 1, 0.5, 0.1));
    block_constraints.push_back(new BlockConstraint(Vector2d(0, 0.41), 0, 1, 0.5, 0.1));
    block_constraints.push_back(new BlockConstraint(Vector2d(0.77,0.2), 0, 0.5, 1, 0.1));
    block_constraints.push_back(new BlockConstraint(Vector2d(-0.77,0.2), 0, 0.5, 1, 0.1));
    
    block_constraints.push_back(new BlockConstraint(Vector2d(0, 3), 0, 1, 5, 0.1));
    block_constraints.push_back(new BlockConstraint(Vector2d(0, -2.6), 0, 1, 5, 0.1));
    
    
//    rrt.multichomp.max_attempts = 10;
    
    for(size_t i=0; i<block_constraints.size(); ++i)
    {
        group.addConstraint(block_constraints[i]);
    }
    
    Eigen::VectorXd limits(2);
    limits << 5,5;
    rrt.setDomain(-limits,limits);
    rrt.setConstraints(&group);
    
    Eigen::VectorXd p(2);
    p << -2, 0;
    rrt.addStartTree(p);
    p << 2, 0;
    rrt.addGoalTree(p);
    
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
        
//        if(counter > 100)
//            break;
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

    for(size_t i=0; i<block_constraints.size(); ++i)
    {
        draw.draw_block(*block_constraints[i]);
    }

    draw.run();
    
    return 0;
}
