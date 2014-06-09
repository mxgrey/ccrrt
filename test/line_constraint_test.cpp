
#include "../ccrrt/Drawer.h"
#include "../ccrrt/ConstraintGroup.h"
#include "../ccrrt/CBiRRT.h"
#include "time.h"

using namespace ccrrt;
using namespace Eigen;

int main(int argc, char* argv[])
{
    CBiRRT rrt;

    ConstraintGroup group;
    std::vector<LineConstraint*> line_constraints;

    line_constraints.push_back(new LineConstraint(Vector2d(0,-0.1), Vector2d(1,-0.1), 2, 1e-5));

    for(size_t i=0; i<line_constraints.size(); ++i)
    {
        group.addConstraint(line_constraints[i]);
    }


    Eigen::VectorXd limits(2);
    limits << 5,5;
    rrt.setDomain(-limits, limits);
    rrt.setRejectionConstraints(&group);

    Eigen::VectorXd p(2);
    p << -1, 0;
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

    for(size_t i=0; i<line_constraints.size(); ++i)
    {
        draw.draw_line_constraint(*line_constraints[i]);
    }

    draw.run();

    return 0;
}


