

#include "../ccrrt/ChompRRT.h"
#include "../ccrrt/ConstraintGroup.h"
#include "../ccrrt/CircleConstraint.h"
#include "../ccrrt/Drawer.h"

using namespace ccrrt;
using namespace Eigen;

int main(int argc, char* argv[])
{
    ConstraintGroup group;

    std::vector<CircleConstraint*> circles;
    circles.push_back(new CircleConstraint(Vector2d(0,0), 2, 0.1));
    circles.push_back(new CircleConstraint(Vector2d(-1,2), 1, 0.1));
    circles.push_back(new CircleConstraint(Vector2d(1,2), 2, 0.1));
    circles.push_back(new CircleConstraint(Vector2d(3.3,-0.3), 1, 0.1));
    circles.push_back(new CircleConstraint(Vector2d(-1,-2), 1, 0.1));
    circles.push_back(new CircleConstraint(Vector2d(2.7,-2.3), 1, 0.1));

    for(size_t i=0; i<circles.size(); ++i)
    {
        group.addConstraint(circles[i]);
    }

    ChompRRT rrt;
//    rrt.maxIterations_ = 5;
    Eigen::VectorXd limits(2);
    limits << 5, 5;
    rrt.setDomain(-limits,limits);
    rrt.setConstraint(&group);

    Trajectory vis;

    Eigen::VectorXd p(2);
    p << -3, 0;
    vis.start = p;
    rrt.addStartTree(p);
    p << 3.4, 1.2;
    vis.end = p;
    rrt.addGoalTree(p);

    vis.state_space = p.size();
    vis.waypoints = 1;
    vis.xi = (vis.start+vis.end)/2;

    Drawer draw;
//    draw.draw_trajectory(vis);

    RRT_Result_t result = RRT_NOT_FINISHED;
    size_t counter=0;
    while(RRT_NOT_FINISHED == result)
    {
        ++counter;
        result = rrt.growTrees(vis);
    }

    if(result != RRT_SOLVED)
        std::cout << "Failure: " << result << std::endl;

    std::cout << "Steps: " << counter << std::endl;

    draw.draw_rrts(rrt);

    for(size_t i=0; i<circles.size(); ++i)
    {
        draw.draw_circle(*circles[i]);
    }

    draw.run();

    return 0;
}
