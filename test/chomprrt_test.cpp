

#include "../ccrrt/ChompRRT.h"
#include "../ccrrt/ConstraintGroup.h"
#include "../ccrrt/CircleConstraint.h"
#include "../ccrrt/Drawer.h"

#include <time.h>

using namespace ccrrt;
using namespace Eigen;

int main(int argc, char* argv[])
{
    ConstraintGroup group;

    std::vector<CircleConstraint*> circles;
    
    // Infinitesimal Passage
//    circles.push_back(new CircleConstraint(Vector2d(0,0), 2, 0.1));
//    circles.push_back(new CircleConstraint(Vector2d(-1,2), 1, 0.1));
//    circles.push_back(new CircleConstraint(Vector2d(1,2), 2, 0.1));
//    circles.push_back(new CircleConstraint(Vector2d(1,4.00), 1, 0.1));
//    circles.push_back(new CircleConstraint(Vector2d(2.8,-0.3), 1, 0.1));
//    circles.push_back(new CircleConstraint(Vector2d(4.2,-0.3), 1, 0.1));
//    circles.push_back(new CircleConstraint(Vector2d(-1,-2), 1, 0.1));
//    circles.push_back(new CircleConstraint(Vector2d(1.3,-2.3), 1, 0.1));


    // Super-Tiny Passage
//    circles.push_back(new CircleConstraint(Vector2d(0,0), 2, 0.1));
//    circles.push_back(new CircleConstraint(Vector2d(-1,2), 1, 0.1));
//    circles.push_back(new CircleConstraint(Vector2d(1,2), 2, 0.1));
//    circles.push_back(new CircleConstraint(Vector2d(1,3.99), 1, 0.1));
//    circles.push_back(new CircleConstraint(Vector2d(2.8,-0.3), 1, 0.1));
//    circles.push_back(new CircleConstraint(Vector2d(4.2,-0.3), 1, 0.1));
//    circles.push_back(new CircleConstraint(Vector2d(-1,-2), 1, 0.1));
//    circles.push_back(new CircleConstraint(Vector2d(1.3,-2.3), 1, 0.1));

    // Multiple Narrow Passages
    circles.push_back(new CircleConstraint(Vector2d(-1.85,-0.98), 0.45, 0.1));
    circles.push_back(new CircleConstraint(Vector2d(-2.2,-0.55), 0.4, 0.1));
    circles.push_back(new CircleConstraint(Vector2d(-2.2,0.2), 0.4, 0.1));
    circles.push_back(new CircleConstraint(Vector2d(-1.8,0.6), 0.5, 0.1));
    circles.push_back(new CircleConstraint(Vector2d(-1.7,1.2), 0.5, 0.1));
    circles.push_back(new CircleConstraint(Vector2d(-1.5,2), 0.5, 0.1));
    circles.push_back(new CircleConstraint(Vector2d(-1.0,2.5), 0.5, 0.1));

    circles.push_back(new CircleConstraint(Vector2d(2.4,0.6), 0.5, 0.1));
    circles.push_back(new CircleConstraint(Vector2d(2.3,1.2), 0.5, 0.1));
    circles.push_back(new CircleConstraint(Vector2d(2.05,1.55), 0.3, 0.1));
    circles.push_back(new CircleConstraint(Vector2d(1.85,2), 0.5, 0.1));
    circles.push_back(new CircleConstraint(Vector2d(1.1,2.5), 0.5, 0.1));
    circles.push_back(new CircleConstraint(Vector2d(1.07,2.89), 0.3, 0.1));

    circles.push_back(new CircleConstraint(Vector2d(2.80,-0.85), 0.5, 0.1));
    circles.push_back(new CircleConstraint(Vector2d(2.05,-0.35), 0.5, 0.1));
    circles.push_back(new CircleConstraint(Vector2d(1.50,-0.0), 0.5, 0.1));
    circles.push_back(new CircleConstraint(Vector2d(1.40,0.5), 0.5, 0.1));
    circles.push_back(new CircleConstraint(Vector2d(1.29,1.15), 0.5, 0.1));
    circles.push_back(new CircleConstraint(Vector2d(1.10,1.69), 0.3, 0.1));
    circles.push_back(new CircleConstraint(Vector2d(0.685,1.585), 0.5, 0.1));

    circles.push_back(new CircleConstraint(Vector2d(0.0,3.2), 0.8, 0.1));
    circles.push_back(new CircleConstraint(Vector2d(1,4.2), 1, 0.1));
    circles.push_back(new CircleConstraint(Vector2d(3.0,0.25), 0.6, 0.1));
    circles.push_back(new CircleConstraint(Vector2d(4.2,-0.3), 1, 0.1));
    circles.push_back(new CircleConstraint(Vector2d(-1,-2), 1, 0.1));
    circles.push_back(new CircleConstraint(Vector2d(-0,-2.6), 0.6, 0.1));
    circles.push_back(new CircleConstraint(Vector2d(2.1,-1.6), 0.6, 0.1));
    circles.push_back(new CircleConstraint(Vector2d(1.3,-2.3), 1, 0.1));


    // Middle Narrow Passage
//    circles.push_back(new CircleConstraint(Vector2d(0,0), 2, 0.1));
//    circles.push_back(new CircleConstraint(Vector2d(-1,2), 1, 0.1));
//    circles.push_back(new CircleConstraint(Vector2d(1,2), 2, 0.1));
//    circles.push_back(new CircleConstraint(Vector2d(1,4.2), 1, 0.1));
//    circles.push_back(new CircleConstraint(Vector2d(3.0,-0.25), 1, 0.1));
//    circles.push_back(new CircleConstraint(Vector2d(4.2,-0.3), 1, 0.1));
//    circles.push_back(new CircleConstraint(Vector2d(-1,-2), 1, 0.1));
//    circles.push_back(new CircleConstraint(Vector2d(1.3,-2.3), 1, 0.1));


    // Simple
//    circles.push_back(new CircleConstraint(Vector2d(0,0), 2, 0.1));
//    circles.push_back(new CircleConstraint(Vector2d(-1,2), 1, 0.1));
//    circles.push_back(new CircleConstraint(Vector2d(3.1,-1), 1, 0.1));
//    circles.push_back(new CircleConstraint(Vector2d(-1,-2), 1, 0.1));
    
    for(size_t i=0; i<circles.size(); ++i)
    {
        group.addConstraint(circles[i]);
    }

    ChompRRT rrt;
//    rrt.maxIterations_ = 5;
    Eigen::VectorXd limits(2);
    limits << 5, 5;
    rrt.setDomain(-limits,limits);
    rrt.setConstraints(&group);

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

    std::cout << "Start" << std::endl;
    RRT_Result_t result = RRT_NOT_FINISHED;
    size_t counter=0;

    clock_t start_time;
    start_time = clock();

    while(RRT_NOT_FINISHED == result)
    {
        ++counter;
        result = rrt.growTrees();
        
//        if(counter > 50)
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

    for(size_t i=0; i<circles.size(); ++i)
    {
        draw.draw_circle(*circles[i]);
    }

    draw.run();

    return 0;
}
