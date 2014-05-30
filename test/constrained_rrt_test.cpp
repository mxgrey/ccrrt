
#include "../ccrrt/ConstrainedRRT.h"
#include "../ccrrt/ConstraintGroup.h"
#include "../ccrrt/CircleConstraint.h"
#include "../ccrrt/Drawer.h"

using namespace ccrrt;
using namespace Eigen;

int main(int argc, char* argv[])
{
    ConstraintGroup group;

    std::vector<CircleConstraint*> circles;
    
    // Infinitesimal Passage
    circles.push_back(new CircleConstraint(Vector2d(0,0), 2, 0.1));
    circles.push_back(new CircleConstraint(Vector2d(-1,2), 1, 0.1));
    circles.push_back(new CircleConstraint(Vector2d(1,2), 2, 0.1));
    circles.push_back(new CircleConstraint(Vector2d(1,4.00), 1, 0.1));
    circles.push_back(new CircleConstraint(Vector2d(2.8,-0.3), 1, 0.1));
    circles.push_back(new CircleConstraint(Vector2d(4.2,-0.3), 1, 0.1));
    circles.push_back(new CircleConstraint(Vector2d(-1,-2), 1, 0.1));
    circles.push_back(new CircleConstraint(Vector2d(1.3,-2.3), 1, 0.1));
    
    // Super-Tiny Passage
//    circles.push_back(new CircleConstraint(Vector2d(0,0), 2, 0.1));
//    circles.push_back(new CircleConstraint(Vector2d(-1,2), 1, 0.1));
//    circles.push_back(new CircleConstraint(Vector2d(1,2), 2, 0.1));
//    circles.push_back(new CircleConstraint(Vector2d(1,3.99), 1, 0.1));
//    circles.push_back(new CircleConstraint(Vector2d(2.8,-0.3), 1, 0.1));
//    circles.push_back(new CircleConstraint(Vector2d(4.2,-0.3), 1, 0.1));
//    circles.push_back(new CircleConstraint(Vector2d(-1,-2), 1, 0.1));
//    circles.push_back(new CircleConstraint(Vector2d(1.3,-2.3), 1, 0.1));
    
    
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

    ConstrainedRRT rrt;
    Eigen::VectorXd limits(2);
    limits << 5, 5;
    rrt.setDomain(-limits,limits);
    rrt.setConstraint(&group);
    
    Eigen::VectorXd p(2);
    p << -3, 0;
    rrt.addStartTree(p);
    p << 3.4, 1.2;
    rrt.addGoalTree(p);
    
    std::cout << "Start" << std::endl;
    RRT_Result_t result = RRT_NOT_FINISHED;
    size_t counter=0;
    while(RRT_NOT_FINISHED == result)
    {
        ++counter;
        result = rrt.growTrees();
    }
    
    if(result != RRT_SOLVED)
        std::cout << "Failure: " << result << std::endl;
    
    std::cout << "Steps: " << counter << std::endl;
    
    Drawer draw;

    for(size_t i=0; i<circles.size(); ++i)
    {
        draw.draw_circle(*circles[i]);
    }

    draw.draw_rrts(rrt);

    draw.run();

    return 0;
}
