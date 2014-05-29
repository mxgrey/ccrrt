
#include "../ccrrt/RRTManager.h"
#include "../ccrrt/Drawer.h"

using namespace ccrrt;

int main(int argc, char* argv[])
{
    RRTManager rrt;
    Eigen::VectorXd limits(2);
    limits << 5, 5;
    
    rrt.setDomain(-limits,limits);
    
    Eigen::VectorXd p(2);
    p << -3, 0;
    rrt.addStartTree(p);
    p << 3, -1;
    rrt.addGoalTree(p);
    
    RRT_Result_t result = RRT_NOT_FINISHED;
    while(RRT_NOT_FINISHED == result)
    {
        result = rrt.growTrees();
        std::cout << result << std::endl;
    }
    
    Drawer draw;
    draw.draw_rrts(rrt);
    
    draw.run();
    
    return 0;
}
