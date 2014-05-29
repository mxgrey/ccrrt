
#include "../ccrrt/MultiChomper.h"
#include <iostream>

using namespace ccrrt;

bool MultiChomper::run(const Trajectory &trajectory,
                              Constraint *constraint,
                              double max_step_size)
{
    _waypoints.clear();

    assert( (size_t)trajectory.xi.size() == trajectory.state_space );

    _start = trajectory.start;
    _end = trajectory.end;
    _waypoints.push_back(trajectory.xi);

    _max_step = max_step_size;

    _constraint = constraint;

    return _go();
}


bool MultiChomper::_go()
{
    bool success = false;
    bool quit = false;
    Waypoint waypoint = _waypoints.begin();
    Trajectory next_traj;
    next_traj.start = _start;
    next_traj.end = _end;
    next_traj.xi = *waypoint;
    next_traj.state_space = _start.size();
    next_traj.waypoints = 1;

    size_t debug_count = 0;
    Eigen::VectorXd wp1, wp2;
    while(!success)
    {
        ++debug_count;
//        std::cout << "Loop " << debug_count << std::endl;
        size_t counter=0, max=10;
        Chomper::initialize(next_traj, _constraint);
        while(Chomper::iterate(true) == Constraint::INVALID)
        {
            ++counter;
            if(counter > max)
            {
                std::cout << "failed" << std::endl;
                quit = true;
                break;
            }
        }
        
        *waypoint = _trajectory.xi;
        if(quit)
            break;

        success = true;
        Waypoint last_check = _waypoints.begin();
        for(Waypoint check=_waypoints.begin(); check != _waypoints.end(); ++check)
        {
            if(check == _waypoints.begin())
            {
                wp1 = _start;
                wp2 = *check;
            }
            else
            {
                wp1 = *last_check;
                wp2 = *check;
                ++last_check;
            }

            if((wp2-wp1).norm() > _max_step)
            {
                success = false;
                next_traj.start = wp1;
                next_traj.end = wp2;
                next_traj.xi = (wp1+wp2)/2;
                _waypoints.insert(check, 1, next_traj.xi);
                waypoint = check;
                --waypoint;
                break;
            }
        }

        if(!success)
            continue;

        wp1 = _waypoints.back();
        wp2 = _end;
        if((wp1-wp2).norm() > _max_step)
        {
            success = false;
            next_traj.start = wp1;
            next_traj.end = wp2;
            next_traj.xi = (wp1+wp2)/2;
            _waypoints.push_back(next_traj.xi);
            waypoint = _waypoints.end();
            --waypoint;
            continue;
        }
    }

    size_t ss = _start.size();
    _trajectory.state_space = ss;
    _trajectory.start = _start;
    _trajectory.end = _end;
    _trajectory.xi.resize(_waypoints.size()*ss);
    size_t counter=0;
    for(waypoint=_waypoints.begin(); waypoint != _waypoints.end(); ++waypoint)
    {
        _trajectory.xi.block(counter*ss,0,ss,1) = *waypoint;
        ++counter;
    }
    _trajectory.waypoints = counter;
    std::cout << "final count: " << counter << std::endl;

    return success;
}

Eigen::VectorXd MultiChomper::_interpolate(const Eigen::VectorXd &start,
                                           const Eigen::VectorXd &end)
{
    return (end+start)/2;
}
