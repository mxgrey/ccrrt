
#include "../ccrrt/MultiChomper.h"
#include <iostream>

using namespace ccrrt;

MultiChomper::MultiChomper()
{
    max_attempts = 5;
}

bool MultiChomper::run(const Trajectory &trajectory,
                        Constraint *constraint, 
                        const Eigen::VectorXd& min, 
                        const Eigen::VectorXd& max,
                        double max_step_size)
{
    _waypoints.clear();

    assert( (size_t)trajectory.xi.size() == trajectory.state_space );
    
    _min = min;
    _max = max;

    _start = trajectory.start;
    _end = trajectory.end;
    _waypoints.push_back(trajectory.xi);

    _max_step = max_step_size;

    _constraint = constraint;

    return _go();
}


bool MultiChomper::_go()
{
    bool success = false, stuck = false;
    bool quit = false;
    
    bool bad_wp = false;
    double norm_check = (_start-_end).norm();
    bool print = false;
    
    Waypoint waypoint = _waypoints.begin();
    Trajectory next_traj;
    next_traj.start = _start;
    next_traj.end = _end;
    next_traj.xi = *waypoint;
    next_traj.state_space = _start.size();
    next_traj.waypoints = 1;

    size_t debug_count = 0;
    size_t hits = 0;
    Eigen::VectorXd wp1, wp2;
    Eigen::VectorXd last_added_waypoint = next_traj.xi;
    Constraint::validity_t valid;
    while(!success && !stuck)
    {
        success = false;
        bad_wp = false;
        ++debug_count;
        if(debug_count >= 100)
        {
            debug_count = 0;
            ++hits;
            
            double distl;
            double distg = 0;
            Waypoint last_check = _waypoints.begin();
            size_t wpc = 0;
            for(Waypoint check=_waypoints.begin(); check != _waypoints.end(); ++check)
            {
                ++wpc;
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
                
                distl = (wp2-wp1).norm();
                distg += distl;
                
                if(print)
                    std::cout << wpc << "(" << distl << ")" << ": " << wp2.transpose() 
                              << "\t|\t" << (wp2-wp1).transpose() << std::endl;
            }
        }
        
        
        size_t counter=0;
//        norm_check = (next_traj.start-next_traj.end).norm();
        valid = Chomper::initialize(next_traj, _constraint, _min, _max);
        while(valid == Constraint::INVALID)
//        while(counter < max_attempts)
        {
            valid = Chomper::iterate(true);
            ++counter;
            if(counter >= max_attempts || valid == Constraint::STUCK)
            {
                quit = true;
                bad_wp = true;
                break;
            }
            
//            if( (_trajectory.start-_trajectory.xi).norm() > norm_check
//                    || (_trajectory.end-_trajectory.xi).norm() > norm_check )
            if( (_end-_trajectory.xi).norm() > norm_check )
            {
                quit = true;
                bad_wp = true;
                break;
            }
        }
        
        
        if(!bad_wp)
            *waypoint = _trajectory.xi;
        
        if(quit)
            break;
        
        size_t loop_count = 0;
        for(Waypoint check=_waypoints.begin(); check != _waypoints.end(); ++check)
        {
            ++loop_count;
            if(check == waypoint)
                continue;
            
            if( (*check - *waypoint).norm() < _max_step/10 )
            {
//                std::cout << "HIT LOOP " << loop_count << std::endl;
                *waypoint = last_added_waypoint;
                quit = true;
                break;
            }
        }
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
            
//            if( (wp1-*waypoint).norm() < _max_step/10 )
//            {
//                *waypoint = last_added_waypoint;
//                stuck = true;
//                break;
//            }

            if((wp2-wp1).norm() > _max_step)
            {
                success = false;
                next_traj.start = wp1;
                next_traj.end = wp2;
                next_traj.xi = (wp1+wp2)/2;
                if( (next_traj.xi-last_added_waypoint).norm() < 1e-6 )
                {
                    stuck = true;
                    break;
                }

                _waypoints.insert(check, 1, next_traj.xi);
                last_added_waypoint = next_traj.xi;
                waypoint = check;
                --waypoint;
                break;
            }
        }

        if(stuck)
            break;

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
            
//            if( (wp1-*waypoint).norm() < _max_step/10 )
//            {
//                *waypoint = last_added_waypoint;
//                stuck = true;
//                break;
//            }
            
            if( (next_traj.xi-last_added_waypoint).norm() < 1e-6 )
            {
                stuck = true;
                break;
            }

            _waypoints.push_back(next_traj.xi);
            last_added_waypoint = next_traj.xi;
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
//    std::cout << "final count: " << counter << std::endl;

//    if(stuck)
//        std::cout << "Got stuck!!" << std::endl;

    return success && !stuck;
}
