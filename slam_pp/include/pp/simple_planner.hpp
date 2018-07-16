#ifndef SIMPLE_PLANNER_HPP
#define SIMPLE_PLANNER_HPP

#include "pp_abstract.hpp"

namespace uavos{

class NoPlanner : public PathPlanning{
public:
    explicit NoPlanner(ros::NodeHandle& nh):PathPlanning(nh){
        // load parameter
        nh.param<double>("no_planner_freq", m_no_planner_freq, 10);

        m_no_planner_timer = nh.createTimer(ros::Duration(0.05), &NoPlanner::noPlannerCallback, this);
    }
    ~NoPlanner(){
        ROS_INFO("NoPlanner is killed................................");
    }

private:
    double m_no_planner_freq;
    ros::Timer m_no_planner_timer;

public:
    void noPlannerCallback(const ros::TimerEvent &event);

};

}

#endif // SIMPLE_PLANNER_HPP
