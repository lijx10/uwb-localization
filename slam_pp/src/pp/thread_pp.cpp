#include "pp/thread_pp.hpp"

using namespace uavos;

void PP_Controller::start(){
    m_pp_thread = boost::thread(&PP_Controller::thread_pp, this);
}

void PP_Controller::thread_pp(){

    // choose planner
    std::string planner_name;
    m_nh.param<std::string>("planner", planner_name, "NoPlanner");

    boost::shared_ptr<NoPlanner> p_no_planner;

    if(std::string("NoPlanner")==planner_name){
        ROS_INFO("NoPlanner is running.");
        p_no_planner = boost::shared_ptr<NoPlanner>(new NoPlanner(m_nh));
    } else {
        ROS_ERROR("Unknown planner! Will use NoPlanner.");
        p_no_planner = boost::shared_ptr<NoPlanner>(new NoPlanner(m_nh));
    }

    // spin for this thread
    const double timeout = 0.001;
    ros::WallDuration timeout_duration(timeout);
    while(m_nh.ok()){
        m_pp_callback_queue.callAvailable(timeout_duration);
    }
}
