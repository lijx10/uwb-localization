#include "slam/thread_slam.hpp"



void uavos::SLAM_Controller::start(){
    m_slam_thread = boost::thread(&SLAM_Controller::thread_slam, this);
}

void uavos::SLAM_Controller::thread_slam(){
    // load parameters
    double slam_frequency;
    m_nh.param<double>("slam_fps", slam_frequency, 50);
    if(std::fabs(slam_frequency)<0.1){
        slam_frequency = 0.1;
    }
    bool is_enable_visualization;
    m_nh.param<bool>("is_enable_visualization", is_enable_visualization, false);

    boost::shared_ptr<uavos::UWB_Localization> p_uwb_localization(new uavos::UWB_Localization(m_nh));
    ros::Timer solve_slam_timer = m_nh.createTimer(ros::Duration(1.0/slam_frequency),
                                                   &uavos::UWB_Localization::solveSLAM,
                                                   p_uwb_localization);

    if(true==is_enable_visualization){
        ros::Timer visualization_timer = m_nh.createTimer(ros::Duration(0.02),
                                                          &uavos::SLAM_System::visualizeTrajectoryCallback,
                                                          boost::dynamic_pointer_cast<uavos::SLAM_System>(p_uwb_localization));
    }


    // spin for this thread
    const double timeout = 0.001;
    ros::WallDuration timeout_duration(timeout);
    while(m_nh.ok()){
        m_p_slam_callback_queue->callAvailable(timeout_duration);
    }
}
