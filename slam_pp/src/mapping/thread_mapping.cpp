#include "mapping/thread_mapping.hpp"

using namespace uavos;

void Mapping_Controller::start(){
    m_mapping_thread = boost::thread(&Mapping_Controller::thread_mapping, this);
}

void Mapping_Controller::thread_mapping(){

    // spin for this thread
    const double timeout = 0.001;
    ros::WallDuration timeout_duration(timeout);
    while(m_nh.ok()){
        m_mapping_callback_queue.callAvailable(timeout_duration);
    }
}
