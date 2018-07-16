#ifndef THREAD_MAPPING_HPP
#define THREAD_MAPPING_HPP

#include "basic_function.hpp"
#include "mapping/mapping_abstract.hpp"

namespace uavos{

class Mapping_Controller{
public:
    explicit Mapping_Controller():m_mapping_thread(), m_nh("~mapping"){
        m_stop_thread = false;
        m_nh.setCallbackQueue(&m_mapping_callback_queue);
    }
    ~Mapping_Controller(){
        m_stop_thread = true;
        if(m_mapping_thread.joinable()){
            m_mapping_thread.join();
        }
    }

    void start();

private:
    ros::NodeHandle m_nh;
    ros::CallbackQueue m_mapping_callback_queue;

    bool m_stop_thread;
    boost::thread m_mapping_thread;


    void thread_mapping();

};


}


#endif // THREAD_MAPPING_HPP
