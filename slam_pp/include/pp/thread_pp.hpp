#ifndef THREAD_PP_HPP
#define THREAD_PP_HPP

#include "mapping/thread_mapping.hpp"
#include "basic_function.hpp"
#include "pp/pp_abstract.hpp"
#include "pp/simple_planner.hpp"

#include "boost/thread/thread.hpp"

namespace uavos{

class PP_Controller{
public:
    explicit PP_Controller(boost::shared_ptr<Mapping_Controller> p_mapping_controller):m_pp_thread(), m_nh("~pp"){
        m_stop_thread = false;
        m_nh.setCallbackQueue(&m_pp_callback_queue);

    }
    ~PP_Controller(){
        m_stop_thread = true;
        if(m_pp_thread.joinable()){
            m_pp_thread.join();
        }
    }

    void start();

private:

    ros::NodeHandle m_nh;
    ros::CallbackQueue m_pp_callback_queue;

    bool m_stop_thread;
    boost::thread m_pp_thread;

    void thread_pp();
};


}

#endif // THREAD_PP_HPP
