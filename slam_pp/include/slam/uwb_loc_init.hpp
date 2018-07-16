#ifndef UWB_LOC_INIT_HPP
#define UWB_LOC_INIT_HPP


#include "slam/anchor_triangulation_error_term.hpp"
#include "uwb_node.hpp"
#include "basic_function.hpp"

namespace uavos{

class UWB_Loc_Init{
public:
    // this class should be constructed and destructed before creating the range info callback
    explicit UWB_Loc_Init(ros::NodeHandle& nh,
                          boost::shared_ptr<uavos::UWB_Mobile> p_mobile):
        m_p_mobile(p_mobile), m_nh(nh){
        m_mobile_position = m_p_mobile->getPosition();
        m_ndb_sub = m_nh.subscribe("/time_domain/NDB",
                                   10,
                                   &uavos::UWB_Loc_Init::ndbCallback,
                                   this);
    }
    ~UWB_Loc_Init(){
        ROS_INFO("UWB Localization Initialization module is now destructed.");
    }

private:
    ros::NodeHandle m_nh;
    boost::shared_ptr<ros::CallbackQueue> m_p_callback_queue;
    boost::shared_ptr<uavos::UWB_Mobile> m_p_mobile;
    geometry_msgs::PointStamped m_mobile_position;

    ros::Subscriber m_ndb_sub;
    common_msgs::UWB_FullNeighborDatabase m_ndb;

    ceres::Problem m_problem;

public:
    inline void ndbCallback(const common_msgs::UWB_FullNeighborDatabase::ConstPtr& msg){
        m_ndb = *msg;
    }
    inline void spinOnce(){
        ((ros::CallbackQueue*)m_nh.getCallbackQueue())->callAvailable();
    }

    bool initializeMobileByCeres();
    bool getValidNeighborDatabse();
    void buildOptimizationProblem(const common_msgs::UWB_FullNeighborDatabase& ndb);
    bool solveOptimizationProblem();
};


}

#endif // UWB_LOC_INIT_HPP
