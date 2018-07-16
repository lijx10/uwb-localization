#include "slam/uwb_localization.hpp"

uavos::UWB_Localization::UWB_Localization(ros::NodeHandle& nh):
    m_nh(nh), SLAM_System(nh){
    // load param
    m_nh.param<double>("slam_fps", m_slam_fps, 0);
    if(m_slam_fps<20){
        m_is_fix_fps = false;
    } else {
        m_is_fix_fps = true;
    }
    m_nh.param<bool>("is_initialize_with_ceres", m_is_initialize_with_ceres, true);

    // create UWB_Mobile
    if( m_nh.getParam(std::string("/uav_id"), m_mobile_id ) ) {
        ROS_INFO("Mobile ID (/uav_id): %d", m_mobile_id);
    } else if( m_nh.getParam(std::string("mobile_id"), m_mobile_id ) ){
        ROS_INFO("Mobile ID (yaml): %d", m_mobile_id);
    } else {
        ROS_ERROR("Mobile ID not found!");
        return;
    }
    createUWBMobile(m_mobile_id);
    ROS_INFO("UWB_Mobile created.");

    // initialize by zero or Ceres
    if(true==m_is_initialize_with_ceres){
        uavos::UWB_Loc_Init initializer(nh, m_p_mobile);
        initializer.initializeMobileByCeres();
    } else {
        m_p_mobile->simpleInitializeEKF();
    }


    // initialize range info callback
    m_range_info_sub = m_nh.subscribe("/time_domain/full_range_info",
                                      10,
                                      &uavos::UWB_Localization::rangeInfoCallback,
                                      this);
    m_print_timer = m_nh.createTimer(ros::Duration(1), &uavos::UWB_Localization::printLocalizationCallback, this);
}

void uavos::UWB_Localization::solveSLAM(const ros::TimerEvent &event){
    if(false==m_p_mobile->getStablizationFlag()){
        return;
    } else {
        if(true==m_is_fix_fps){
            setPositionVelocityWorldCurrNWU(m_position, m_velocity);
        }
    }
}


void uavos::UWB_Localization::rangeInfoCallback(const common_msgs::UWB_FullRangeInfo::ConstPtr &msg){
    //ROS_INFO("full range info received.");
    if(m_p_mobile->filter3DUpdate(*msg) && m_p_mobile->getStablizationFlag()==true){
        // set measurement to uav_os
        m_position.position.x = m_p_mobile->getPosition().point.x;
        m_position.position.y = m_p_mobile->getPosition().point.y;
        m_position.position.z = m_p_mobile->getPosition().point.z;
        m_position.orientation.x = 0;
        m_position.orientation.y = 0;
        m_position.orientation.z = 0;
        m_position.orientation.w = 1;

        m_velocity.linear.x = m_p_mobile->getVelocity().point.x;
        m_velocity.linear.y = m_p_mobile->getVelocity().point.y;
        m_velocity.linear.z = m_p_mobile->getVelocity().point.z;
        m_velocity.angular.x = m_p_mobile->getVelocity().point.x;
        m_velocity.angular.y = m_p_mobile->getVelocity().point.y;
        m_velocity.angular.z = m_p_mobile->getVelocity().point.z;

        if(false==m_is_fix_fps){
            setPositionVelocityWorldCurrNWU(m_position, m_velocity);
        }
    } else {

    }
}

void uavos::UWB_Localization::printLocalizationCallback(const ros::TimerEvent &event){
    if(m_p_mobile->getStablizationFlag()==false){
        ROS_INFO("Not yet steady.");
        return;
    }

    // print UWB anchor stat
    for(std::map<int, boost::shared_ptr<uavos::UWB_Anchor> >::iterator iter=m_anchor_map.begin();
        iter!=m_anchor_map.end();
        ++iter){
        const int anchor_id = iter->first;
        const int total_reading = m_p_mobile->getReadingCount(anchor_id);
        const int successful_reading = m_p_mobile->getSuccessfulReadingCount(anchor_id);
        float success_rate = 0;
        if(total_reading==0){
            success_rate = 0;
        } else {
            success_rate = float(successful_reading)/float(total_reading);
        }

        std::cout<<"anchor "<<anchor_id<<": "<<total_reading<<"   "<<success_rate<<std::endl;
    }

    // clear UWB anchor stat
    m_p_mobile->m_anchor_reading_counter.clear();
    m_p_mobile->m_anchor_successful_reading_counter.clear();


    // print the SLAM result.
    m_p_mobile->printPosition();
    m_p_mobile->printVelocity();
    m_p_mobile->printAccelerationBias();
    
    // print innovation threshold
    std::cout<<"innovation threshold: "<<m_p_mobile->getInnovationThreshold()<<std::endl;
    
    std::cout<<"--------------"<<std::endl;
}

void uavos::UWB_Localization::createUWBMobile(const int mobile_id){
    // get anchor list from parameter server
    std::string param_key("anchor_list");
    if(!m_nh.getParam(param_key, m_anchor_list)){
        ROS_ERROR("Can't find anchor list param.");
        return;
    }

    // get anchor position, build anchor_map
    for(int i=0;i<m_anchor_list.size();++i){
        int anchor_id = m_anchor_list.at(i);

        param_key = std::string("anchor_") + uavos::num2str(anchor_id);
        std::vector<double> position;
        if(m_nh.getParam(param_key, position) && position.size()==3){
            // has valid position
            // set anchor, variance set to constant 1
            boost::shared_ptr<uavos::UWB_Anchor> p_anchor(
                        new uavos::UWB_Anchor( anchor_id,
                                               position.at(0), position.at(1), position.at(2),
                                               1, 1, 1 ) );
            m_anchor_map.insert(std::pair<int, boost::shared_ptr<uavos::UWB_Anchor> >(anchor_id, p_anchor) );
        }
    }

    // create pointer
    m_p_mobile = boost::shared_ptr<uavos::UWB_Mobile>(new uavos::UWB_Mobile(mobile_id, m_anchor_map, m_nh) );
}
