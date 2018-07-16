#ifndef UWB_LOCALIZATION_H
#define UWB_LOCALIZATION_H

#include "basic_function.hpp"
#include "slam/slam_abstract.hpp"
#include "slam/uwb_node.hpp"
#include "slam/uwb_loc_init.hpp"

namespace uavos {

class UWB_Localization : public SLAM_System
{
public:
    explicit UWB_Localization(ros::NodeHandle& nh);
    ~UWB_Localization(){
    }

private:
    ros::NodeHandle m_nh;
    ros::Subscriber m_range_info_sub;
    ros::Timer m_print_timer;

    double m_slam_fps;
    bool m_is_fix_fps;
    bool m_is_initialize_with_ceres;

    int m_mobile_id;
    boost::shared_ptr<uavos::UWB_Mobile> m_p_mobile;
    geometry_msgs::Pose m_position;
    geometry_msgs::Twist m_velocity;


    std::vector<int> m_anchor_list;
    std::map<int, boost::shared_ptr<uavos::UWB_Anchor> > m_anchor_map;



public:
    void createUWBMobile(const int mobile_id);

    void rangeInfoCallback(const common_msgs::UWB_FullRangeInfo::ConstPtr& msg);
    void printLocalizationCallback(const ros::TimerEvent& event);

    void solveSLAM(const ros::TimerEvent& event);
};



}


#endif // UWB_LOCALIZATION_H
