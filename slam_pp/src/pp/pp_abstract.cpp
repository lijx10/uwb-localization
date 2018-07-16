#include "pp/pp_abstract.hpp"

using namespace uavos;

void PathPlanning::uav_state_sub_callback(const common_msgs::NavigationState::ConstPtr &msg){
    m_uav_state_NWU = *msg;
}
void PathPlanning::curr_reference_sub_callback(const common_msgs::Reference::ConstPtr &msg){
    m_curr_reference_NWU = *msg;
}
void PathPlanning::curr_waypoint_sub_callback(const common_msgs::Waypoint::ConstPtr &msg){
    m_curr_waypoint_NWU = *msg;

    // reset path planning status;
    m_path_planning_status.is_reached = false;
    m_path_planning_status.is_target_reachable = true;
}
void PathPlanning::max_dynamics_sub_callback(const common_msgs::MaxDynamics::ConstPtr& msg){
    m_max_dynamics_NWU = *msg;
}
void PathPlanning::laser_scan_sub_callback(const sensor_msgs::LaserScan::ConstPtr &msg){
    *m_p_laser_scan = *msg;

    // transform to eliminate roll and pitch
    common_msgs::NavigationState curr_state = this->getUAVStateNWU();
    tf::Quaternion q;
    tf::quaternionMsgToTF(curr_state.pose.orientation, q);
    tf::Matrix3x3 rotation_matrix(q);

    tf::Transform transform(rotation_matrix, tf::Vector3(0,0,0));
    for(size_t i=0;i<msg->ranges.size();++i){
        if(msg->ranges.at(i) <= msg->range_min+0.001){
            m_p_laser_scan->ranges.at(i) = 0;
        } else if(msg->ranges.at(i) >= msg->range_max-0.001){
            m_p_laser_scan->ranges.at(i) = 32;
        } else {
            float theta = msg->angle_min + i*msg->angle_increment;

            float y = msg->ranges.at(i) * std::sin(theta);
            float x = msg->ranges.at(i) * std::cos(theta);
            tf::Vector3 p_orth = transform*tf::Vector3(x,y,0);

            m_p_laser_scan->ranges.at(i) = std::sqrt(p_orth.x()*p_orth.x()+p_orth.y()*p_orth.y());
        }

    }


}
void PathPlanning::point_cloud_sub_callback(const sensor_msgs::PointCloud2::ConstPtr &msg){
    *m_p_point_cloud = *msg;
}



