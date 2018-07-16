#ifndef PP_ABSTRACT_HPP
#define PP_ABSTRACT_HPP

#include "basic_function.hpp"
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>

namespace uavos{

class PathPlanning{
public:
    explicit PathPlanning(ros::NodeHandle& nh){
        m_uav_state_sub = nh.subscribe("/uav_state/navigation_state", 2, &PathPlanning::uav_state_sub_callback, this);
        m_curr_reference_sub = nh.subscribe("/ref_generation/current_reference", 2, &PathPlanning::curr_reference_sub_callback, this);
        m_curr_waypoint_sub = nh.subscribe("/uav_api/waypoint", 2, &PathPlanning::curr_waypoint_sub_callback, this);
        m_max_dynamics_sub = nh.subscribe("/ref_generation/max_dynamics", 2, &PathPlanning::max_dynamics_sub_callback, this);
        m_laser_scan_sub = nh.subscribe("/scan", 2, &PathPlanning::laser_scan_sub_callback, this);
        m_point_cloud_sub = nh.subscribe("/cloud_in", 2, &PathPlanning::point_cloud_sub_callback, this);

        m_planned_waypoint_NWU_pub = nh.advertise<common_msgs::Waypoint>("planned_waypoint", 10);
        m_path_planning_status_pub = nh.advertise<common_msgs::PathPlanningStatus>("path_planning_status", 10);

        // set zeros
        setZeroNavigationState(m_uav_state_NWU);
        setZeroNavigationState(m_curr_waypoint_NWU.navigation_state);
        setZeroNavigationState(m_planned_waypoint_NWU.navigation_state);
        setZeroReference(m_curr_reference_NWU);

        m_p_laser_scan = boost::shared_ptr<sensor_msgs::LaserScan>(new sensor_msgs::LaserScan());
        m_p_point_cloud = boost::shared_ptr<sensor_msgs::PointCloud2>(new sensor_msgs::PointCloud2());
    }
    ~PathPlanning(){

    }

private:
    // laser scan
    boost::shared_ptr<sensor_msgs::LaserScan> m_p_laser_scan;
    ros::Subscriber m_laser_scan_sub;

    // point cloud
    boost::shared_ptr<sensor_msgs::PointCloud2> m_p_point_cloud;
    ros::Subscriber m_point_cloud_sub;

    // NWU coordinate
    common_msgs::NavigationState m_uav_state_NWU;
    common_msgs::Reference m_curr_reference_NWU;
    common_msgs::Waypoint m_curr_waypoint_NWU;
    common_msgs::MaxDynamics m_max_dynamics_NWU;

    // result of pathplanning
    common_msgs::Waypoint m_planned_waypoint_NWU;
    common_msgs::PathPlanningStatus m_path_planning_status;


    // callback members
    ros::Subscriber m_uav_state_sub;
    ros::Subscriber m_curr_reference_sub;
    ros::Subscriber m_curr_waypoint_sub;
    ros::Subscriber m_max_dynamics_sub;

    ros::Publisher m_planned_waypoint_NWU_pub;
    ros::Publisher m_path_planning_status_pub;

private:
    void uav_state_sub_callback(const common_msgs::NavigationState::ConstPtr& msg);
    void curr_reference_sub_callback(const common_msgs::Reference::ConstPtr& msg);
    void curr_waypoint_sub_callback(const common_msgs::Waypoint::ConstPtr& msg);
    void max_dynamics_sub_callback(const common_msgs::MaxDynamics::ConstPtr& msg);
    void laser_scan_sub_callback(const sensor_msgs::LaserScan::ConstPtr& msg);
    void point_cloud_sub_callback(const sensor_msgs::PointCloud2::ConstPtr& msg);

protected:
    inline common_msgs::NavigationState getUAVStateNWU() const{
        return m_uav_state_NWU;
    }
    inline common_msgs::Reference getCurrentReferenceNWU() const{
        return m_curr_reference_NWU;
    }
    inline common_msgs::Waypoint getInputWaypointNWU() const{
        return m_curr_waypoint_NWU;
    }

    inline common_msgs::Waypoint getPlannedWaypointNWU() const{
        return m_planned_waypoint_NWU;
    }
    inline void setPlannedWaypointNWU(const common_msgs::Waypoint& planned_waypoint_NWU){
        // send the result to ref_generation at the same time.
        m_planned_waypoint_NWU = planned_waypoint_NWU;
        m_planned_waypoint_NWU.m_id = m_curr_waypoint_NWU.m_id;
        m_planned_waypoint_NWU_pub.publish(m_planned_waypoint_NWU);

        //std::cout<<"m_id: "<< m_curr_waypoint_NWU.m_id<<std::endl;
    }

    inline common_msgs::PathPlanningStatus getPathPlanningStatus(){
        return m_path_planning_status;
    }
    inline void setPathPlanningStatus(bool is_reached){
        m_path_planning_status.header.stamp = ros::Time::now();
        m_path_planning_status.is_reached = is_reached;

        m_path_planning_status.waypoint = m_curr_waypoint_NWU;

        m_path_planning_status_pub.publish(m_path_planning_status);
    }
    inline void setPathPlanningStatusNWU(bool is_reached, bool is_target_reachable, double alt_x, double alt_y, double alt_z, double alt_c){
        m_path_planning_status.header.stamp = ros::Time::now();
        m_path_planning_status.is_reached = is_reached;
        m_path_planning_status.is_target_reachable = is_target_reachable;

        m_path_planning_status.waypoint = m_curr_waypoint_NWU;

        m_path_planning_status.alternative_target_position_NWU.position.x = alt_x;
        m_path_planning_status.alternative_target_position_NWU.position.y = alt_y;
        m_path_planning_status.alternative_target_position_NWU.position.z = alt_z;
        m_path_planning_status.alternative_target_position_NWU.orientation = tf::createQuaternionMsgFromYaw(alt_c);

        m_path_planning_status_pub.publish(m_path_planning_status);
    }


    inline boost::shared_ptr<sensor_msgs::LaserScan> getLaserScanPointer() const{
        return m_p_laser_scan;
    }
    inline boost::shared_ptr<sensor_msgs::PointCloud2> getPointCloudPointer() const{
        return m_p_point_cloud;
    }

    inline common_msgs::MaxDynamics getMaxDynamicsNWU(){
    	return m_max_dynamics_NWU;
    }


};

}

#endif // PP_ABSTRACT_HPP
