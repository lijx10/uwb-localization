#ifndef SLAM_ABSTRACT_HPP
#define SLAM_ABSTRACT_HPP

#include "basic_function.hpp"

#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <visualization_msgs/Marker.h>

// tf
#include <tf/transform_broadcaster.h>
#include "tf_conversions/tf_eigen.h"

// opencv 2
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/eigen.hpp>

// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>


// Eigen
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

// boost thread
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>

namespace uavos{


class SLAM_System{
public:
    explicit SLAM_System(ros::NodeHandle& nh)
    :m_nh(nh)
    {
        m_p_raw_pointcloud = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> >(new pcl::PointCloud<pcl::PointXYZ>());
        m_p_processed_pointcloud = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> >(new pcl::PointCloud<pcl::PointXYZ>());

        // set base_link to the original of map.
        m_tf_map_base = tf::StampedTransform(tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(0,0,0)), ros::Time::now(), "map", "base_link");

        // send the common_msgs::MeasurementPosition/Velocity to the rest of the uav_os
        m_measured_position_pub = nh.advertise<common_msgs::MeasurementPosition>("measurement_position", 10);
        m_measured_velocity_pub = nh.advertise<common_msgs::MeasurementVelocity>("measurement_velocity", 10);
        m_measured_position_velocity_pub = nh.advertise<common_msgs::MeasurementPosVel>("measurement_position_velocity", 10);

        m_pointcloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/cloud_in", 10);

        // parameters
        nh.param<double>("slam_result_print_freq", m_slam_result_print_freq, 1);
        m_slam_result_print_duration = 1.0 / m_slam_result_print_freq;

        nh.param<bool>("is_xy_valid", m_is_xy_valid, true);
        nh.param<bool>("is_z_valid", m_is_z_valid, true);
        nh.param<bool>("is_yaw_valid", m_is_yaw_valid, true);

        nh.param<bool>("is_v_xy_valid", m_is_v_xy_valid, false);
        nh.param<bool>("is_v_z_valid", m_is_v_z_valid, false);

        // visualize
        m_marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    }
    ~SLAM_System(){
    }

private:
    ros::NodeHandle m_nh;

    // all in ros NWU coordinate
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > m_p_raw_pointcloud;  // raw input from sensor driver
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > m_p_processed_pointcloud; // usually down-sampled

    // tf transform that represent camera/base's pose, in NWU coordinate
    tf::StampedTransform m_tf_map_base;
    tf::TransformBroadcaster m_tf_broadcaster;

    common_msgs::MeasurementPosition m_position_world_curr_NWU;
    Eigen::Matrix4d m_G_world_curr_Camera;

    // velocity
    common_msgs::MeasurementVelocity m_velocity_world_curr_NWU;

    // send the common_msgs::MeasurementPosition/Velocity to the rest of the uav_os
    ros::Publisher m_measured_position_pub;
    ros::Publisher m_measured_velocity_pub;
    ros::Publisher m_measured_position_velocity_pub;

    // send point cloud
    ros::Publisher m_pointcloud_pub;

    // debug parameters
    double m_slam_result_print_duration, m_slam_result_print_freq;
    bool m_is_xy_valid;
    bool m_is_z_valid;
    bool m_is_yaw_valid;

    bool m_is_v_xy_valid;
    bool m_is_v_z_valid;


    // store the trajectory
    std::vector<common_msgs::MeasurementPosition> m_trajectory;
    // visualization
    ros::Publisher m_marker_pub;


protected:
    // spinOnce for my own callback queue
    inline void spinOnce(){
        ros::CallbackQueue* queue = dynamic_cast<ros::CallbackQueue*>(m_nh.getCallbackQueue());
        queue->callAvailable();
    }

    Eigen::Matrix4d tfNWU_2_GCamera(const tf::Transform& ros_transform); // camera is East-Down-North
    tf::Transform GCamera_2_tfNWU(const Eigen::Matrix4d& G);
    common_msgs::MeasurementPosition tfNWU_2_PositionNWU(const tf::Transform& ros_transform);
    common_msgs::MeasurementPosition GCamera_2_PositionNWU(const Eigen::Matrix4d& G);

    inline boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > getRawPointcloudPointer() const{
        return m_p_raw_pointcloud;
    }
    inline boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > getProcessedPointcloudPointer() const{
        return m_p_processed_pointcloud;
    }
    inline void setRawPointcloud(const pcl::PointCloud<pcl::PointXYZ>& raw_pc){
        *m_p_raw_pointcloud = raw_pc;
    }
    inline void setProcessedPointcloud(const pcl::PointCloud<pcl::PointXYZ>& processed_pc){
        *m_p_processed_pointcloud = processed_pc;
    }
    inline void setRawPointcloudHeader(){
        m_p_raw_pointcloud->header.frame_id = "base_link";
        pcl_conversions::toPCL(ros::Time::now(), m_p_raw_pointcloud->header.stamp);
    }
    inline void setProcessedPointcloudHeader(){
        m_p_processed_pointcloud->header.frame_id = "base_link";
        pcl_conversions::toPCL(ros::Time::now(), m_p_processed_pointcloud->header.stamp);
    }


    inline void sendRawPointCloud(){
        boost::shared_ptr<sensor_msgs::PointCloud2> pd_ros(new sensor_msgs::PointCloud2());
        pcl::toROSMsg(*m_p_raw_pointcloud, *pd_ros);
        m_pointcloud_pub.publish(pd_ros);
    }
    inline void sendProcessedPointCloud(){
        boost::shared_ptr<sensor_msgs::PointCloud2> pd_ros(new sensor_msgs::PointCloud2());
        pcl::toROSMsg(*m_p_processed_pointcloud, *pd_ros);
        m_pointcloud_pub.publish(pd_ros);
    }


    inline common_msgs::MeasurementPosition getPositionWorldCurrNWU() const{
        return m_position_world_curr_NWU;
    }
    inline Eigen::Matrix4d getGWorldCurrCamera() const{
        return m_G_world_curr_Camera;
    }
    inline tf::StampedTransform getStampedTransform() const{
        return m_tf_map_base;
    }
    inline void setPositionWorldCurrNWU(const common_msgs::MeasurementPosition& pose){
        m_position_world_curr_NWU = pose;
        m_position_world_curr_NWU.header.stamp = ros::Time::now();
        m_position_world_curr_NWU.is_xy_valid = m_is_xy_valid;
        m_position_world_curr_NWU.is_z_valid = m_is_z_valid;
        m_position_world_curr_NWU.is_yaw_valid = m_is_yaw_valid;

        m_measured_position_pub.publish(m_position_world_curr_NWU);
    }
    inline void setPositionVelocityWorldCurrNWU(const geometry_msgs::Pose& pose, const geometry_msgs::Twist& velocity){
        m_position_world_curr_NWU.pose = pose;
        m_position_world_curr_NWU.header.stamp = ros::Time::now();
        m_position_world_curr_NWU.is_xy_valid = m_is_xy_valid;
        m_position_world_curr_NWU.is_z_valid = m_is_z_valid;
        m_position_world_curr_NWU.is_yaw_valid = m_is_yaw_valid;

        m_velocity_world_curr_NWU.velocity = velocity;
        m_velocity_world_curr_NWU.header.stamp = ros::Time::now();
        m_velocity_world_curr_NWU.is_xy_valid = m_is_v_xy_valid;
        m_velocity_world_curr_NWU.is_z_valid = m_is_v_z_valid;
        m_velocity_world_curr_NWU.is_yaw_valid = m_is_yaw_valid; // not in use

        common_msgs::MeasurementPosVel mea_pos_vel;
        mea_pos_vel.position = m_position_world_curr_NWU;
        mea_pos_vel.velocity = m_velocity_world_curr_NWU;
        m_measured_position_velocity_pub.publish(mea_pos_vel);

    }

    inline void setGWorldCurrCamera(const Eigen::Matrix4d& G){
        m_G_world_curr_Camera = G;
    }
    inline void setStampedTransform(const tf::Transform& transform){
        // don't use now()! Think about the synchronization problem.
        ros::Time pointcloud_stamp;
        pcl_conversions::fromPCL(m_p_raw_pointcloud->header.stamp, pointcloud_stamp);

        m_tf_map_base = tf::StampedTransform(transform, pointcloud_stamp, "map", m_p_raw_pointcloud->header.frame_id);
        m_tf_broadcaster.sendTransform(m_tf_map_base);
    }


    inline double getSLAMResultPrintDuration() const{
        return m_slam_result_print_duration;
    }

public:
    void visualizeTrajectoryCallback(const ros::TimerEvent &event);
    virtual void solveSLAM(const ros::TimerEvent& event)=0;


};





class RGBD_SLAM : public SLAM_System{
public:
    explicit RGBD_SLAM(ros::NodeHandle& nh):SLAM_System(nh){
        //ROS_INFO("inside rgbd_slam constructor");

        // check the existence of parameters
        if( !nh.hasParam("rgb_camera_K")
                || !nh.hasParam("depth_camera_K") ){
            ROS_FATAL("RGBD_SLAM: RGB/Depth camera matrix non found in setting.");
        }

        double fx_rgb, fy_rgb, cx_rgb, cy_rgb;
        nh.param<double>("rgb_camera_K/fx", fx_rgb, 525);
        nh.param<double>("rgb_camera_K/fy", fy_rgb, 525);
        nh.param<double>("rgb_camera_K/cx", cx_rgb, 319.5);
        nh.param<double>("rgb_camera_K/cy", cy_rgb, 239.5);

        double fx_depth, fy_depth, cx_depth, cy_depth;
        nh.param<double>("depth_camera_K/fx", fx_depth, 525);
        nh.param<double>("depth_camera_K/fy", fy_depth, 525);
        nh.param<double>("depth_camera_K/cx", cx_depth, 319.5);
        nh.param<double>("depth_camera_K/cy", cy_depth, 239.5);


        m_K_rgb<<fx_rgb,    0,   cx_rgb,
                    0,   fy_rgb, cy_rgb,
                    0,      0,     1;
        m_K_depth<<fx_depth,    0,   cx_depth,
                      0,   fy_depth, cy_depth,
                      0,      0,      1;

        // load tf for depth registration
        nh.param<bool>("is_depth_need_register", m_is_depth_need_register, true);
        nh.param<float>("depth_tx", m_t_x, -0.059);
        nh.param<float>("depth_ty", m_t_y, 0);
        nh.param<float>("depth_tz", m_t_z, 0);

        // depth parameters, pcl::PointXYZ is float
        nh.param<float>("pd_min_depth", m_min_depth, 0.4);
        nh.param<float>("pd_max_depth", m_max_depth, 12);
        nh.param<float>("img_depth_scale", m_depth_scale, 1000);
        m_depth_scale_recip = 1.0f / m_depth_scale;

        // filters
        nh.param<bool>("is_enable_passthrough_filter", m_is_enable_passthrough_filter, false);
        nh.param<bool>("is_enable_downsample_filter", m_is_enable_downsample_filter, true);
        nh.param<float>("pd_downsample_leaf_size", m_pd_downsample_leaf_size, 0.05);
        nh.param<bool>("is_enable_radius_filter", m_is_enable_radius_filter, false);
        nh.param<float>("pd_filter_radius", m_pd_filter_radius, 0.2);
        nh.param<float>("pd_filter_radius_neighbour", m_pd_filter_radius_neighbour, 15);
        nh.param<bool>("is_enable_stat_filter", m_is_enable_stat_filter, true);
        nh.param<float>("pd_filter_mean_k", m_pd_filter_mean_k, 50);
        nh.param<float>("pd_filter_stddev_mul", m_pd_filter_stddev_mul, 1);
    }

protected:
    // they should be in the same resolution
    sensor_msgs::ImageConstPtr m_msgRGB; // CV_8U
    sensor_msgs::ImageConstPtr m_msgDepth; // CV_16U

    cv::Mat m_registered_depth; // CV_32F

    // calibration
    Eigen::Matrix3f m_K_rgb, m_K_depth;
    float m_t_x, m_t_y, m_t_z; // camera coordinate
    bool m_is_depth_need_register;

    // depth parameters, pcl::PointXYZ is float
    float m_min_depth, m_max_depth;
    float m_depth_scale, m_depth_scale_recip;

    // filters
    bool m_is_enable_passthrough_filter;

    bool m_is_enable_downsample_filter;
    float m_pd_downsample_leaf_size;

    bool m_is_enable_radius_filter;
    float m_pd_filter_radius;
    float m_pd_filter_radius_neighbour;

    bool m_is_enable_stat_filter;
    float m_pd_filter_mean_k;
    float m_pd_filter_stddev_mul;

public:
    void saveImages(const ros::TimerEvent& event);

    virtual void registerDepth();
    virtual void generateProcessedPointcloud();
    virtual void processedPointcloudInPlace(pcl::PointCloud<pcl::PointXYZ> &pc);
    virtual void testDepthRegistration();

    virtual void prepareSLAM(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgDepth);
    virtual void solveSLAM(const ros::TimerEvent& event)=0;
};


}

#endif // SLAM_ABSTRACT_HPP
