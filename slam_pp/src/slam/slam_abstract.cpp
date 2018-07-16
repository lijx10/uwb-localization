#include "slam/slam_abstract.hpp"


using namespace uavos;


Eigen::Matrix4d SLAM_System::tfNWU_2_GCamera(const tf::Transform& ros_transform){
    Eigen::Matrix3d R_camera;
    Eigen::Vector3d T_camera;

    // convert R
    tf::Quaternion tf_rotation_NWU = ros_transform.getRotation();
    tf::Quaternion tf_rotation_camera = tf::Quaternion(-tf_rotation_NWU.getY(),
                                                -tf_rotation_NWU.getZ(),
                                                tf_rotation_NWU.getX(),
                                                tf_rotation_NWU.getW()); // (x,y,z,w)
    tf::matrixTFToEigen(tf::Matrix3x3(tf_rotation_camera),
                        R_camera);

    // convert T
    tf::Vector3 tf_translation_NWU = ros_transform.getOrigin();
    T_camera(0) = -tf_translation_NWU.getY();
    T_camera(1) = -tf_translation_NWU.getZ();
    T_camera(2) = tf_translation_NWU.getX();

    // assemble
    Eigen::MatrixXd G_camera_inhomo(3,4);
    G_camera_inhomo<<R_camera, T_camera;
    Eigen::MatrixXd tmp(1,4);
    tmp<<0,0,0,1;
    Eigen::Matrix4d G_camera;
    G_camera<<G_camera_inhomo, tmp;

    return G_camera;
}

tf::Transform SLAM_System::GCamera_2_tfNWU(const Eigen::Matrix4d& G){
    // convert rotation
    tf::Matrix3x3 basis_camera;
    tf::matrixEigenToTF(G.topLeftCorner(3,3), basis_camera);
    tf::Quaternion rotation_camera;
    basis_camera.getRotation(rotation_camera);
    tf::Quaternion rotation_NWU(rotation_camera.getZ(),
                                -rotation_camera.getX(),
                                -rotation_camera.getY(),
                                rotation_camera.getW());


    // convert translation
    tf::Vector3 translation_camera(G(0,3), G(1,3), G(2,3));
    tf::Vector3 translation_NWU(translation_camera.getZ(),
                                -translation_camera.getX(),
                                -translation_camera.getY());

    // assemble
    tf::Transform tf_NWU(rotation_NWU, translation_NWU);

    return tf_NWU;
}



common_msgs::MeasurementPosition SLAM_System::tfNWU_2_PositionNWU(const tf::Transform &ros_transform){

    // get x,y,z
    tf::Vector3 tf_translation_NWU = ros_transform.getOrigin();

    // get yaw
    tf::Quaternion tf_rotation_NWU = ros_transform.getRotation();
    geometry_msgs::Quaternion q_NWU;
    tf::quaternionTFToMsg(tf_rotation_NWU, q_NWU);


    common_msgs::MeasurementPosition position_NWU;
    position_NWU.pose.position.x = tf_translation_NWU.getX();
    position_NWU.pose.position.y = tf_translation_NWU.getY();
    position_NWU.pose.position.z = tf_translation_NWU.getZ();
    position_NWU.pose.orientation = q_NWU;

    return position_NWU;
}


common_msgs::MeasurementPosition SLAM_System::GCamera_2_PositionNWU(const Eigen::Matrix4d &G){

    common_msgs::MeasurementPosition position_NWU;

    // convert rotation
    tf::Matrix3x3 basis_camera;
    tf::matrixEigenToTF(G.topLeftCorner(3,3), basis_camera);
    double roll_camera, pitch_camera, yaw_camera;
    basis_camera.getRPY(roll_camera,pitch_camera,yaw_camera);

    geometry_msgs::Quaternion q_NWU =  tf::createQuaternionMsgFromRollPitchYaw(yaw_camera, -roll_camera, -pitch_camera);
    position_NWU.pose.orientation = q_NWU;


    // convert translation
    position_NWU.pose.position.x = G(2,3);
    position_NWU.pose.position.y = -G(0,3);
    position_NWU.pose.position.z = -G(1,3);

    return position_NWU;

}

// visualize
void uavos::SLAM_System::visualizeTrajectoryCallback(const ros::TimerEvent &event){

    // avoid adding visualization marker when UAV doesn't move.
    if(m_trajectory.size()==0){
        m_trajectory.push_back(m_position_world_curr_NWU);
    }
    common_msgs::MeasurementPosition last_trajectory = m_trajectory.at(m_trajectory.size()-1);
    if(std::fabs(m_position_world_curr_NWU.pose.position.x-last_trajectory.pose.position.x)>0.01 &&
       std::fabs(m_position_world_curr_NWU.pose.position.y-last_trajectory.pose.position.y)>0.01 &&
       std::fabs(m_position_world_curr_NWU.pose.position.z-last_trajectory.pose.position.z)>0.01){
        m_trajectory.push_back(m_position_world_curr_NWU);
    }


    visualization_msgs::Marker points;
    points.header.frame_id = "/map";
    points.header.stamp = ros::Time::now();
    points.type = visualization_msgs::Marker::POINTS;
    points.action = visualization_msgs::Marker::ADD;
    points.id = 0;
    points.lifetime = ros::Duration();

    points.scale.x = 0.1;
    points.scale.y = 0.1;
    points.scale.z = 0.1;
    points.color.g = 1.0;
    points.color.a = 1.0;

    for(int i=0;i<m_trajectory.size();++i){
        geometry_msgs::Point point = m_trajectory.at(i).pose.position;
        points.points.push_back(point);
    }

    m_marker_pub.publish(points);

//    ROS_INFO("marker point count: %d", int(points.points.size()));
}

// -------------------------------------------------------------------------
// -------------------------------------------------------------------------



void RGBD_SLAM::registerDepth(){
    // the transformation is /camera/depth/camera_info
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > p_raw_pointcloud = getRawPointcloudPointer();

    if(true==m_is_depth_need_register){

        float fx_depth = m_K_depth(0,0);
        float fy_depth = m_K_depth(1,1);
        float cx_depth = m_K_depth(0,2);
        float cy_depth = m_K_depth(1,2);

        float fx_rgb = m_K_rgb(0,0);
        float fy_rgb = m_K_rgb(1,1);
        float cx_rgb = m_K_rgb(0,2);
        float cy_rgb = m_K_rgb(1,2);

        cv_bridge::CvImageConstPtr image_RGB = cv_bridge::toCvShare(m_msgRGB);
        cv_bridge::CvImageConstPtr image_depth = cv_bridge::toCvShare(m_msgDepth);

        p_raw_pointcloud->clear();
        m_registered_depth = cv::Mat::zeros(image_RGB->image.rows, image_RGB->image.cols,CV_32F);

        size_t counter = 0;
        for(int i=0;i<image_depth->image.rows;++i){
            const ushort* row_depth = image_depth->image.ptr<ushort>(i);
            for(int j=0;j<image_depth->image.cols;++j){
                const ushort raw_depth = row_depth[j];
                if(raw_depth>m_min_depth*m_depth_scale && raw_depth<m_max_depth*m_depth_scale){
                    // calculate depth
                    float depth = raw_depth * m_depth_scale_recip;

                    // camera coordinate
                    float X = (j*depth-cx_depth*depth)/fx_depth + m_t_x;
                    float Y = (i*depth-cy_depth*depth)/fy_depth + m_t_y;
                    float Z = depth + m_t_z;

                    float x = fx_rgb*X/Z + cx_rgb;
                    float y = fy_rgb*Y/Z + cy_rgb;

                    // save to raw pointcloud vector, convert to NWU coordinate
                    p_raw_pointcloud->points.push_back(pcl::PointXYZ(Z,-X,-Y));
                    counter+=1;

                    // save to depth image
                    if(is_inImageBound(x,y,image_RGB->image.cols,image_RGB->image.rows)){
                        m_registered_depth.at<float>(y,x) = Z;
                    }
                }
            }
        }

        p_raw_pointcloud->width = counter;
        p_raw_pointcloud->height = 1;
        setRawPointcloudHeader();

    } else {
        float fx_depth = m_K_depth(0,0);
        float fy_depth = m_K_depth(1,1);
        float cx_depth = m_K_depth(0,2);
        float cy_depth = m_K_depth(1,2);

        cv_bridge::CvImageConstPtr image_depth = cv_bridge::toCvShare(m_msgDepth);

        p_raw_pointcloud->clear();
        m_registered_depth = image_depth->image;

        size_t counter = 0;
        for(int i=0;i<image_depth->image.rows;++i){
            const ushort* row_depth = image_depth->image.ptr<ushort>(i);
            for(int j=0;j<image_depth->image.cols;++j){
                const ushort raw_depth = row_depth[j];
                if(raw_depth>m_min_depth*m_depth_scale && raw_depth<m_max_depth*m_depth_scale){
                    // calculate depth
                    float depth = raw_depth * m_depth_scale_recip;

                    // camera coordinate
                    float X = (j*depth-cx_depth*depth)/fx_depth;
                    float Y = (i*depth-cy_depth*depth)/fy_depth;
                    float Z = depth;

                    // save to raw pointcloud vector, convert to NWU coordinate
                    p_raw_pointcloud->points.push_back(pcl::PointXYZ(Z,-X,-Y));
                    counter+=1;

                }
            }
        }


        p_raw_pointcloud->width = counter;
        p_raw_pointcloud->height = 1;
        setRawPointcloudHeader();
    }

}

void RGBD_SLAM::generateProcessedPointcloud(){
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > p_processed_pointcloud = getProcessedPointcloudPointer();

    *p_processed_pointcloud = *getRawPointcloudPointer();
    processedPointcloudInPlace(*p_processed_pointcloud);

    setProcessedPointcloudHeader();

}

void RGBD_SLAM::processedPointcloudInPlace(pcl::PointCloud<pcl::PointXYZ> &pc){
    if(m_is_enable_passthrough_filter){
        // set up filter for height range, also removes NANs:
        pcl::PassThrough<pcl::PointXYZ> pass_x;
        pass_x.setFilterFieldName("x");
        pass_x.setFilterLimits(-100, 100);
        pcl::PassThrough<pcl::PointXYZ> pass_y;
        pass_y.setFilterFieldName("y");
        pass_y.setFilterLimits(-100, 100);
        pcl::PassThrough<pcl::PointXYZ> pass_z;
        pass_z.setFilterFieldName("z");
        pass_z.setFilterLimits(-100, 100);

        pass_x.setInputCloud(pc.makeShared());
        pass_x.filter(pc);
        pass_y.setInputCloud(pc.makeShared());
        pass_y.filter(pc);
        pass_z.setInputCloud(pc.makeShared());
        pass_z.filter(pc);
    }

    if(m_is_enable_downsample_filter){
        pcl::VoxelGrid<pcl::PointXYZ> downsample;
        downsample.setInputCloud (pc.makeShared());
        downsample.setLeafSize (m_pd_downsample_leaf_size, m_pd_downsample_leaf_size, m_pd_downsample_leaf_size);
        downsample.filter(pc);
    }

    if(m_is_enable_radius_filter){
        pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
        outrem.setInputCloud(pc.makeShared());
        outrem.setRadiusSearch(m_pd_filter_radius);
        outrem.setMinNeighborsInRadius(m_pd_filter_radius_neighbour);
        outrem.filter (pc);
    }

    if(m_is_enable_stat_filter){
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> stat_filter;
        stat_filter.setInputCloud (pc.makeShared());
        stat_filter.setMeanK (m_pd_filter_mean_k);
        stat_filter.setStddevMulThresh (m_pd_filter_stddev_mul);
        stat_filter.filter (pc);
    }
}


void RGBD_SLAM::testDepthRegistration(){
    cv_bridge::CvImageConstPtr cv_ptrRGB = cv_bridge::toCvShare(boost::const_pointer_cast<sensor_msgs::Image>(m_msgRGB));
    cv_bridge::CvImageConstPtr cv_ptrDepth = cv_bridge::toCvShare(boost::const_pointer_cast<sensor_msgs::Image>(m_msgDepth));

    // debug for depth registration.
    cv::Mat rgb_d_disp;
    cv_ptrRGB->image.copyTo(rgb_d_disp);
    for(int i=0;i<rgb_d_disp.rows;++i){
        for(int j=0;j<rgb_d_disp.cols;++j){
            if(m_registered_depth.at<float>(i,j)>=0.3 && m_registered_depth.at<float>(i,j)<=20){
                rgb_d_disp.at<uchar>(i,j) = 255;
            }
        }
    }


    cv::Mat depth_ori;
    cv_ptrDepth->image.convertTo(depth_ori, CV_32F);
    depth_ori = 0.05*depth_ori;
    depth_ori.convertTo(depth_ori, CV_8U);

    cv::imshow("rgb", cv_ptrRGB->image);
    cv::imshow("rgb-depth", rgb_d_disp);
    cv::imshow("original depth", depth_ori);

    cvWaitKey(0);
}


void RGBD_SLAM::prepareSLAM(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgDepth){
    // update the stored images
    m_msgRGB = msgRGB;
    m_msgDepth = msgDepth;

}

void RGBD_SLAM::saveImages(const ros::TimerEvent& event){
    if(false==bool(m_msgRGB) || false==bool(m_msgDepth)){
        ROS_WARN("In ORB_SLAM_RGBD - solveSLAM, RGB/Depth pointer is NULL.");
        return;
    }

    // save images
    cv_bridge::CvImageConstPtr cv_ptrRGB = cv_bridge::toCvShare(boost::const_pointer_cast<sensor_msgs::Image>(m_msgRGB));
    cv::imwrite(std::string("/home/jiaxin/Downloads/images/rgb/")+boost::lexical_cast<std::string>(m_msgRGB->header.seq)+std::string(".png"), cv_ptrRGB->image);

    cv_bridge::CvImageConstPtr cv_ptrDepth = cv_bridge::toCvShare(boost::const_pointer_cast<sensor_msgs::Image>(m_msgDepth));
    cv::imwrite(std::string("/home/jiaxin/Downloads/images/depth/")+boost::lexical_cast<std::string>(m_msgDepth->header.seq)+std::string(".png"), cv_ptrDepth->image);
}
