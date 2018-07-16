#ifndef BASIC_FUNCTION_H_
#define BASIC_FUNCTION_H_

#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <iomanip>
#include <cmath>

#include "ros/ros.h"
#include "std_msgs/UInt32.h"
#include "std_msgs/UInt16.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include <tf/transform_datatypes.h>
#include <ros/console.h>

#include "common_msgs/NavigationState.h"
#include "common_msgs/GPSVelocity.h"
#include "common_msgs/MeasurementPosition.h"
#include "common_msgs/MeasurementVelocity.h"
#include "common_msgs/MeasurementPosVel.h"
#include "common_msgs/PixhawkCMD.h"
#include "common_msgs/PixhawkServo.h"
#include "common_msgs/MaxDynamics.h"
#include "common_msgs/Reference.h"
#include "common_msgs/Mission.h"
#include "common_msgs/MissionStatus.h"
#include "common_msgs/PathPlanningStatus.h"

#include "common_msgs/UWB_FullRangeInfo.h"
#include "common_msgs/UWB_EchoedRangeInfo.h"
#include "common_msgs/UWB_FullNeighborDatabase.h"
#include "common_msgs/UWB_DataInfo.h"
#include "common_msgs/UWB_SendData.h"

#include "nav_msgs/Odometry.h"


// ros callback
#include <ros/callback_queue.h>
#include <ros/callback_queue_interface.h>
#include <ros/spinner.h>


// boost
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

namespace uavos{

// the small delta is to ensure that 180 degree is remain 180, instead of -180.
//#define INPI(angle)		(angle -= floor((angle+M_PI-0.0000000001)/(2*M_PI))*2*M_PI)
//#define INPI(angle)		(angle -= floor((angle+M_PI)/(2*M_PI))*2*M_PI)

void printNavigationPose(const common_msgs::NavigationState& navigation_state, int level_of_detail=0);

template<typename T>
inline std::string num2str(const T src){
    std::stringstream ss;
    ss << src;
    return ss.str();
}

template<typename T>
inline T INPI(const T src){
    T dst = src;

    // the small delta is to ensure that 180 degree is remain 180, instead of -180.
    dst -= floor((dst+M_PI-0.0000000001)/(2*M_PI))*2*M_PI;
    return dst;
}


inline bool is_inImageBound(const int& x, const int& y, const int& width, const int& height){
    if(x<0||x>width-1 || y<0||y>height-1) return false;
    else return true;
}
inline bool is_inImageBound_strict(const float& x, const float& y, const int& width, const int& height){
    if(x<=1||x>=width-2 || y<=1||y>=height-2) return false;
    else return true;
}

inline void setZeroPixhawkCMD(common_msgs::PixhawkCMD& cmd){
    cmd.header.stamp = ros::Time::now();
    for(int i=0;i<3;++i){
        cmd.cmd.at(i)=0;
    }
}
inline void setZeroPixhawkServo(common_msgs::PixhawkServo& servo){
    servo.header.stamp = ros::Time::now();
    for(int i=0;i<4;++i){
        servo.servo.at(i)=0;
    }
}

inline common_msgs::NavigationState getZeroNavigationState(){
    common_msgs::NavigationState state;
    state.header.stamp = ros::Time::now();

    state.pose.position.x = 0;
    state.pose.position.y = 0;
    state.pose.position.z = 0;
    state.pose.orientation.x = 0;
    state.pose.orientation.y = 0;
    state.pose.orientation.z = 0;
    state.pose.orientation.w = 1;

    state.velocity.linear.x=0;
    state.velocity.linear.y=0;
    state.velocity.linear.z=0;
    state.velocity.angular.x=0;
    state.velocity.angular.y=0;
    state.velocity.angular.z=0;

    state.acceleration.linear.x=0;
    state.acceleration.linear.y=0;
    state.acceleration.linear.z=0;
    state.acceleration.angular.x=0;
    state.acceleration.angular.y=0;
    state.acceleration.angular.z=0;

    return state;
}
inline void setZeroNavigationState(common_msgs::NavigationState& state){
    state.header.stamp = ros::Time::now();

    state.pose.position.x = 0;
    state.pose.position.y = 0;
    state.pose.position.z = 0;
    state.pose.orientation.x = 0;
    state.pose.orientation.y = 0;
    state.pose.orientation.z = 0;
    state.pose.orientation.w = 1;

    state.velocity.linear.x=0;
    state.velocity.linear.y=0;
    state.velocity.linear.z=0;
    state.velocity.angular.x=0;
    state.velocity.angular.y=0;
    state.velocity.angular.z=0;

    state.acceleration.linear.x=0;
    state.acceleration.linear.y=0;
    state.acceleration.linear.z=0;
    state.acceleration.angular.x=0;
    state.acceleration.angular.y=0;
    state.acceleration.angular.z=0;
}
inline void setZeroVelocityAccleration4NavigationState(common_msgs::NavigationState& state){
    state.header.stamp = ros::Time::now();

    state.velocity.linear.x=0;
    state.velocity.linear.y=0;
    state.velocity.linear.z=0;
    state.velocity.angular.x=0;
    state.velocity.angular.y=0;
    state.velocity.angular.z=0;

    state.acceleration.linear.x=0;
    state.acceleration.linear.y=0;
    state.acceleration.linear.z=0;
    state.acceleration.angular.x=0;
    state.acceleration.angular.y=0;
    state.acceleration.angular.z=0;
}
inline void setZeroAccleration4NavigationState(common_msgs::NavigationState& state){
    state.header.stamp = ros::Time::now();

    state.acceleration.linear.x=0;
    state.acceleration.linear.y=0;
    state.acceleration.linear.z=0;
    state.acceleration.angular.x=0;
    state.acceleration.angular.y=0;
    state.acceleration.angular.z=0;
}
inline common_msgs::Reference getZeroReference(){
    common_msgs::Reference ref;
    ref.header.stamp = ros::Time::now();
    setZeroNavigationState(ref.navigation);
    setZeroPixhawkCMD(ref.pixhawkCMD);
    setZeroPixhawkServo(ref.pixhawkServo);

    // should be removed later.
    ref.pixhawkCMD.cmd.at(0)= 88;

    return ref;
}
inline void setZeroReference(common_msgs::Reference& ref){
    ref.header.stamp = ros::Time::now();
    setZeroNavigationState(ref.navigation);
    setZeroPixhawkCMD(ref.pixhawkCMD);
    setZeroPixhawkServo(ref.pixhawkServo);

    // should be removed later.
    ref.pixhawkCMD.cmd.at(0)= 88;
}


// ---------- quaternion to rpy
void quaternion2RPY(const geometry_msgs::Quaternion q, double &roll, double &pitch, double &yaw);
void quaternion2RPY(const tf::Quaternion q, double &roll, double &pitch, double &yaw);
void navigation2RPY(const common_msgs::NavigationState state, double &roll, double &pitch, double &yaw);

// vector to NavigationState
common_msgs::NavigationState vector2Navigation(double x, double y, double z, double c,
                                               double vx, double vy, double vz, double vc,
                                               double ax, double ay, double az, double ac);

// vector arithematic
template <typename T, const size_t N>
inline boost::array<T,N> vectorSubtraction(const boost::array<T,N> &a, const boost::array<T,N> &b){
    boost::array<T,N> dst;
    for(int i=0;i<a.size();++i){
        dst.at(i) = ( a.at(i)-b.at(i) );
    }
    return dst;
}
template <typename T, const size_t N>
inline boost::array<T,N> vectorAddition(const boost::array<T,N> &a, const boost::array<T,N> &b){
    boost::array<T,N> dst;
    for(int i=0;i<a.size();++i){
        dst.at(i) = ( a.at(i)+b.at(i) );
    }
    return dst;
}
template <typename T, size_t N>
inline void setZeroVector(boost::array<T,N> &a){
    for(int i=0;i<a.size();++i){
        a.at(i) = 0;
    }
}


template <typename T>
inline std::vector<T> vectorSubtraction(const std::vector<T> &a, const std::vector<T> &b){
    std::vector<T> dst;
    for(int i=0;i<a.size();++i){
        dst.push_back( a.at(i)-b.at(i) );
    }
    return dst;
}
template <typename T>
inline std::vector<T> vectorAddition(const std::vector<T> &a, const std::vector<T> &b){
    std::vector<T> dst;
    for(int i=0;i<a.size();++i){
        dst.push_back( a.at(i)+b.at(i) );
    }
    return dst;
}
template <typename T>
inline void setZeroVector(std::vector<T> &a){
    for(int i=0;i<a.size();++i){
        a.at(i) = 0;
    }
}


common_msgs::NavigationState navigationNWU2NED(common_msgs::NavigationState state_NWU);
common_msgs::NavigationState navigationNED2NWU(common_msgs::NavigationState state_NED);
common_msgs::Reference referenceNWU2NED(common_msgs::Reference ref_NWU);
common_msgs::Reference referenceNED2NWU(common_msgs::Reference ref_NED);


}

#endif //BASIC_FUNCTION_H_
