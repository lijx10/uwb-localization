#ifndef UWB_NODE_H
#define UWB_NODE_H

#include "ros/ros.h"
#include "geometry_msgs/PointStamped.h"
#include "common_msgs/UWB_FullRangeInfo.h"
#include "common_msgs/UWB_FullNeighborDatabase.h"
#include "common_msgs/UWB_DataInfo.h"
#include "common_msgs/UWB_SendData.h"
#include "common_msgs/UWB_EchoedRangeInfo.h"
#include "common_msgs/NavigationState.h"

#include <Eigen/Core>
#include <Eigen/Dense>

#include <boost/shared_ptr.hpp>
#include <map>

namespace uavos{

typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 6, 6> Matrix6d;


// -----------------------------
struct comp{
    template <typename T>
    inline bool operator()(const T& a, const T& b) const{
        return (a<b);
    }
};



template <typename T>
double robustAverage(std::vector<T> array){
    // the paramter is copied, so that the original data will not be affected.
    const int cut_length = 2;

    if(array.size()<=2*cut_length){
        return 0;
    }

    double sum = 0;
    std::sort(array.begin(), array.end(), comp());
    for(int i=cut_length;i<array.size()-cut_length;++i){
        sum+=array.at(i);
    }

    return sum / (array.size()-2*cut_length);

}

template <typename T>
bool medianAndVariance(std::vector<T> array, double& median, double& variance){
    if(array.size()==0){
        return false;
    }

    // get median
    std::sort(array.begin(), array.end(), comp());
    median = array.at(std::floor(array.size()/2));

    // get mean
    double sum = 0;
    for(int i=0;i<array.size();++i){
        sum+=array.at(i);
    }
    double mean = sum / array.size();


    // get variance
    double variance_sum = 0;
    for(int i=0;i<array.size();++i){
        double res = array.at(i)-mean;
        variance_sum += res*res;
    }
    variance = variance_sum / array.size();

    return true;
}

// -----------------------------


class UWB_Node{
public:
    explicit UWB_Node(const int node_id);

protected:
    int m_node_id;

    geometry_msgs::PointStamped m_position;
    geometry_msgs::PointStamped m_velocity;



public:
    inline int getNodeId() const{
        return m_node_id;
    }
    inline geometry_msgs::PointStamped getPosition() const{
        return m_position;
    }
    inline geometry_msgs::PointStamped* getPositionPtr(){
        return &m_position;
    }
    inline geometry_msgs::PointStamped getVelocity() const{
        return m_velocity;
    }


    inline void setPosition(const double x, const double y, const double z){
        m_position.header.stamp = ros::Time::now();

        m_position.point.x = x;
        m_position.point.y = y;
        m_position.point.z = z;
    }
    inline void setVelocity(const double vx, const double vy, const double vz){
        m_velocity.header.stamp = ros::Time::now();

        m_velocity.point.x = vx;
        m_velocity.point.y = vy;
        m_velocity.point.z = vz;
    }



};


class UWB_Anchor : public UWB_Node{
public:
    explicit UWB_Anchor(const int node_id,
                        const double x, const double y, const double z,
                        const double var_x, const double var_y, const double var_z);

public:
    std::map<int, std::vector<double> > m_range_map;

    void insertRangeInfo(const common_msgs::UWB_EchoedRangeInfo& msg);
    void insertRangeInfo(const common_msgs::UWB_FullRangeInfo& msg);
    double getAverageRangeToNode(const int anchor_id);
    bool getMedianAndVarianceToNode(const int anchor_id, double& median, double& variance);

    inline void printPosition(){
        std::cout<<m_node_id<<": "<<m_position.point.x<<", "<<m_position.point.y<<", "<<m_position.point.z<<std::endl;
    }

};


class UWB_Mobile : public UWB_Node{
public:
    explicit UWB_Mobile(const int node_id,
                        const std::map<int, boost::shared_ptr<UWB_Anchor> > anchor_map,
                        ros::NodeHandle& nh);

private:
    // stores last correct/accepted measurements
    std::map<int, common_msgs::UWB_FullRangeInfo> m_ranges_to_anchors;
    ros::Time m_last_range_time;
    bool m_is_last_filter_succeed;

    // provided by parameter server
    std::map<int, boost::shared_ptr<UWB_Anchor> > m_anchor_map;

    double m_kalman_sigma_a;
    double m_snr_threshold;
    double m_innovation_threshold;
    double m_tao_acc_sqrt;
    double m_tao_bias_sqrt;

    // check whether the filter is properly initialized and ready to use
    bool m_is_stablized;
    std::vector<geometry_msgs::PointStamped> m_init_inspector_position_array;

    // variables for 6 state filter
    Matrix6d m_six_state_covariance;

    // variables for 9 state filter, including acceleration bias
    geometry_msgs::PointStamped m_acc_bias;
    Eigen::MatrixXd m_nine_state_covariance;
    common_msgs::NavigationState m_uav_state;
    ros::Subscriber m_uav_state_sub;


    // parameters
    ros::NodeHandle m_nh;
    std::string m_filter_type;
    
    double m_initialization_innovation_threshold;
    double m_normal_innovation_threshold;
    double m_innovation_threshold_inc_on_succ;
    double m_innovation_threshold_inc_on_fail;

    double m_z_damping_factor;
    double m_Q_scale;
    double m_R_scale;

    // UKF parameters
    double m_ukf_alpha, m_ukf_beta, m_ukf_kappa;

    // debug
    int m_counter;


public:
    // reading statistics, to get effective frequency
    std::map<int, int> m_anchor_reading_counter;
    std::map<int, int> m_anchor_successful_reading_counter;

public:
    inline void setAccelerationBias(const double ax_bias, const double ay_bias, const double az_bias){
        m_acc_bias.header.stamp = ros::Time::now();
        m_acc_bias.point.x = ax_bias;
        m_acc_bias.point.y = ay_bias;
        m_acc_bias.point.z = az_bias;
    }
    inline Eigen::MatrixXd getNineStateCovariance() const{
        return m_nine_state_covariance;
    }
    inline void setNineStateCovariance(const Eigen::MatrixXd P){
        m_nine_state_covariance = P;
    }
    inline Matrix6d getSixStateCovariance() const{
        return m_six_state_covariance;
    }
    inline void setSixStateCovariance(const Matrix6d state_covariance){
        m_six_state_covariance = state_covariance;
    }

    inline common_msgs::UWB_FullRangeInfo getRangeInfoToAnchor(const int anchor_id) const{
        return m_ranges_to_anchors.find(anchor_id)->second;
    }
    inline double getRangeMeterToAnchor(const int anchor_id) const{
        return double(m_ranges_to_anchors.find(anchor_id)->second.precisionRangeMm) / 1000.0;
    }

    inline void setRangeInfoToAnchor(const common_msgs::UWB_FullRangeInfo& range_info){
        m_ranges_to_anchors[range_info.responderId] = range_info;
    }

    inline bool getAnchorPosition(const int anchor_id, geometry_msgs::PointStamped& anchor_position) const {
        if(m_anchor_map.find(anchor_id)!=m_anchor_map.end()){
            anchor_position = m_anchor_map.find(anchor_id)->second->getPosition();
            return true;
        } else {
            return false;
        }
    }
    inline int getAnchorNumber() const{
        return m_anchor_map.size();
    }


    // stablization of filter
    bool checkStablization();
    inline bool getStablizationFlag() const{
        return m_is_stablized;
    }
    inline double getInnovationThreshold() const{
        return m_innovation_threshold;
    }


    // statistics for monitoring the ranging status
    inline void readingCounterInc(const int anchor_id){
        if(m_anchor_reading_counter.find(anchor_id)==m_anchor_reading_counter.end()){
            m_anchor_reading_counter[anchor_id] = 1;
        } else {
            m_anchor_reading_counter[anchor_id] += 1;
        }
    }
    inline void successfulReadingCounterInc(const int anchor_id){
        if(m_anchor_successful_reading_counter.find(anchor_id)==m_anchor_successful_reading_counter.end()){
            m_anchor_successful_reading_counter[anchor_id] = 1;
        } else {
            m_anchor_successful_reading_counter[anchor_id] += 1;
        }
    }
    inline int getReadingCount(const int anchor_id){
        if(m_anchor_reading_counter.find(anchor_id)==m_anchor_reading_counter.end()){
            return 0;
        } else {
            return m_anchor_reading_counter.at(anchor_id);
        }
    }
    inline int getSuccessfulReadingCount(const int anchor_id){
        if(m_anchor_successful_reading_counter.find(anchor_id)==m_anchor_successful_reading_counter.end()){
            return 0;
        } else {
            return m_anchor_successful_reading_counter.at(anchor_id);
        }
    }



    // EKF for position, velocity estimation
    void navigationStateCallback(const common_msgs::NavigationState::ConstPtr& msg){
        m_uav_state = *msg;
        
        // threshold the acceleration to prevent filter divergence
        double acceleration_linear_threshold_xy = 3.0;
        double acceleration_linear_threshold_z = 2.0;
        if(m_uav_state.acceleration.linear.x>=acceleration_linear_threshold_xy){
            m_uav_state.acceleration.linear.x = acceleration_linear_threshold_xy;
        } else if(m_uav_state.acceleration.linear.y<=-acceleration_linear_threshold_xy){
            m_uav_state.acceleration.linear.y = -acceleration_linear_threshold_xy;
        }
        if(m_uav_state.acceleration.linear.y>=acceleration_linear_threshold_xy){
            m_uav_state.acceleration.linear.y = acceleration_linear_threshold_xy;
        } else if(m_uav_state.acceleration.linear.y<=-acceleration_linear_threshold_xy){
            m_uav_state.acceleration.linear.y = -acceleration_linear_threshold_xy;
        }
        if(m_uav_state.acceleration.linear.z>=acceleration_linear_threshold_z){
            m_uav_state.acceleration.linear.z = acceleration_linear_threshold_z;
        } else if(m_uav_state.acceleration.linear.z<=-acceleration_linear_threshold_z){
            m_uav_state.acceleration.linear.z = -acceleration_linear_threshold_z;
        }
        
    }
    
    void simpleInitializeEKF();
    void fullInitializeEKF(const common_msgs::UWB_FullNeighborDatabase& ndb, const geometry_msgs::PointStamped& position);
    
    // dynamic innovation threshold, to avoid filter divergence
    void getInnovation();
    
    bool filter3DUpdate(const common_msgs::UWB_FullRangeInfo& range_info);
    bool kalmanFilter3DUpdate(const common_msgs::UWB_FullRangeInfo& range_info);
    bool unscentedKalmanFilter3DUpdate(const common_msgs::UWB_FullRangeInfo& range_info);

    bool ekfWithAcc(const common_msgs::UWB_FullRangeInfo& range_info);
    bool ukfWithAcc(const common_msgs::UWB_FullRangeInfo& range_info);



    // print
    inline void printPosition(){
        std::cout<<m_node_id<<":"<<std::endl;

        if(std::string("EKF")==m_filter_type || std::string("UKF")==m_filter_type){
            std::cout<<"pose: "<<m_position.point.x<<", "<<m_position.point.y<<", "<<m_position.point.z
                    <<"   cov: "<<m_six_state_covariance(0,0)<<", "<<m_six_state_covariance(2,2)<<", "<<m_six_state_covariance(4,4)
                    <<std::endl;
        } else {
            std::cout<<"pose: "<<m_position.point.x<<", "<<m_position.point.y<<", "<<m_position.point.z
                    <<"   cov: "<<m_nine_state_covariance(0,0)<<", "<<m_nine_state_covariance(3,3)<<", "<<m_nine_state_covariance(6,6)
                    <<std::endl;
        }

    }
    inline void printVelocity(){
        if(std::string("EKF")==m_filter_type || std::string("UKF")==m_filter_type){
            std::cout<<"velo: "<<m_velocity.point.x<<", "<<m_velocity.point.y<<", "<<m_velocity.point.z
                    <<"   cov: "<<m_six_state_covariance(1,1)<<", "<<m_six_state_covariance(3,3)<<", "<<m_six_state_covariance(5,5)
                    <<std::endl;
        } else {
            std::cout<<"velo: "<<m_velocity.point.x<<", "<<m_velocity.point.y<<", "<<m_velocity.point.z
                    <<"   cov: "<<m_nine_state_covariance(1,1)<<", "<<m_nine_state_covariance(4,4)<<", "<<m_nine_state_covariance(7,7)
                    <<std::endl;
        }
    }
    inline void printAccelerationBias(){
        if(std::string("EKF")==m_filter_type || std::string("UKF")==m_filter_type){

        } else {
            std::cout<<"acc bias: "<<m_acc_bias.point.x<<", "<<m_acc_bias.point.y<<", "<<m_acc_bias.point.z
                     <<"   cov: "<<m_nine_state_covariance(2,2)<<", "<<m_nine_state_covariance(5,5)<<", "<<m_nine_state_covariance(8,8)
                     <<std::endl;
        }
    }
};


}

#endif // UWB_NODE_H
