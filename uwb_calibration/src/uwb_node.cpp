#include "uwb_node.hpp"


//------------------------------------------
//------------------------------------------

uavos::UWB_Node::UWB_Node(const int node_id):m_node_id(node_id){

}


// ---------------------------------------------
uavos::UWB_Anchor::UWB_Anchor(const int node_id,
                              const double x, const double y, const double z,
                              const double var_x, const double var_y, const double var_z)
    :UWB_Node(node_id)
{
    setPosition(x,y,z);
}

void uavos::UWB_Anchor::insertRangeInfo(const common_msgs::UWB_EchoedRangeInfo &msg){
    // only insert range info that is requested by myself.

    int requester_id = msg.requesterId;
    int responder_id = msg.responderId;
    double range = double(msg.precisionRangeMm) / 1000.0;

    if(m_node_id==requester_id){
        // if haven't seen the responder yet, create it
        if(m_range_map.find(responder_id)==m_range_map.end()){
            m_range_map.insert(std::pair<int, std::vector<double> >(responder_id, std::vector<double>()));
            // add the range measurement into the vector
            m_range_map.find(responder_id)->second.push_back(range);
        } else {
            // add the range measurement into the vector
            m_range_map.find(responder_id)->second.push_back(range);
        }
    }
}

void uavos::UWB_Anchor::insertRangeInfo(const common_msgs::UWB_FullRangeInfo& msg){
    // only insert range info that is requested by myself.

    int requester_id = msg.nodeId;
    int responder_id = msg.responderId;
    double range = double(msg.precisionRangeMm) / 1000.0;

    if(m_node_id==requester_id){
        // if haven't seen the responder yet, create it
        if(m_range_map.find(responder_id)==m_range_map.end()){
            m_range_map.insert(std::pair<int, std::vector<double> >(responder_id, std::vector<double>()));
            // add the range measurement into the vector
            m_range_map.find(responder_id)->second.push_back(range);
        } else {
            // add the range measurement into the vector
            m_range_map.find(responder_id)->second.push_back(range);
        }
    }
}

double uavos::UWB_Anchor::getAverageRangeToNode(const int anchor_id){
    // check whether the anchor_id has been seen
    std::map<int, std::vector<double> >::iterator it = m_range_map.find(anchor_id);
    if(it==m_range_map.end()){
        return 0;
    } else {
        // check the size of the vector
        if(it->second.size()==0){
            return 0;
        } else {
            return uavos::robustAverage<double>(it->second);
        }
    }
}

bool uavos::UWB_Anchor::getMedianAndVarianceToNode(const int anchor_id, double &median, double &variance){
    std::map<int, std::vector<double> >::iterator it = m_range_map.find(anchor_id);
    if(it==m_range_map.end()){
        return false;
    } else {
        if(it->second.size()==0){
            return false;
        } else {
            uavos::medianAndVariance<double>(it->second, median, variance);
            return true;
        }
    }
}



// ----------------------------------------------
uavos::UWB_Mobile::UWB_Mobile(const int node_id, const std::map<int, boost::shared_ptr<UWB_Anchor> > anchor_map)
    :UWB_Node(node_id),
      m_anchor_map(anchor_map)
{
    m_kalman_sigma_a = 0.125;
//    m_kalman_sigma_a = 1.25;

    m_snr_threshold = 15;
    m_innovation_threshold = 2;

    m_is_initialized = false;
}

void uavos::UWB_Mobile::initializeEKF(){
    setPosition(0,0,0);
    setVelocity(0,0,0);
    setStateCovariance(Eigen::MatrixXd::Identity(6,6));

    common_msgs::UWB_FullRangeInfo zero_range_info;
    for(std::map<int, boost::shared_ptr<uavos::UWB_Anchor> >::iterator iter=m_anchor_map.begin();
        iter!=m_anchor_map.end();
        ++iter){
        int anchor_id = iter->second->getNodeId();

        zero_range_info.responderId = anchor_id;
        zero_range_info.header.stamp = ros::Time::now();
        setRangeInfoToAnchor(zero_range_info);

        m_last_range_time = ros::Time::now();
    }
}


bool uavos::UWB_Mobile::checkInitialization(){
    if(m_is_initialized==true){
        return true;
    }

    m_init_inspector_position_array.push_back(this->getPosition());

    // get time nearby positions
    std::vector<geometry_msgs::PointStamped> valid_position_array;
    for(int i=m_init_inspector_position_array.size()-1;i>=0;--i){
        geometry_msgs::PointStamped position = m_init_inspector_position_array.at(i);
        if(ros::Time::now().toSec() - position.header.stamp.toSec() < 5){
            valid_position_array.push_back(position);
        } else {
            break;
        }
    }

    if(valid_position_array.size()<10){
        return false;
    } else {
        // get x,y,z mean
        float x_sum=0;
        float y_sum=0;
        float z_sum=0;
        for(int i=0;i<valid_position_array.size();++i){
            geometry_msgs::PointStamped position = valid_position_array.at(i);
            x_sum += position.point.x;
            y_sum += position.point.y;
            z_sum += position.point.z;
        }
        float x_mean = x_sum / float(valid_position_array.size());
        float y_mean = y_sum / float(valid_position_array.size());
        float z_mean = z_sum / float(valid_position_array.size());

        float squared_residual_error_sum_x = 0;
        float squared_residual_error_sum_y = 0;
        float squared_residual_error_sum_z = 0;
        for(int i=0;i<valid_position_array.size();++i){
            geometry_msgs::PointStamped position = valid_position_array.at(i);
            squared_residual_error_sum_x += std::pow(position.point.x-x_mean, 2);
            squared_residual_error_sum_y += std::pow(position.point.y-y_mean, 2);
            squared_residual_error_sum_z += std::pow(position.point.z-z_mean, 2);
        }
        float sigma_x = std::sqrt( squared_residual_error_sum_x / float(valid_position_array.size()) );
        float sigma_y = std::sqrt( squared_residual_error_sum_y / float(valid_position_array.size()) );
        float sigma_z = std::sqrt( squared_residual_error_sum_z / float(valid_position_array.size()) );


        ROS_INFO_THROTTLE(1.0, "[Check Init] Sigma in the last 5 second (x, y, z): %f, %f, %f", sigma_x, sigma_y, sigma_z);
        if(sigma_x<0.2 && sigma_y<0.2 && sigma_z<0.3){
            return true;
        } else {
            return false;
        }
    }
}

bool uavos::UWB_Mobile::kalmanFilter3DUpdate(const common_msgs::UWB_FullRangeInfo& range_info){
    // KALMANFILTER3D_UPDATE - updates a EKF with 6 state variables
    // Called each time a new mobile-to-anchor measurement is available

    // INPUTS:
    // X - the 6x1 a priori state vector - [x,xdot,y,ydot,z,zdot]'
    // P - the 6x6 a priori covariance matrix
    // T - the delta time between updates
    // sigma_r - the estimated standard deviation of the new range measurement
    // r_meas - the actual range measurement
    // XYZ0 - the [x y z] location of the associated anchor node

    // OUTPUTS:
    // X - the new state vector estimate
    // P - the new covariance matrix estimate
    // innovation - the difference between the range measurement and the estimated
    // range measurement (perhaps useful for outlier filtering.)


    int anchor_id = range_info.responderId;
    readingCounterInc(anchor_id);

    // determine whether to perform filter using SNR
    float SNR = 20 * std::log( range_info.vPeak / ( range_info.noise + 0.1) );
    if(SNR < m_snr_threshold){
        //ROS_WARN("Anchor %d SNR %f too small, discard.", anchor_id, SNR);
        return false;
    }

    // set innovation threshold
    if(false==m_is_initialized){
        m_innovation_threshold = 10;
        m_is_initialized = checkInitialization();
    } else {
        m_innovation_threshold = 2;
    }

    // ------------------------------------------------------------------

    uavos::Vector6d X;
    X(0) = m_position.point.x;
    X(1) = m_velocity.point.x;
    X(2) = m_position.point.y;
    X(3) = m_velocity.point.y;
    X(4) = m_position.point.z;
    X(5) = m_velocity.point.z;

    uavos::Matrix6d P = getStateCovariance();

    double T = range_info.header.stamp.toSec() - m_last_range_time.toSec();
//    double T = range_info.header.stamp.toSec() - getRangeInfoToAnchor(anchor_id).header.stamp.toSec();
    double sigma_r = double(range_info.precisionRangeErrEst) / 1000.0 * 2;
    double sigma_a = m_kalman_sigma_a;
    double r_meas = double(range_info.precisionRangeMm) / 1000.0;

    Eigen::Vector3d XYZ0 = Eigen::VectorXd::Zero(3);
    geometry_msgs::PointStamped anchor_position = m_anchor_map.find(anchor_id)->second->getPosition();
    XYZ0(0) = anchor_position.point.x;
    XYZ0(1) = anchor_position.point.y;
    XYZ0(2) = anchor_position.point.z;


    // F is a 6x6 State Transition Matrix
    uavos::Matrix6d F;
    F << 1,  T,  0,  0,  0,  0,
         0 , 1,  0,  0,  0,  0,
         0,  0,  1,  T,  0,  0,
         0,  0,  0,  1,  0,  0,
         0,  0,  0,  0,  1,  T,
         0,  0,  0,  0,  0,  1;

    // Q is the acceleration model
    double z_damping_factor = 2;
    uavos::Matrix6d Q;
    Q << std::pow(T,4)/3.0,  std::pow(T,3)/2.0,  0,                  0,                  0,                  0,
         std::pow(T,3)/2.0,  std::pow(T,2),      0,                  0,                  0,                  0,
             0,              0,                  std::pow(T,4)/3.0,  std::pow(T,3)/2.0,  0,                  0,
             0,              0,                  std::pow(T,3)/2.0,  std::pow(T,2),      0,                  0,
             0,              0,                  0,                  0,                  std::pow(T,4)/3.0/z_damping_factor,  std::pow(T,3)/2.0/z_damping_factor,
             0,              0,                  0,                  0,                  std::pow(T,3)/2.0/z_damping_factor,  std::pow(T,2)/z_damping_factor;
    Q *= std::pow(sigma_a,2);

    // X is the predicted state vector and the predicted covariance matrix
    X = F*X;

    // M is the predicted covariance matrix
    uavos::Matrix6d M = F*P*F.transpose() + Q;

    // r_pred is the predicted range measure
    double r_pred = std::sqrt( std::pow(X(0) - XYZ0(0), 2) +
                               std::pow(X(2) - XYZ0(1), 2) +
                               std::pow(X(4) - XYZ0(2), 2) ) + 1e-6;
    // H is the linearized measurement matrix
    Eigen::MatrixXd H(1,6);
    H << (X(0) - XYZ0(0))/r_pred,
         0,
         (X(2) - XYZ0(1))/r_pred,
         0,
         (X(4) - XYZ0(2))/r_pred,
         0;

    // K is the Kalman Gain
    uavos::Vector6d K = M*H.transpose() / ( (H*M*H.transpose())(0,0) + std::pow(sigma_r,2) );

    // Update P for the a posteriori covariance matrix
    P = ( Eigen::MatrixXd::Identity(6,6) - K*H ) * M;

    // Return the measurement innovation
    double innovation = r_meas - r_pred;

    // Update the state
    X = X + K * (r_meas - r_pred);

    // decide to take the range info or not.
    if(innovation<m_innovation_threshold){
        setPosition(X(0),X(2),X(4));
        setVelocity(X(1),X(3),X(5));
        setStateCovariance(P);

        setRangeInfoToAnchor(range_info);
        m_last_range_time = range_info.header.stamp;

        successfulReadingCounterInc(anchor_id);
        return true;
    } else {
        ROS_WARN("Anchor id %d, Innovation too large: %f",  anchor_id, innovation);
        return false;
    }


}
