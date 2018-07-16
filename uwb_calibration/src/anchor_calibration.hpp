#ifndef ANCHOR_CALIBRATION_H
#define ANCHOR_CALIBRATION_H

#include "uwb_node.hpp"
#include "triangulation_error_term.hpp"
#include "point_line_distance_error_term.hpp"
#include "coordinate_error_term.hpp"
#include "ceres/ceres.h"
#include "basic_functions.hpp"

#include <time.h>

namespace uavos{

class Anchor_Calibration
{
public:
    explicit Anchor_Calibration(ros::NodeHandle& nh);

private:
    ros::NodeHandle m_nh;
    ros::Subscriber m_echo_sub;
    ros::Subscriber m_range_sub;
    ros::Timer m_matrix_printer_timer;
    ros::Timer m_optimization_timer;

    std::map<int, boost::shared_ptr<uavos::UWB_Anchor>, std::less<int> > m_anchor_map;

    // Ceres optimization
    ceres::Problem m_problem;
    bool m_problem_ready;
    bool m_problem_solved;

    // parameters
    bool m_is_consider_covariance;
    bool m_is_use_cauchy_loss;
    double m_cauchy_loss;



public:
    boost::shared_ptr<uavos::UWB_Anchor> createAnchor(const int id);
    void echoedRangeCallback(const common_msgs::UWB_EchoedRangeInfo::ConstPtr& msg);
    void rangeInfoCallback(const common_msgs::UWB_FullRangeInfo::ConstPtr& msg);

    void printAnchorMap();
    void printAnchorMapCallback(const ros::TimerEvent& event);

    Eigen::MatrixXd getAnchorAffinityMatrix();
    void printAffinityMatrixCallback(const ros::TimerEvent& event);

    bool getAnchorAffinityAndVarianceMatrix(Eigen::MatrixXd& affinity, Eigen::MatrixXd& cov_mat);
    void printAffinityAndVarianceMatrixCallback(const ros::TimerEvent& event);

    void printOptimizedAnchorAffinityMatrix();

    // Ceres optimization
    void buildOptimizationProblem();
    bool solveOptimizationProblem();
    void calibrationCallback(const ros::TimerEvent& event);
};


}

#endif // ANCHOR_CALIBRATION_H
