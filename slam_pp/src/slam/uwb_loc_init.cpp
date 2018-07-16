#include "slam/uwb_loc_init.hpp"

bool uavos::UWB_Loc_Init::initializeMobileByCeres(){
    getValidNeighborDatabse();
    buildOptimizationProblem(m_ndb);
    if( solveOptimizationProblem() ){
        m_p_mobile->fullInitializeEKF(m_ndb, m_mobile_position);
        ROS_INFO("Innitialization with Ceres OK. %f, %f, %f",
                 m_mobile_position.point.x,
                 m_mobile_position.point.y,
                 m_mobile_position.point.z);
        return true;
    } else {
        ROS_INFO("Fail to initialize with Ceres.");
        return false;
    }
}

bool uavos::UWB_Loc_Init::getValidNeighborDatabse(){
    const int anchor_number = m_p_mobile->getAnchorNumber();
    int valid_anchor_counter = 0;

    while(valid_anchor_counter<anchor_number){
        ros::Duration(1.0).sleep();
        spinOnce();

        valid_anchor_counter = 0;
        for(int i=0;i<m_ndb.numNeighborEntries;++i){
            geometry_msgs::PointStamped anchor_position;
            std::cout<<m_ndb.neighbors.at(i).nodeId<<", "<<m_ndb.neighbors.at(i).rangeMm<<std::endl;
            if(m_p_mobile->getAnchorPosition(m_ndb.neighbors.at(i).nodeId, anchor_position)
                    && m_ndb.neighbors.at(i).rangeMm/1000.0>0.3){
                valid_anchor_counter += 1;
            }
        }
        ROS_INFO("[Ceres Init] Total anchor: %d, ndb anchor: %d", anchor_number, valid_anchor_counter);
    }

    return true;
}

void uavos::UWB_Loc_Init::buildOptimizationProblem(const common_msgs::UWB_FullNeighborDatabase& ndb){
    ceres::LossFunction* loss_function = new ceres::CauchyLoss(1.0);
    for(int i=0;i<m_ndb.numNeighborEntries;++i){
        geometry_msgs::PointStamped anchor_position;
        if(m_p_mobile->getAnchorPosition(m_ndb.neighbors.at(i).nodeId, anchor_position)){
            ceres::CostFunction* cost_function = uavos::AnchorTriangulationErrorTerm::Create(anchor_position.point.x,
                                                                                             anchor_position.point.y,
                                                                                             anchor_position.point.z,
                                                                                             m_ndb.neighbors.at(i).rangeMm/1000.0,
                                                                                             m_ndb.neighbors.at(i).rangeErrorEstimate/1000.0*5);
            m_problem.AddResidualBlock(cost_function, loss_function,
                                       &(m_mobile_position.point.x),
                                       &(m_mobile_position.point.y),
                                       &(m_mobile_position.point.z));
        }
    }
}

bool uavos::UWB_Loc_Init::solveOptimizationProblem(){
    ceres::Solver::Options options;
    options.max_num_iterations = 10000;
    options.minimizer_progress_to_stdout = false;
    options.linear_solver_type = ceres::DENSE_QR;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &m_problem, &summary);

    std::cout<<"\n"<<summary.BriefReport()<<"\n";
//    std::cout << "Initial cost: " << summary.initial_cost << "  Final cost: "<< summary.final_cost << '\n';

    return summary.IsSolutionUsable();

}

