#include "anchor_calibration.hpp"


uavos::Anchor_Calibration::Anchor_Calibration(ros::NodeHandle& nh):
    m_nh(nh)
{
    m_echo_sub = m_nh.subscribe("/time_domain/echoed_range", 10, &uavos::Anchor_Calibration::echoedRangeCallback, this);
    m_range_sub = m_nh.subscribe("/time_domain/full_range_info", 10, &uavos::Anchor_Calibration::rangeInfoCallback, this);
//    m_matrix_printer_timer = m_nh.createTimer(ros::Duration(2.0), &uavos::Anchor_Calibration::printAffinityMatrixCallback, this);
    m_matrix_printer_timer = m_nh.createTimer(ros::Duration(2.0), &uavos::Anchor_Calibration::printAffinityAndVarianceMatrixCallback, this);
    m_optimization_timer = m_nh.createTimer(ros::Duration(2.0), &uavos::Anchor_Calibration::calibrationCallback, this);

    m_problem_ready = false;
    m_problem_solved = false;

    std::srand(std::time(NULL));

    // load parameter
    m_nh.param<bool>("is_consider_covariance", m_is_consider_covariance, true);
    m_nh.param<bool>("is_use_cauchy_loss", m_is_use_cauchy_loss, false);
    m_nh.param<double>("cauchy_loss", m_cauchy_loss, 10);
}


boost::shared_ptr<uavos::UWB_Anchor> uavos::Anchor_Calibration::createAnchor(const int id){
    // read ros param to get initial position guess
    std::vector<double> init_position;
    m_nh.getParam(std::string("anchor_")+uavos::num2str(id), init_position);

    double init_var = 10;
    boost::shared_ptr<uavos::UWB_Anchor> p_new_anchor;
    if(init_position.size()==3){
        p_new_anchor = boost::shared_ptr<uavos::UWB_Anchor> (new uavos::UWB_Anchor(id,
                                                                                   init_position.at(0),
                                                                                   init_position.at(1),
                                                                                   init_position.at(2),
                                                                                   init_var, init_var, init_var) );
    } else {
        p_new_anchor = boost::shared_ptr<uavos::UWB_Anchor> (new uavos::UWB_Anchor(id,
                                                                                   1.0/double(std::rand()),
                                                                                   1.0/double(std::rand()),
                                                                                   1.0/double(std::rand()),
                                                                                   init_var,init_var,init_var) );
    }

    return p_new_anchor;
}

void uavos::Anchor_Calibration::echoedRangeCallback(const common_msgs::UWB_EchoedRangeInfo::ConstPtr &msg){
    // check whether the responder/requester has been seen, if not, add it.
    int requester_id = msg->requesterId;
    int responder_id = msg->responderId;
    if(m_anchor_map.find(requester_id)==m_anchor_map.end()){
        boost::shared_ptr<uavos::UWB_Anchor> p_new_anchor = createAnchor(requester_id);
        m_anchor_map.insert(std::pair<int, boost::shared_ptr<uavos::UWB_Anchor> >(requester_id, p_new_anchor));
    }
    if (m_anchor_map.find(responder_id)==m_anchor_map.end() ){
        boost::shared_ptr<uavos::UWB_Anchor> p_new_anchor = createAnchor(responder_id);
        m_anchor_map.insert(std::pair<int, boost::shared_ptr<uavos::UWB_Anchor> >(responder_id, p_new_anchor));
    }

    // insert the echo range info to the anchor_map, each anchor stores its range to others, i.e.
    // each anchor store the range measurement that requested by itself.
    m_anchor_map.find(requester_id)->second->insertRangeInfo(*msg);
}

void uavos::Anchor_Calibration::rangeInfoCallback(const common_msgs::UWB_FullRangeInfo::ConstPtr &msg){
    // check whether the responder/requester has been seen, if not, add it.
    int requester_id = msg->nodeId;
    int responder_id = msg->responderId;
    if(m_anchor_map.find(requester_id)==m_anchor_map.end()){
        boost::shared_ptr<uavos::UWB_Anchor> p_new_anchor = createAnchor(requester_id);
        m_anchor_map.insert(std::pair<int, boost::shared_ptr<uavos::UWB_Anchor> >(requester_id, p_new_anchor));
    }
    if (m_anchor_map.find(responder_id)==m_anchor_map.end() ){
        boost::shared_ptr<uavos::UWB_Anchor> p_new_anchor = createAnchor(responder_id);
        m_anchor_map.insert(std::pair<int, boost::shared_ptr<uavos::UWB_Anchor> >(responder_id, p_new_anchor));
    }

    // insert the echo range info to the anchor_map, each anchor stores its range to others, i.e.
    // each anchor store the range measurement that requested by itself.
    m_anchor_map.find(requester_id)->second->insertRangeInfo(*msg);
}

Eigen::MatrixXd uavos::Anchor_Calibration::getAnchorAffinityMatrix(){
    int anchor_number = m_anchor_map.size();
    Eigen::MatrixXd affinity = Eigen::MatrixXd::Zero(anchor_number, anchor_number);

    // iterate over the anchor map, to get the average affinity matrix.
    // iterate as requester
    std::map<int, boost::shared_ptr<uavos::UWB_Anchor> >::iterator iter;
    int i=0;
    for(iter=m_anchor_map.begin(); iter!=m_anchor_map.end(); ++iter){
        int j=0;
        int requester_id = iter->first;
        // iterate again as the responder
        std::map<int, boost::shared_ptr<uavos::UWB_Anchor> >::iterator iter_responder;
        for(iter_responder=m_anchor_map.begin();
            iter_responder!=m_anchor_map.end();
            iter_responder++){
            int responder_id = iter_responder->first;
            double req2res_range = iter->second->getAverageRangeToNode(responder_id);
            affinity(i,j) = req2res_range;

            j++;
        }
        i++;
    }

    return affinity;
}

bool uavos::Anchor_Calibration::getAnchorAffinityAndVarianceMatrix(Eigen::MatrixXd &affinity, Eigen::MatrixXd &cov_mat){
  int anchor_number = m_anchor_map.size();
  affinity = Eigen::MatrixXd::Zero(anchor_number, anchor_number);
  cov_mat = Eigen::MatrixXd::Zero(anchor_number, anchor_number);

  // iterate over the anchor map, to get the average affinity matrix.
  // iterate as requester
  std::map<int, boost::shared_ptr<uavos::UWB_Anchor> >::iterator iter;
  int i=0;
  for(iter=m_anchor_map.begin(); iter!=m_anchor_map.end(); ++iter){
      int j=0;
      int requester_id = iter->first;
      // iterate again as the responder
      std::map<int, boost::shared_ptr<uavos::UWB_Anchor> >::iterator iter_responder;
      for(iter_responder=m_anchor_map.begin();
          iter_responder!=m_anchor_map.end();
          iter_responder++){
          int responder_id = iter_responder->first;

          double median = 0;
          double variance = 0;
          iter->second->getMedianAndVarianceToNode(responder_id, median, variance);

          affinity(i,j) = median;
          cov_mat(i,j) = variance;

          j++;
      }
      i++;
  }

  return true;
}

void uavos::Anchor_Calibration::printAffinityMatrixCallback(const ros::TimerEvent& event){
    if(false==m_problem_solved){
        std::cout<<"\nAnchor Affinity Matrix:\n"<<getAnchorAffinityMatrix()<<std::endl;
    }

}

void uavos::Anchor_Calibration::printAffinityAndVarianceMatrixCallback(const ros::TimerEvent& event){
    if(false==m_problem_solved){
        Eigen::MatrixXd affinity, cov_mat;
        getAnchorAffinityAndVarianceMatrix(affinity, cov_mat);

        std::cout<<"\n------\nAnchor Affinity Matrix:\n"
                <<affinity
                <<"\nAnchor Variance Matrix:\n"
                <<cov_mat
                <<std::endl;
    }

}

void uavos::Anchor_Calibration::printAnchorMap(){
    std::cout<<"\n\n------------------\nAnchor list:"<<std::endl;
    std::map<int, boost::shared_ptr<uavos::UWB_Anchor> >::iterator iter;

    // print the id of anchors
    for(iter=m_anchor_map.begin();iter!=m_anchor_map.end();++iter){
        std::cout<<iter->second->getNodeId()<<", ";
    }
    std::cout<<std::endl;

    std::cout<<"\nAnchor detail"<<std::endl;
    for(iter=m_anchor_map.begin();iter!=m_anchor_map.end();++iter){
        // iter is each anchor
        boost::shared_ptr<uavos::UWB_Anchor> p_anchor = iter->second;
        std::cout<<"\nRequester "<<p_anchor->getNodeId()<<": "<<std::endl;

        std::map<int, std::vector<double> >::iterator range_map_iter;
        for(range_map_iter=p_anchor->m_range_map.begin();
            range_map_iter!=p_anchor->m_range_map.end();
            ++range_map_iter){
            // iter is each responder, inside that anchor

            std::cout<<"Responder "<<range_map_iter->first<<" : ";
            std::vector<double> range_array = range_map_iter->second;
            for(int i=0;i<range_array.size();++i){
                std::cout<<range_array.at(i)<<", ";
            }
            std::cout<<std::endl;
        }
    }
}

void uavos::Anchor_Calibration::printAnchorMapCallback(const ros::TimerEvent &event){
    printAnchorMap();
}




void uavos::Anchor_Calibration::printOptimizedAnchorAffinityMatrix(){
    int anchor_number = m_anchor_map.size();
    Eigen::MatrixXd affinity = Eigen::MatrixXd::Zero(anchor_number, anchor_number);

    int i=0;
    for(std::map<int, boost::shared_ptr<uavos::UWB_Anchor> >::iterator iter_i=m_anchor_map.begin();
        iter_i!=m_anchor_map.end();
        ++iter_i){
        geometry_msgs::PointStamped anchor_i_pose = iter_i->second->getPosition();

        int j=i;
        for(std::map<int, boost::shared_ptr<uavos::UWB_Anchor> >::iterator iter_j=iter_i;
            iter_j!=m_anchor_map.end();
            ++iter_j){
            geometry_msgs::PointStamped anchor_j_pose = iter_j->second->getPosition();

            affinity(i,j) = std::sqrt(   std::pow(anchor_i_pose.point.x-anchor_j_pose.point.x,2)
                                       + std::pow(anchor_i_pose.point.y-anchor_j_pose.point.y,2)
                                       + std::pow(anchor_i_pose.point.z-anchor_j_pose.point.z,2) );
            j+=1;
        }

        i+=1;
    }

    std::cout<<"\nOptimized affinity matrix: "<<std::endl<<affinity<<std::endl;
}


// ----------------------------------------
// Ceres optimization
void uavos::Anchor_Calibration::buildOptimizationProblem(){

    // ensure that the anchor map is built.
    assert(m_anchor_map.size()>0);

    ceres::LossFunction* loss_function;
    if(true==m_is_use_cauchy_loss){
        loss_function = new ceres::CauchyLoss(m_cauchy_loss);
    } else {
        loss_function = NULL;
    }

    Eigen::MatrixXd affinity_matrix;
    Eigen::MatrixXd covariance_matrix;
    getAnchorAffinityAndVarianceMatrix(affinity_matrix, covariance_matrix);
    const double sqrt_information_const = 1;
    std::map<int, boost::shared_ptr<uavos::UWB_Anchor> >::iterator iter_requester;
    std::map<int, boost::shared_ptr<uavos::UWB_Anchor> >::iterator iter_responder;



    // print init anchor position
    std::cout<<"\nInitial anchor position: \n";
    for(iter_requester=m_anchor_map.begin();iter_requester!=m_anchor_map.end();++iter_requester){
        iter_requester->second->printPosition();
    }
    std::cout<<std::endl;


    // set UWB range constraints
    int i=0;
    for(iter_requester=m_anchor_map.begin();iter_requester!=m_anchor_map.end();++iter_requester){
        geometry_msgs::PointStamped* requester_position_ptr = iter_requester->second->getPositionPtr();

        int j=0;
        for(iter_responder=m_anchor_map.begin();iter_responder!=m_anchor_map.end();++iter_responder){
            geometry_msgs::PointStamped* responder_position_ptr = iter_responder->second->getPositionPtr();

            // no need to optimize itself-itself constrain, because it equals 0.
            if(i==j){
                ++j;
                continue;
            }

            ceres::CostFunction* cost_function;
            if(false==m_is_consider_covariance){
                cost_function = uavos::TriangulationErrorTerm::Create( affinity_matrix(i,j),
                                                                       sqrt_information_const );
            } else {
                cost_function = uavos::TriangulationErrorTerm::Create( affinity_matrix(i,j),
                                                                       1.0/std::sqrt(covariance_matrix(i,j)) );
            }




            m_problem.AddResidualBlock(cost_function, loss_function,
                                        &(requester_position_ptr->point.x), &(requester_position_ptr->point.y), &(requester_position_ptr->point.z),
                                        &(responder_position_ptr->point.x), &(responder_position_ptr->point.y), &(responder_position_ptr->point.z));


            ++j;
        }
        ++i;
    }


    // set direction constraint & coordinate constraint.
    for(iter_requester=m_anchor_map.begin();iter_requester!=m_anchor_map.end();++iter_requester){

        // direction constraints
        std::vector<double> line_constraint_params;
        std::string param_key_line = std::string("anchor_") +
                uavos::num2str( iter_requester->second->getNodeId() ) +
                               std::string("_direction");

        m_nh.getParam(param_key_line, line_constraint_params);
        if(6==line_constraint_params.size()){
            // have correct direction constraint, add it
            std::cout<<"Found direction constraint: "<<param_key_line<<std::endl;

            geometry_msgs::PointStamped* requester_position_ptr = iter_requester->second->getPositionPtr();
            ceres::CostFunction* cost_function = uavos::PointLineErrorTerm::Create(line_constraint_params.at(0),
                                                                                   line_constraint_params.at(1),
                                                                                   line_constraint_params.at(2),
                                                                                   line_constraint_params.at(3),
                                                                                   line_constraint_params.at(4),
                                                                                   line_constraint_params.at(5));
            m_problem.AddResidualBlock(cost_function, loss_function,
                                       &(requester_position_ptr->point.x), &(requester_position_ptr->point.y), &(requester_position_ptr->point.z));
        }


        // coordinate constraints
        std::map<std::string, double> coordinate_constraint_params;
        std::string param_key_coordinate = std::string("anchor_") +
                uavos::num2str( iter_requester->second->getNodeId() ) +
                               std::string("_coordinate");

        m_nh.getParam(param_key_coordinate, coordinate_constraint_params);
        if(coordinate_constraint_params.size()>0){
            geometry_msgs::PointStamped* requester_position_ptr = iter_requester->second->getPositionPtr();
            ceres::CostFunction* cost_function = uavos::CoordinateErrorTerm::Create(coordinate_constraint_params);
            m_problem.AddResidualBlock(cost_function, loss_function,
                                       &(requester_position_ptr->point.x), &(requester_position_ptr->point.y), &(requester_position_ptr->point.z));
        }


    }



    // add constant block parameter
    m_problem.SetParameterBlockConstant(&(m_anchor_map.begin()->second->getPositionPtr()->point.x));
    m_problem.SetParameterBlockConstant(&(m_anchor_map.begin()->second->getPositionPtr()->point.y));
    m_problem.SetParameterBlockConstant(&(m_anchor_map.begin()->second->getPositionPtr()->point.z));

    m_problem_ready = true;
}


bool uavos::Anchor_Calibration::solveOptimizationProblem(){
    assert(m_problem_ready==true);

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


void uavos::Anchor_Calibration::calibrationCallback(const ros::TimerEvent &event){
    // check whether calibration has been done.
    if(true==m_problem_solved){
        return;
    }

    // check whether there is enough constraints.
    if(m_anchor_map.size()<2){
        return;
    }

    // ensure the number of measurement is larger than a certain threshold
    int least_number_of_measurement = 20;

    std::map<int, boost::shared_ptr<uavos::UWB_Anchor> >::iterator iter;
    for(iter=m_anchor_map.begin();iter!=m_anchor_map.end();++iter){
        // iter is each anchor
        boost::shared_ptr<uavos::UWB_Anchor> p_anchor = iter->second;
        std::map<int, std::vector<double> >::iterator range_map_iter;
        for(range_map_iter=p_anchor->m_range_map.begin();
            range_map_iter!=p_anchor->m_range_map.end();
            ++range_map_iter){
            // iter is each responder, inside that anchor
            if(range_map_iter->second.size()<least_number_of_measurement){
                ROS_INFO("To few measurement %zd: requester %d, responder %d",
                         range_map_iter->second.size(),
                         p_anchor->getNodeId(),
                         range_map_iter->first);
                return;
            }
        }
    }

    printAnchorMap();

    buildOptimizationProblem();
    solveOptimizationProblem();
    m_problem_solved = true;

    // print the id of anchors
    std::cout<<"\nOptimized anchor position: \n";
    std::map<int, boost::shared_ptr<uavos::UWB_Anchor> >::iterator iter_requester;
    for(iter_requester=m_anchor_map.begin();iter_requester!=m_anchor_map.end();++iter_requester){
        iter_requester->second->printPosition();
    }
    std::cout<<std::endl;

    // print optimized affinity matrix
    printOptimizedAnchorAffinityMatrix();
}
