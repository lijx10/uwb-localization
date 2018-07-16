#include "basic_function.hpp"
#include "slam/thread_slam.hpp"
#include "mapping/thread_mapping.hpp"
#include "pp/thread_pp.hpp"

using namespace uavos;

int main(int argc, char **argv){

    //initialize ros
    ros::init(argc, argv, "slam_pp");
    ros::NodeHandle slam_pp_node("~");

    //google::InitGoogleLogging(argv[0]);



    std::cout<<"------------- [slam_pp] node starts. -------------"<<std::endl;

    boost::shared_ptr<Mapping_Controller> p_mapping_controller(new Mapping_Controller());
    p_mapping_controller->start();
    std::cout<<"------------- [slam_pp] mapping thread started. -------------"<<std::endl;

    ros::Duration(1).sleep();

    boost::shared_ptr<SLAM_Controller> p_slam_controller(new SLAM_Controller());
    p_slam_controller->start();
    std::cout<<"------------- [slam_pp] slam thread started. -------------"<<std::endl;

    boost::shared_ptr<PP_Controller> p_pp_controller(new PP_Controller(p_mapping_controller));
    p_pp_controller->start();
    std::cout<<"------------- [slam_pp] pp thread started. -------------"<<std::endl;


    ros::spin();
    std::cout<<"------------- [slam_pp] node exits -------------"<<std::endl;


    return 0;
}
