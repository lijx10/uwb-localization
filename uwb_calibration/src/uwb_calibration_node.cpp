#include "anchor_calibration.hpp"
#include "uwb_node.hpp"

using namespace uavos;

int main(int argc, char **argv){

    //initialize ros
    ros::init(argc, argv, "uwb_calibration");
    ros::NodeHandle uwb_calibration_node("~");

    std::cout<<"------------- [uwb_calibration] node starts. -------------"<<std::endl;

    uavos::Anchor_Calibration calib(uwb_calibration_node);

    ros::spin();
    std::cout<<"------------- [uwb_calibration] node exits -------------"<<std::endl;

    return 0;
}
