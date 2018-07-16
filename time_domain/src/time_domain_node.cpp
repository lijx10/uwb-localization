#include <iostream>
#include <cstdlib>
#include <cstdio>
#include <iomanip>
#include <cmath>

#include <ros/spinner.h>

#include "time_domain_interface.hpp"


int main(int argc, char **argv){

    //initialize ros
    ros::init(argc, argv, "time_domain");
    ros::NodeHandle time_domain_node("~");

    std::cout<<"------------- [time_domain] node starts. -------------"<<std::endl;


    boost::shared_ptr<uavos::TimeDomainInterface> p_td_interface(new uavos::TimeDomainInterface(time_domain_node));
    p_td_interface->setupUWB();

    ros::Timer read_msg_timer = time_domain_node.createTimer(ros::Duration(1.0/100), &uavos::TimeDomainInterface::readMsgCallback, p_td_interface);
    //ros::Timer watch_dog_callback = time_domain_node.createTimer(ros::Duration(0.2), &uavos::TimeDomainInterface::watchDogCallback, p_td_interface);

    ros::spin();
    //ros::MultiThreadedSpinner spinner(2); // Use 2 threads
    //spinner.spin(); // spin() will not return until the node has been shutdown

    std::cout<<"------------- [time_domain] node exits -------------"<<std::endl;


    return 0;

}
