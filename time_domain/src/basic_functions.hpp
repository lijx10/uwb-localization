#ifndef BASIC_FUNCTIONS_HPP
#define BASIC_FUNCTIONS_HPP

#include <sstream>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <cstdio>
#include <cstdlib>
#include <vector>
#include <string>

#include "ros/ros.h"
#include "std_msgs/UInt32.h"
#include "std_msgs/UInt16.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/UInt8MultiArray.h"
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
#include <ros/spinner.h>
#include <ros/package.h>

#include "common_msgs/UWB_FullRangeInfo.h"
#include "common_msgs/UWB_FullNeighborDatabaseEntry.h"
#include "common_msgs/UWB_FullNeighborDatabase.h"
#include "common_msgs/UWB_DataInfo.h"
#include "common_msgs/UWB_EchoedRangeInfo.h"
#include "common_msgs/UWB_SendData.h"

#include <boost/shared_ptr.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/trim.hpp>

namespace uavos{

template<typename T>
inline std::string num2str(const T src){
    std::stringstream ss;
    ss << src;
    return ss.str();
}

template<typename T>
std::vector<T> lineStr2array(const std::string line, const std::string separator){
    std::vector<std::string> tokens;
    boost::split(tokens, line, boost::is_any_of(separator.c_str()));

    std::vector<T> dst;
    for(int i=0;i<tokens.size();++i){
        try{
            boost::trim(tokens.at(i));
            dst.push_back( boost::lexical_cast<T>(tokens.at(i)) );
        } catch (boost::bad_lexical_cast const&){
            std::cout<<"string to number conversion error: "<<tokens.at(i)<<std::endl;
        }
    }

    return dst;
}

template<typename T>
int readFromTxt(std::vector<std::vector<T> > &data_array, const std::string file_name, const std::string separator){

    std::ifstream txtFILE(file_name.c_str());


    if(!txtFILE){
        printf("Open file failed.\n");
        return 0;
    }

    std::string line;
    while (std::getline(txtFILE, line)){
        std::vector<T> line_array = lineStr2array<T>(line, separator);
        data_array.push_back(line_array);

    }

    return data_array.size();
}

inline int test(){
    return 1;
}




}

#endif // BASIC_FUNCTIONS_HPP
