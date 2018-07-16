#ifndef COORDINATE_ERROR_TERM_HPP
#define COORDINATE_ERROR_TERM_HPP

#include "Eigen/Core"
#include "ceres/ceres.h"
#include <iostream>
#include <string>
#include <map>

namespace uavos{

class CoordinateErrorTerm{
public:
    CoordinateErrorTerm(const std::map<std::string, double> coordinate_constraint_map):
    m_coordinate_constraint_map(coordinate_constraint_map){}

    template <typename T>
    bool operator()(const T* const x, const T* const y, const T* const z,
                    T* residuals_ptr) const{
        residuals_ptr[0] = T(0);

        std::map<std::string, double>::iterator iter;
        for(iter=m_coordinate_constraint_map.begin();iter!=m_coordinate_constraint_map.end();++iter){
            if(std::string("x")==iter->first){
                residuals_ptr[0] += T(100) * (*x - T(iter->second));
            } else if(std::string("y")==iter->first){
                residuals_ptr[0] += T(100) * (*y - T(iter->second));
            } else if(std::string("z")==iter->first){
                residuals_ptr[0] += T(100) * (*z - T(iter->second));
            }
        }

        return true;
    }

    static ceres::CostFunction* Create(const std::map<std::string, double> coordinate_constraint_map){
        return (new ceres::AutoDiffCostFunction<CoordinateErrorTerm, 1, 1, 1, 1>( new CoordinateErrorTerm(coordinate_constraint_map) ) );
    }

private:
    mutable std::map<std::string, double> m_coordinate_constraint_map;

};



}




#endif // COORDINATE_ERROR_TERM_HPP
