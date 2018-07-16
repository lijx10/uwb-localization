#ifndef POINT_LINE_DISTANCE_ERROR_TERM_HPP
#define POINT_LINE_DISTANCE_ERROR_TERM_HPP

#include "Eigen/Core"
#include "ceres/ceres.h"
#include <iostream>

namespace uavos{

class PointLineErrorTerm{
public:
    PointLineErrorTerm(const double a0, const double a1, const double a2,
                       const double n0, const double n1, const double n2){
        a << a0, a1, a2;
        n << n0, n1, n2;
        n /= n.norm();
    }

    template <typename T>
    bool operator()(const T* const x, const T* const y, const T* const z,
                    T* residuals_ptr) const{
        Eigen::Matrix<T, 3, 1> p(*x, *y, *z);

        Eigen::Matrix<T, 3, 1> a_casted = a.cast<T>();
        Eigen::Matrix<T, 3, 1> n_casted = n.cast<T>();
        Eigen::Matrix<T, 3, 1> dist_vec = (a_casted-p) - ( (a_casted-p).dot(n_casted) ) * n_casted;

        if(dist_vec(0,0)==T(0) && dist_vec(1,0)==T(0) && dist_vec(2,0)==T(0)){
            dist_vec(0,0) = T(1e-6);
        }

        residuals_ptr[0] = dist_vec.norm() * T(100);

        // avoid zero derivate for sqrt()
        const T threshold = T(1e-6);
        if(residuals_ptr[0]<threshold){
            residuals_ptr[0] = threshold;
        }

        return true;
    }

    static ceres::CostFunction* Create(const double a0, const double a1, const double a2,
                                       const double n0, const double n1, const double n2){
        return (new ceres::AutoDiffCostFunction<PointLineErrorTerm, 1, 1, 1, 1>( new PointLineErrorTerm(a0, a1, a2, n0, n1, n2) ) );
    }


    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
    Eigen::Vector3d a;
    Eigen::Vector3d n;
};


}

#endif // POINT_LINE_DISTANCE_ERROR_TERM_HPP
