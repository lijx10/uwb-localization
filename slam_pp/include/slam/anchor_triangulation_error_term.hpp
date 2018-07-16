#ifndef ANCHOR_TRIANGULATION_ERROR_TERM_HPP
#define ANCHOR_TRIANGULATION_ERROR_TERM_HPP

#include "Eigen/Core"
#include "ceres/ceres.h"
#include <iostream>

namespace uavos{

class AnchorTriangulationErrorTerm{
public:
    AnchorTriangulationErrorTerm(const double anchor_x, const double anchor_y, const double anchor_z,
                                 const double r, const double covariance):
        m_r(r), m_covariance(covariance){
        m_anchor << anchor_x, anchor_y, anchor_z;
    }


    template <typename T>
    bool operator()(const T* const x, const T* const y, const T* const z,
                    T* residuals_ptr) const{
        const Eigen::Matrix<T, 3, 1> anchor = m_anchor.cast<T>();
        const Eigen::Matrix<T, 3, 1> mobile(*x, *y, *z);
        Eigen::Matrix<T, 3, 1> diff = anchor - mobile;
        if(diff(0,0)==T(0) && diff(1,0)==T(0) && diff(2,0)==T(0)){
            diff(0,0) = T(1e-6);
        }

        residuals_ptr[0] = ( ceres::abs( diff.norm() - T(m_r) ) ) * T(1.0/std::sqrt(m_covariance));

        // avoid zero derivate for sqrt()
        const T threshold = T(1e-6);
        if(residuals_ptr[0]<threshold){
            residuals_ptr[0] = threshold;
        }

        return true;
    }

    static ceres::CostFunction* Create(const double anchor_x, const double anchor_y, const double anchor_z,
                                       const double r, const double covariance){
        return (new ceres::AutoDiffCostFunction<AnchorTriangulationErrorTerm, 1, 1, 1, 1>
                ( new AnchorTriangulationErrorTerm(anchor_x, anchor_y, anchor_z, r, covariance ) )
                );
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
    Eigen::Vector3d m_anchor;
    const double m_r, m_covariance;
};


}



#endif // ACHOR_TRIANGULATION_ERROR_TERM_HPP
