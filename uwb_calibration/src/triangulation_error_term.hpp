#ifndef TRIANGULATION_ERROR_TERM_HPP
#define TRIANGULATION_ERROR_TERM_HPP

#include "Eigen/Core"
#include "ceres/ceres.h"
#include <iostream>

namespace uavos{


class TriangulationErrorTerm{
public:
    TriangulationErrorTerm(const double r_ab, const double sqrt_information):
        m_r_ab(r_ab), m_sqrt_information(sqrt_information){}

    template <typename T>
    bool operator()(const T* const x_a, const T* const y_a, const T* const z_a,
                    const T* const x_b, const T* const y_b, const T* const z_b,
                    T* residuals_ptr) const{
        const Eigen::Matrix<T, 3, 1> p_a(*x_a, *y_a, *z_a);
        const Eigen::Matrix<T, 3, 1> p_b(*x_b, *y_b, *z_b);
        Eigen::Matrix<T, 3, 1> diff = p_a - p_b;
        if(diff(0,0)==T(0) && diff(1,0)==T(0) && diff(2,0)==T(0)){
            diff(0,0) = T(1e-6);
        }

        residuals_ptr[0] = ceres::abs( diff.norm() - T(m_r_ab) );

        // consider the sqrt_information, i.e. 1/sqrt(covariance)
        residuals_ptr[0] = residuals_ptr[0] * T(m_sqrt_information);

        // avoid zero derivate for sqrt()
        const T threshold = T(1e-6);
        if(residuals_ptr[0]<threshold){
            residuals_ptr[0] = threshold;
        }

//        std::cout<<*x_a<<", "<<*y_a<<", "<<*z_a<<std::endl;
//        std::cout<<*x_b<<", "<<*y_b<<", "<<*z_b<<std::endl;

//        T distance = ceres::sqrt( (*x_a-*x_b)*(*x_a-*x_b) +
//                                  (*y_a-*y_b)*(*y_a-*y_b) +
//                                  (*z_a-*z_b)*(*z_a-*z_b) );
//        residuals_ptr[0] = ceres::abs( T(m_r_ab) - distance );

//        std::cout<<"distance: "<<distance<<", r_ab: "<<m_r_ab<<std::endl;
//        std::cout<<residuals_ptr[0]<<std::endl;
//        std::cout<<"-------"<<std::endl;

        return true;

    }

    static ceres::CostFunction* Create(const double r_ab, const double sqrt_information){
        return (new ceres::AutoDiffCostFunction<TriangulationErrorTerm, 1, 1, 1, 1, 1, 1, 1>( new TriangulationErrorTerm(r_ab, sqrt_information) ) );
//        return (new ceres::NumericDiffCostFunction<TriangulationErrorTerm, ceres::CENTRAL, 1, 1, 1, 1, 1, 1, 1>( new TriangulationErrorTerm(r_ab, sqrt_information) ) );
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
    const double m_r_ab;
    const double m_sqrt_information;

};

}


#endif // TRIANGULATION_ERROR_TERM_HPP
