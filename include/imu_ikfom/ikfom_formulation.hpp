#pragma once

#include "IKFoM/IKFoM_toolkit/esekfom/esekfom.hpp"

// hyper params
#define G 9.81

// scalar type of variable: double
typedef MTK::SO3<double> SO3;
// dimension of the defined Euclidean variable: 3
typedef MTK::vect<3, double> vect3;

// state
MTK_BUILD_MANIFOLD(
    state, // name of compound manifold: state
    ((SO3, rot))
);

// input
MTK_BUILD_MANIFOLD(
    input,
    ((vect3, ang_vel))
);

// measurement
MTK_BUILD_MANIFOLD(
    measurement,
    ((vect3, lin_acc))
);

// process noise
MTK_BUILD_MANIFOLD(
    process_noise,
    ((vect3, ang_vel))
);

// measurement noise
MTK_BUILD_MANIFOLD(
    measurement_noise,
    ((vect3, lin_acc))
);

// system dynamics
inline Eigen::Matrix<double, 3, 1> f(state &x, const input &u) {
    return u.ang_vel;
}

inline Eigen::Matrix<double, 3, 3> df_dx(state &x, const input &u) {
    return Eigen::Matrix3d::Zero();
}

inline Eigen::Matrix<double, 3, 3> df_dw(state &x, const input &u) {
    return Eigen::Matrix3d::Identity();
}

// measurement
inline measurement h(state &x, bool &is_valid) {
    measurement result;
    const Eigen::Vector3d g(0.0, 0.0, G);
    result.lin_acc = x.rot.toRotationMatrix().transpose() * g;
    return result;
}

inline Eigen::Matrix<double, 3, 3> dh_dx(state &x, bool &is_valid) {
    const Eigen::Vector3d g(0.0, 0.0, G);
    vect3 g_imu = x.rot.toRotationMatrix().transpose() * g;
    return MTK::hat(g_imu);
}

inline Eigen::Matrix<double, 3, 3> dh_dv(state &x, bool &is_valid) {
    return Eigen::Matrix<double, 3, 3>::Identity();
}

// process noise covariance (Q) matrix
inline MTK::get_cov<process_noise>::type mat_Q(double var_ang_vel) {
    MTK::get_cov<process_noise>::type mat = MTK::get_cov<process_noise>::type::Zero();
    MTK::setDiagonal<process_noise, vect3, 0>(mat, &process_noise::ang_vel, var_ang_vel);
    return mat;
}

// measurement noise covariance (R) matrix
inline Eigen::Matrix3d mat_R(double var_lin_acc) {
    return var_lin_acc * Eigen::Matrix3d::Identity();
}
