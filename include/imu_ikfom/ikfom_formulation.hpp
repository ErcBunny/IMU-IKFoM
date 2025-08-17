#pragma once

#include "IKFoM/IKFoM_toolkit/esekfom/esekfom.hpp"
#include "gen.h"

// scalar type of variable: double
typedef MTK::SO3<double> SO3;
// dimension of the defined Euclidean variable: 3
typedef MTK::vect<3, double> vect3;

inline Eigen::Vector3d zero3d = Eigen::Vector3d::Zero();

#define STATE_LEN 3
#define STATE_DOF 3
#define MEAS_DOF 3
#define PROC_NOISE_DOF 3
#define MEAS_NOISE_DOF 3

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
inline Eigen::Matrix<double, STATE_LEN, 1> f(state &x, const input &u) {
    const casadi_real *arg[f_SZ_ARG] = {nullptr};
    const auto rot_mat = x.rot.toRotationMatrix();
    arg[0] = rot_mat.data();
    arg[1] = u.ang_vel.data();
    arg[2] = zero3d.data();

    casadi_real *res[f_SZ_RES] = {nullptr};
    Eigen::Matrix<double, STATE_LEN, 1> result;
    res[0] = result.data();

    casadi_int iw[f_SZ_IW + 1];
    casadi_real w[f_SZ_W];
    f(arg, res, iw, w, 0);
    return result;
}

inline Eigen::Matrix<double, STATE_LEN, STATE_DOF> df_dx(state &x, const input &u) {
    const casadi_real *arg[df_dx_SZ_ARG] = {nullptr};
    const auto rot_mat = x.rot.toRotationMatrix();
    arg[0] = rot_mat.data();
    arg[1] = zero3d.data();
    arg[2] = u.ang_vel.data();

    casadi_real *res[df_dx_SZ_RES] = {nullptr};
    Eigen::Matrix<double, STATE_LEN, STATE_DOF> result;
    res[0] = result.data();

    casadi_int iw[df_dx_SZ_IW + 1];
    casadi_real w[df_dx_SZ_W];
    df_dx(arg, res, iw, w, 0);
    return result;
}

inline Eigen::Matrix<double, STATE_LEN, PROC_NOISE_DOF> df_dw(state &x, const input &u) {
    const casadi_real *arg[df_dw_SZ_ARG] = {nullptr};
    const auto rot_mat = x.rot.toRotationMatrix();
    arg[0] = rot_mat.data();
    arg[1] = u.ang_vel.data();
    arg[2] = zero3d.data();

    casadi_real *res[df_dw_SZ_RES] = {nullptr};
    Eigen::Matrix<double, STATE_LEN, PROC_NOISE_DOF> result;
    res[0] = result.data();

    casadi_int iw[df_dw_SZ_IW];
    casadi_real w[df_dw_SZ_W];
    df_dw(arg, res, iw, w, 0);
    return result;
}

// measurement
inline measurement h(state &x, bool &is_valid) {
    const casadi_real *arg[h_SZ_ARG] = {nullptr};
    const auto rot_mat = x.rot.toRotationMatrix();
    arg[0] = rot_mat.data();
    arg[1] = zero3d.data();

    casadi_real *res[h_SZ_RES] = {nullptr};
    measurement result;
    res[0] = result.lin_acc.data();

    casadi_int iw[h_SZ_IW + 1];
    casadi_real w[h_SZ_W];
    h(arg, res, iw, w, 0);
    is_valid = true;
    return result;
}

inline Eigen::Matrix<double, MEAS_DOF, STATE_DOF> dh_dx(state &x, bool &is_valid) {
    const casadi_real *arg[dh_dx_SZ_ARG] = {nullptr};
    const auto rot_mat = x.rot.toRotationMatrix();
    arg[0] = rot_mat.data();
    arg[1] = zero3d.data();

    casadi_real *res[dh_dx_SZ_RES] = {nullptr};
    Eigen::Matrix<double, MEAS_DOF, STATE_DOF> result;
    res[0] = result.data();

    casadi_int iw[dh_dx_SZ_IW];
    casadi_real w[dh_dx_SZ_W];
    dh_dx(arg, res, iw, w, 0);
    is_valid = true;
    return result;
}

inline Eigen::Matrix<double, MEAS_DOF, MEAS_NOISE_DOF> dh_dv(state &x, bool &is_valid) {
    const casadi_real *arg[dh_dv_SZ_ARG] = {nullptr};
    const auto rot_mat = x.rot.toRotationMatrix();
    arg[0] = rot_mat.data();
    arg[1] = zero3d.data();

    casadi_real *res[dh_dv_SZ_RES] = {nullptr};
    Eigen::Matrix<double, MEAS_DOF, MEAS_NOISE_DOF> result;
    res[0] = result.data();

    casadi_int iw[dh_dv_SZ_IW];
    casadi_real w[dh_dv_SZ_W];
    dh_dv(arg, res, iw, w, 0);
    is_valid = true;
    return result;
}

// process noise covariance (Q) matrix
inline MTK::get_cov<process_noise>::type mat_Q(const double var_ang_vel) {
    MTK::get_cov<process_noise>::type mat = MTK::get_cov<process_noise>::type::Zero();
    MTK::setDiagonal<process_noise, vect3, 0>(mat, &process_noise::ang_vel, var_ang_vel);
    return mat;
}

// measurement noise covariance (R) matrix
inline MTK::get_cov<measurement_noise>::type mat_R(const double var_lin_acc) {
    MTK::get_cov<measurement_noise>::type mat = MTK::get_cov<measurement_noise>::type::Zero();
    MTK::setDiagonal<measurement_noise, vect3, 0>(mat, &measurement_noise::lin_acc, var_lin_acc);
    return mat;
}
