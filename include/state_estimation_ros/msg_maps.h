#pragma once

#include <state_estimation/definitions/common_measurements.h>
#include <state_estimation/definitions/planar_2d_motion.h>
#include <state_estimation/definitions/six_d_rates.h>
#include <array>

namespace state_estimation {

////////////////////////////////////
// Planer 2D state maps

const std::array<int, 6> PLANER_2D_POSE_MSG_MAP = {
    planar_2d::state::X, planar_2d::state::Y, -1, -1, -1, planar_2d::state::PSI};

const std::array<int, 3> PLANER_2D_LINEAR_VEL_MSG_MAP = {planar_2d::state::VX, planar_2d::state::VY,
                                                         -1};
const std::array<int, 3> PLANER_2D_ANGULAR_VEL_MSG_MAP = {-1, -1, planar_2d::state::VPSI};

const std::array<int, 6> PLANER_2D_TWIST_MSG_MAP = {
    planar_2d::state::VX, planar_2d::state::VY, -1, -1, -1, planar_2d::state::VPSI};

////////////////////////////////////
// Six D Rates state maps

const std::array<int, 3> SIX_D_RATES_LINEAR_VEL_MSG_MAP = {
    six_d_rates::state::VX, six_d_rates::state::VY, six_d_rates::state::VZ};
const std::array<int, 3> SIX_D_RATES_ANGULAR_VEL_MSG_MAP = {
    six_d_rates::state::VPHI, six_d_rates::state::VTHETA, six_d_rates::state::VPSI};

const std::array<int, 6> SIX_D_RATES_TWIST_MSG_MAP = {
    six_d_rates::state::VX,   six_d_rates::state::VY,     six_d_rates::state::VZ,
    six_d_rates::state::VPHI, six_d_rates::state::VTHETA, six_d_rates::state::VPSI};

const std::array<int, 3> SIX_D_RATES_LINEAR_ACCEL_MSG_MAP = {
    six_d_rates::state::AX, six_d_rates::state::AY, six_d_rates::state::AZ};
const std::array<int, 3> SIX_D_RATES_GRAVITY_MSG_MAP = {
    six_d_rates::state::GX, six_d_rates::state::GY, six_d_rates::state::GZ};
const std::array<int, 3> SIX_D_RATES_ACCEL_BIAS_MSG_MAP = {
    six_d_rates::state::B_AX, six_d_rates::state::B_AY, six_d_rates::state::B_AZ};
const std::array<int, 3> SIX_D_RATES_GYRO_BIAS_MSG_MAP = {
    six_d_rates::state::B_WX, six_d_rates::state::B_WY, six_d_rates::state::B_WZ};

////////////////////////////////////
// Common Measurement Maps

// Odom measurement maps
const std::array<int, 3> ODOM_MEAS_ANGULAR_VEL_MSG_MAP = {-1, -1, meas::odom::VPSI};
const std::array<int, 3> ODOM_MEAS_LINEAR_VEL_MSG_MAP = {meas::odom::VX, meas::odom::VY,
                                                         meas::odom::VZ};
const std::array<int, 6> ODOM_MEAS_TWIST_MSG_MAP = {
    meas::odom::VX, meas::odom::VY, meas::odom::VZ, -1, -1, meas::odom::VZ};

// Gyroscope measurement maps
const std::array<int, 3> GYRO_MEAS_ANGULAR_VEL_MSG_MAP = {meas::gyro::VPHI, meas::gyro::VTHETA,
                                                          meas::gyro::VPSI};

// Accelerometer measurement maps
const std::array<int, 3> ACCEL_MEAS_LINEAR_ACCEL_MSG_MAP = {meas::accel::AX, meas::accel::AY,
                                                            meas::accel::AZ};

// IMU (accel + gyro) measurement maps
const std::array<int, 3> IMU_MEAS_ANGULAR_VEL_MSG_MAP = {meas::imu::VPHI, meas::imu::VTHETA,
                                                         meas::imu::VPSI};
const std::array<int, 3> IMU_MEAS_LINEAR_ACCEL_MSG_MAP = {meas::imu::AX, meas::imu::AY,
                                                          meas::imu::AZ};
const std::array<int, 6> IMU_MEAS_IMU_MSG_MAP = {meas::imu::VPHI, meas::imu::VTHETA,
                                                 meas::imu::VPSI, meas::imu::AX,
                                                 meas::imu::AY,   meas::imu::AZ};

// IMU with quaternion orientation measurement maps
const std::array<int, 4> IMU_QUAT_MEAS_ORIENTATION_MSG_MAP = {meas::imu_quat::X, meas::imu_quat::Y,
                                                              meas::imu_quat::Z, meas::imu_quat::W};
const std::array<int, 3> IMU_QUAT_MEAS_ANGULAR_VEL_MSG_MAP = {
    meas::imu_quat::VPHI, meas::imu_quat::VTHETA, meas::imu_quat::VPSI};
const std::array<int, 3> IMU_QUAT_MEAS_LINEAR_ACCEL_MSG_MAP = {
    meas::imu_quat::AX, meas::imu_quat::AY, meas::imu_quat::AZ};
const std::array<int, 6> IMU_QUAT_MEAS_IMU_MSG_MAP = {meas::imu_quat::VPHI, meas::imu_quat::VTHETA,
                                                      meas::imu_quat::VPSI, meas::imu_quat::AX,
                                                      meas::imu_quat::AY,   meas::imu_quat::AZ};

// IMU with RPY orientation measurement maps
const std::array<int, 3> IMU_RPY_MEAS_ORIENTATION_MSG_MAP = {
    meas::imu_rpy::PHI, meas::imu_rpy::THETA, meas::imu_rpy::PSI};
const std::array<int, 3> IMU_RPY_MEAS_ANGULAR_VEL_MSG_MAP = {
    meas::imu_rpy::VPHI, meas::imu_rpy::VTHETA, meas::imu_rpy::VPSI};
const std::array<int, 3> IMU_RPY_MEAS_LINEAR_ACCEL_MSG_MAP = {meas::imu_rpy::AX, meas::imu_rpy::AY,
                                                              meas::imu_rpy::AZ};
const std::array<int, 6> IMU_RPY_MEAS_IMU_MSG_MAP = {meas::imu_rpy::VPHI, meas::imu_rpy::VTHETA,
                                                     meas::imu_rpy::VPSI, meas::imu_rpy::AX,
                                                     meas::imu_rpy::AY,   meas::imu_rpy::AZ};

}  // namespace state_estimation
