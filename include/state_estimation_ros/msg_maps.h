#pragma once

#include <state_estimation/definitions/common_measurements.h>
#include <state_estimation/definitions/planer_2d_motion.h>
#include <array>

namespace state_estimation {

////////////////////////////////////
// Planer 2D state maps

const std::array<int, 6> PLANER_2D_POSE_MSG_MAP = {
    planer_2d::state::X, planer_2d::state::Y, -1, -1, -1, planer_2d::state::PSI};

const std::array<int, 3> PLANER_2D_LINEAR_VEL_MSG_MAP = {planer_2d::state::VX, planer_2d::state::VY,
                                                         -1};
const std::array<int, 3> PLANER_2D_ANGULAR_VEL_MSG_MAP = {-1, -1, planer_2d::state::VPSI};

const std::array<int, 6> PLANER_2D_TWIST_MSG_MAP = {
    planer_2d::state::VX, planer_2d::state::VY, -1, -1, -1, planer_2d::state::VPSI};

////////////////////////////////////
// Accel, Gyro, and IMU measurement maps

const std::array<int, 3> GYRO_MEAS_ANGULAR_VEL_MSG_MAP = {meas::gyro::VPHI, meas::gyro::VTHETA,
                                                          meas::gyro::VPSI};

const std::array<int, 3> ACCEL_MEAS_LINEAR_ACCEL_MSG_MAP = {meas::accel::AX, meas::accel::AY,
                                                            meas::accel::AZ};

const std::array<int, 3> IMU_MEAS_ANGULAR_VEL_MSG_MAP = {meas::imu::VPHI, meas::imu::VTHETA,
                                                         meas::imu::VPSI};
const std::array<int, 3> IMU_MEAS_LINEAR_ACCEL_MSG_MAP = {meas::imu::AX, meas::imu::AY,
                                                          meas::imu::AZ};
const std::array<int, 6> IMU_MEAS_IMU_MSG_MAP = {meas::imu::VPHI, meas::imu::VTHETA,
                                                 meas::imu::VPSI, meas::imu::AX,
                                                 meas::imu::AY,   meas::imu::AZ};

}  // namespace state_estimation
