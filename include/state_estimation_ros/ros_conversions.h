#pragma once

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64MultiArray.h>
#include <Eigen/Core>

namespace state_estimation {

// All conversion methods consume a mapping which maps FROM a ros message TO a
// state/control/measurement vector. The map size will always be equal to the number of fields in
// the ros message to populate.
//
// Any entries in the map with a value of -1 mean that field of the ROS message is not present in
// the provided vector.
//
// Example: The map for a state vector of [x, y, z, vx, vy, vz] to a Twist message would be
// [3, 4, 5, -1, -1, -1]. The first three elements map the linear velocity values, the last three
// elements map the angular velocity (which are not present in the original state vector).

////////////////////////////////////

// toMsg
//
// @param x: State/control/measurement vector to convert
// @param map: x indices for the point x, y, and z values (in that order)
// @param msg: Point message populated from x
void toMsg(const Eigen::VectorXd& x, const std::array<int, 3>& map, geometry_msgs::Point* msg);

// toMsg
//
// @param x: State/control/measurement vector to convert
// @param map: x indices for the vector x, y, and z values (in that order)
// @param msg: Vector3 message populated from x
void toMsg(const Eigen::VectorXd& x, const std::array<int, 3>& map, geometry_msgs::Vector3* msg);

// toMsg
//
// @param x: State/control/measurement vector to convert
// @param map: x indices for the Quaternion x, y, z, and w values (in that order)
// @param msg: Quaternion message populated from x
void toMsg(const Eigen::VectorXd& x, const std::array<int, 4>& map, geometry_msgs::Quaternion* msg);

// toMsg
//
// Special version that populates a quaternion from fixed axis roll, pitch, and yaw angles. This
// requires all entries in the map to ve valid (>= 0).
//
// @param x: State/control/measurement vector to convert
// @param map: x indices for the roll, pitch, and yaw (in that order)
// @param msg: Quaternion message populated from x
void toMsg(const Eigen::VectorXd& x, const std::array<int, 3>& map, geometry_msgs::Quaternion* msg);

// toMsg
//
// @param x: State/control/measurement vector to convert
// @param map: x indices for the Pose's x, y, z position values and x, y, z, w quaternion values
//             (in that order)
// @param msg: Pose message populated from x
void toMsg(const Eigen::VectorXd& x, const std::array<int, 7>& map, geometry_msgs::Pose* msg);

// toMsg
//
// Special version that populates a Pose from fixed axis roll, pitch, and yaw angles. This requires
// all entries in the map to ve valid (>= 0).
//
// @param x: State/control/measurement vector to convert
// @param map: x indices for the Pose x, y, z position, and roll, pitch, and yaw angles (in that
//             order)
// @param msg: Pose message populated from x
void toMsg(const Eigen::VectorXd& x, const std::array<int, 6>& map, geometry_msgs::Pose* msg);

// toMsg
//
// @param x: State/control/measurement vector to convert
// @param map: x indices for the Twist's x, y, z linear velocity values and x, y, z angular velocity
//             values (in that order)
// @param msg: Pose message populated from x
void toMsg(const Eigen::VectorXd& x, const std::array<int, 6>& map, geometry_msgs::Twist* msg);

// toMsg
//
// Special version that populates a quaternion from fixed axis roll, pitch, and yaw angles.
//
// @param x: State/control/measurement vector to convert
// @param map: x indices for the angular x, y, and z velocities and linear x, y, and z
//             accelerations (in that order)
// @param msg: Imu message populated from x
void toMsg(const Eigen::VectorXd& x, const std::array<int, 6>& map, sensor_msgs::Imu* msg);

// toMsg
//
// @param timestamp: Timestampe for the state
// @param x: State vector
// @param cov: State covariance
// @param msg: Array message populated with [timestamp, x, cov]
void toMsg(double timestamp, const Eigen::VectorXd& x, const Eigen::MatrixXd& cov,
           std_msgs::Float64MultiArray* msg);

////////////////////////////////////

// fromMsg
//
// @param msg: Point message to convert
// @param map: Output vector indices for the point x, y, and z values (in that order)
// @param x: State/control/measurement vector populated from msg
void fromMsg(const geometry_msgs::Point& msg, const std::array<int, 3>& map, Eigen::VectorXd* x);

// fromMsg
//
// @param msg: Vector3 message to convert
// @param map: Output vector indices for the vector x, y, and z values (in that order)
// @param x: State/control/measurement vector populated from msg
void fromMsg(const geometry_msgs::Vector3& msg, const std::array<int, 3>& map, Eigen::VectorXd* x);

// fromMsg
//
// @param msg: Quaternion message to convert
// @param map: Output vector indices for the quaternion x, y, z, and w values (in that order)
// @param x: State/control/measurement vector populated from msg
void fromMsg(const geometry_msgs::Quaternion& msg, const std::array<int, 4>& map,
             Eigen::VectorXd* x);

// fromMsg
//
// Special version that creates fixed axis roll, pitch, and yaw angles from a quaternion message.
// This Requires all entries in the map to ve valid (>= 0).
//
// @param msg: Quaternion message to convert
// @param map: Output vector indices for roll, pitch, and yaw angles (in that order)
// @param x: State/control/measurement vector populated from msg
void fromMsg(const geometry_msgs::Quaternion& msg, const std::array<int, 3>& map,
             Eigen::VectorXd* x);

// fromMsg
//
// @param msg: Pose message to convert
// @param map: Output vector indices for the position x, y, and z values, and quaternion x, y, z,
//             and w values (in that order)
// @param x: State/control/measurement vector populated from msg
void fromMsg(const geometry_msgs::Pose& msg, const std::array<int, 7>& map, Eigen::VectorXd* x);

// fromMsg
//
// Special version that creates pose data with fixed axis roll, pitch, and yaw angles from a Pose
// message. This Requires all entries in the map to ve valid (>= 0).
//
// @param msg: Pose message to convert
// @param map: Output vector indices for position x, y, and z, and orientation roll, pitch, and yaw
//             angles (in that order)
// @param x: State/control/measurement vector populated from msg
void fromMsg(const geometry_msgs::Pose& msg, const std::array<int, 6>& map, Eigen::VectorXd* x);

// fromMsg
//
// @param msg: Twist message to convert
// @param map: Output vector indices for the linear x, y, and z velocity values, and angular
//             velocity x, y, and z values (in that order)
// @param x: State/control/measurement vector populated from msg
void fromMsg(const geometry_msgs::Twist& msg, const std::array<int, 6>& map, Eigen::VectorXd* x);

// fromMsg
//
// Special version that creates imu data while ignoring orientation in the imu message.
//
// @param msg: Imu message to convert
// @param map: Output vector indices for angular x, y, and z velocities, and linear x, y, and z
//             accelerations (in that order)
// @param x: State/control/measurement vector populated from msg
void fromMsg(const sensor_msgs::Imu& msg, const std::array<int, 6>& map, Eigen::VectorXd* x);

// covarianceMatrixToVector
//
// Example: For the state vector [X, Vx, Y, Vy, Psi, Vpsi], coverting to a Twist message covariance
// (which has the order [Vx, Vy, Vz, Vphi, Vtheta, Vpsi]), the map {1, 3, -1, -1, -1, 5} would be
// provided, and Twist covariance with only the Vx, Vy, and Vpsi terms (and their cross terms)
// populated would be produced.
//
// Note, this will not modify any entries in the output covariance that are not indicated in the
// mapping.
//
// @param cov_in: Covariance matrix to convert
// @param map: cov indices for the output covariance entries with -1 indicating that field is not
//             present (see the example above)
// @param cov_out: Covariance in vector format populated from cov
template <uint32_t N>
void covarianceMatrixToVector(const Eigen::MatrixXd& cov_in, const std::array<int, N>& map,
                              boost::array<double, N * N>* cov_out);

// covarianceVectorToMatrix
//
// Example: For the state vector [X, Vx, Y, Vy, Psi, Vpsi], coverting from a Twist message
// covariance (which has the order [Vx, Vy, Vz, Vphi, Vtheta, Vpsi]), the map {1, 3, -1, -1, -1, 5}
// would be provided, and the covariance matrix would be populated with the Vx, Vy, and Vpsi values
// from the Twist covariance vector.
//
// Note, this will not modify any entries in the output covariance that are not indicated in the
// mapping. This also assumes the output covariance matrix is already of the correct size, since
// we cannot know what it's actually size should be.
//
// @param cov_in: Covariance in vector format to convert
// @param map: cov indices for the output covariance entries, with -1 indicating that field is not
//             present (see the example above)
// @param cov_out: Covariance populated from cov
template <uint32_t N>
void covarianceVectorToMatrix(const boost::array<double, N * N>& cov_in,
                              const std::array<int, N>& map, Eigen::MatrixXd* cov_out);

}  // namespace state_estimation

#include "ros_conversions.hpp"
