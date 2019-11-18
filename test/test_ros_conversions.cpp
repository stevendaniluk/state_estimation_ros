#include <gtest/gtest.h>
#include <state_estimation_ros/ros_conversions.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <numeric>

using namespace state_estimation;

// Test fixture which contains an assortment of data vectors to feed through the conversion
// functions
class StateEstimationRosConversions : public ::testing::Test {
  protected:
    void SetUp() override {
        // Provide some dummy values for each data vector
        xyz_vec.resize(3);
        xyz_vec << 1.1, 2.2, 3.3;

        rpy_vec.resize(3);
        rpy_vec << 0.111, 0.222, 0.333;

        // Make the quaternion vector from the roll pitch yaw angles
        Eigen::Quaterniond quat =
            (Eigen::Quaterniond)Eigen::AngleAxisd(rpy_vec(2), Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(rpy_vec(1), Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(rpy_vec(0), Eigen::Vector3d::UnitX());

        quat_vec.resize(4);
        quat_vec << quat.x(), quat.y(), quat.z(), quat.w();

        pose_vec.resize(7);
        pose_vec << xyz_vec, quat_vec;

        pose_rpy_vec.resize(6);
        pose_rpy_vec << xyz_vec, rpy_vec;

        twist_vec.resize(6);
        twist_vec << 11.1, 22.2, 33.3, 44.4, 55.5, 66.6;

        imu_no_orient_vec.resize(6);
        imu_no_orient_vec << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6;

        // Populate our sample covariance data. This will model something like a linear
        // acceleration covariance message (like in an IMU message). We'll make our internal
        // covariance matrix have more states though.
        //
        // We'll assume the covariance vector is for the state variables [Ax, Ay, Az], and we'll
        // have a state vector of [Vx, Ax, Vy, Ay]
        //
        // We also want to populate some of the cross terms, so we know if those get modified.
        // Make all the common terms <1 so we can identify them.
        cov_map = {1, 3, -1};

        cov_mat.resize(4, 4);
        // clang-format off
        cov_mat <<
            1.0, 1.1, 1.2, 1.3,
            1.1, 0.4, 1.5, 0.6,
            1.2, 1.5, 1.7, 1.8,
            1.3, 0.6, 1.8, 0.9;

        cov_vec = {
            cov_mat(1, 1), cov_mat(1, 3),   2.1,
            cov_mat(1, 3), cov_mat(3, 3),   2.2,
                2.3,            2.4,        2.5
        };
        // clang-format on
    }

    // Data vectors for each type of ROS message being converted
    Eigen::VectorXd xyz_vec;
    Eigen::VectorXd rpy_vec;
    Eigen::VectorXd quat_vec;
    Eigen::VectorXd pose_vec;
    Eigen::VectorXd pose_rpy_vec;
    Eigen::VectorXd twist_vec;
    Eigen::VectorXd imu_no_orient_vec;

    // Sample covariance entries
    Eigen::MatrixXd cov_mat;
    boost::array<double, 9> cov_vec;
    std::array<int, 3> cov_map;
};

// randomizeVector
//
// Pads a data vector with two extra values and generates a new map for the indices.
//
// Example: Given x=[1.1, 2.2, 3.3], it will be shuffled to be [PAD, 3.3, PAD, 2.2, 1.1], with
// a new map of {4, 3, 1}, where PAD is a random number inserted.
//
// @param x: Data vector to randomize
// @param x_random: Shuffled version of x with two extra elements
// @param map: Map with indices for x_random for the values from x
template <uint32_t N>
void randomizeVector(const Eigen::VectorXd& x, Eigen::VectorXd* x_random, std::array<int, N>* map) {
    // Make a map with entries {0, 1, ..., N + 1}, shuffle it, then copy over the first N elements
    // so we have a map containing any values from 0 to N + 1
    std::array<int, N + 2> map_padded;
    std::iota(map_padded.begin(), map_padded.end(), 0);
    std::random_shuffle(map_padded.begin(), map_padded.end());
    for (int i = 0; i < N; ++i) {
        (*map)[i] = map_padded[i];
    }

    // Fill out randomized vector with the padded value, then copy over the original data using the
    // naw map indices
    x_random->resize(N + 2);
    double pad_number = 9999.987654321;
    for (int i = 0; i < x_random->size(); ++i) {
        (*x_random)(i) = pad_number;
    }
    for (int i = 0; i < N; ++i) {
        (*x_random)((*map)[i]) = x(i);
    }
}

// msgCheck
//
// Helper function to convert a data vector to a message and back, and check that the values match.
// This will only compare the entries defined in the map
template <typename MsgT, uint32_t N>
void msgCheck(const Eigen::VectorXd& x, const std::array<int, N>& map) {
    Eigen::VectorXd output(x.size());
    MsgT msg;

    toMsg(x, map, &msg);
    fromMsg(msg, map, &output);

    bool all_match = true;
    for (int index : map) {
        all_match &= fabs(x(index) - output(index)) < 1e-6;
    }
    EXPECT_TRUE(all_match) << "Target: " << x.transpose() << ", Actual: " << output.transpose();
}

// randomizedMsgCheck
//
// Version of msgCheck() that will randomize the data vector and map.
template <typename MsgT, uint32_t N>
void randomizedMsgCheck(const Eigen::VectorXd& x, const std::array<int, N>& map) {
    Eigen::VectorXd x_random;
    std::array<int, N> map_random;
    randomizeVector<N>(x, &x_random, &map_random);

    msgCheck<MsgT, N>(x_random, map_random);
}

TEST_F(StateEstimationRosConversions, Point) {
    msgCheck<geometry_msgs::Point, 3>(xyz_vec, {0, 1, 2});
}

TEST_F(StateEstimationRosConversions, PointMixedSizeAndOrder) {
    randomizedMsgCheck<geometry_msgs::Point, 3>(xyz_vec, {0, 1, 2});
}

TEST_F(StateEstimationRosConversions, Vector3) {
    msgCheck<geometry_msgs::Vector3, 3>(xyz_vec, {0, 1, 2});
}

TEST_F(StateEstimationRosConversions, Vector3MixedSizeAndOrder) {
    randomizedMsgCheck<geometry_msgs::Vector3, 3>(xyz_vec, {0, 1, 2});
}

TEST_F(StateEstimationRosConversions, Quaternion) {
    msgCheck<geometry_msgs::Quaternion, 4>(quat_vec, {0, 1, 2, 3});
}

TEST_F(StateEstimationRosConversions, QuaternionMixedSizeAndOrder) {
    randomizedMsgCheck<geometry_msgs::Quaternion, 4>(quat_vec, {0, 1, 2, 3});
}

TEST_F(StateEstimationRosConversions, QuaternionRPY) {
    msgCheck<geometry_msgs::Quaternion, 3>(rpy_vec, {0, 1, 2});
}

TEST_F(StateEstimationRosConversions, QuaternionRPYMixedSizeAndOrder) {
    randomizedMsgCheck<geometry_msgs::Quaternion, 3>(rpy_vec, {0, 1, 2});
}

TEST_F(StateEstimationRosConversions, Pose) {
    msgCheck<geometry_msgs::Pose, 7>(pose_vec, {0, 1, 2, 3, 4, 5, 6});
}

TEST_F(StateEstimationRosConversions, PoseMixedSizeAndOrder) {
    randomizedMsgCheck<geometry_msgs::Pose, 7>(pose_vec, {0, 1, 2, 3, 4, 5, 6});
}

TEST_F(StateEstimationRosConversions, PoseRPY) {
    msgCheck<geometry_msgs::Pose, 6>(pose_rpy_vec, {0, 1, 2, 3, 4, 5});
}

TEST_F(StateEstimationRosConversions, PoseRPYMixedSizeAndOrder) {
    randomizedMsgCheck<geometry_msgs::Pose, 6>(pose_rpy_vec, {0, 1, 2, 3, 4, 5});
}

TEST_F(StateEstimationRosConversions, Twist) {
    msgCheck<geometry_msgs::Twist, 6>(twist_vec, {0, 1, 2, 3, 4, 5});
}

TEST_F(StateEstimationRosConversions, TwistMixedSizeAndOrder) {
    randomizedMsgCheck<geometry_msgs::Twist, 6>(twist_vec, {0, 1, 2, 3, 4, 5});
}

TEST_F(StateEstimationRosConversions, ImuNoOrientation) {
    msgCheck<sensor_msgs::Imu, 6>(imu_no_orient_vec, {0, 1, 2, 3, 4, 5});
}

TEST_F(StateEstimationRosConversions, ImuNoOrientationMixedSizeAndOrder) {
    randomizedMsgCheck<sensor_msgs::Imu, 6>(imu_no_orient_vec, {0, 1, 2, 3, 4, 5});
}

TEST_F(StateEstimationRosConversions, Float64MultiArraySizeCorrect) {
    double timestamp = 12345.6789;
    Eigen::VectorXd x(2);
    Eigen::MatrixXd cov(2, 2);

    std_msgs::Float64MultiArray msg;
    toMsg(timestamp, x, cov, &msg);
    ASSERT_EQ(1, msg.layout.dim.size());
    EXPECT_EQ(7, msg.layout.dim[0].size);
    EXPECT_EQ(7, msg.data.size());
}

TEST_F(StateEstimationRosConversions, Float64MultiArrayContentsCorrect) {
    double timestamp = 12345.6789;

    Eigen::VectorXd x(2);
    x << 1.1, 2.2;

    Eigen::MatrixXd cov(2, 2);
    cov << 1, 2, 3, 4;

    std_msgs::Float64MultiArray msg;
    toMsg(timestamp, x, cov, &msg);

    ASSERT_EQ(7, msg.data.size());
    EXPECT_EQ(timestamp, msg.data[0]);
    EXPECT_EQ(x(0), msg.data[1]);
    EXPECT_EQ(x(1), msg.data[2]);
    EXPECT_EQ(cov(0, 0), msg.data[3]);
    EXPECT_EQ(cov(0, 1), msg.data[4]);
    EXPECT_EQ(cov(1, 0), msg.data[5]);
    EXPECT_EQ(cov(1, 1), msg.data[6]);
}

TEST_F(StateEstimationRosConversions, CovarianceMatrixToVector) {
    boost::array<double, 9> cov_converted;
    covarianceMatrixToVector<3>(cov_mat, cov_map, &cov_converted);

    // Only check the common terms on the converted covariance, so any entries in the ground truth
    // that are <1
    for (int i = 0; i < cov_vec.size(); ++i) {
        if (cov_vec[i] < 1) {
            EXPECT_EQ(cov_vec[i], cov_converted[i]) << "Index " << i;
        }
    }
}

TEST_F(StateEstimationRosConversions, CovarianceMatrixToVectorDoesNotAlterOtherTerms) {
    boost::array<double, 9> cov_converted = cov_vec;
    covarianceMatrixToVector<3>(cov_mat, cov_map, &cov_converted);

    // Check that the non common terms were left unchanged, so any entries in the ground truth that
    // are >= 1
    for (int i = 0; i < cov_vec.size(); ++i) {
        if (cov_vec[i] >= 1) {
            EXPECT_EQ(cov_vec[i], cov_converted[i]) << "Index " << i;
        }
    }
}

TEST_F(StateEstimationRosConversions, CovarianceVectorToMatrix) {
    Eigen::MatrixXd cov_converted(cov_mat.rows(), cov_mat.cols());
    covarianceVectorToMatrix<3>(cov_vec, cov_map, &cov_converted);

    // Only check the common terms on the converted covariance, so any entries in the ground truth
    // that are <1
    for (int i = 0; i < cov_converted.rows(); ++i) {
        for (int j = 0; j < cov_converted.cols(); ++j) {
            if (cov_mat(i, j) < 1) {
                EXPECT_EQ(cov_mat(i, j), cov_converted(i, j)) << "Index [" << i << ", " << j << "]";
            }
        }
    }
}

TEST_F(StateEstimationRosConversions, CovarianceVectorToMatrixDoesNotAlterOtherTerms) {
    Eigen::MatrixXd cov_converted = cov_mat;
    covarianceVectorToMatrix<3>(cov_vec, cov_map, &cov_converted);

    // Only check the common terms on the converted covariance, so any entries in the ground truth
    // that are <1
    for (int i = 0; i < cov_converted.rows(); ++i) {
        for (int j = 0; j < cov_converted.cols(); ++j) {
            if (cov_mat(i, j) >= 1) {
                EXPECT_EQ(cov_mat(i, j), cov_converted(i, j)) << "Index [" << i << ", " << j << "]";
            }
        }
    }
}
