#include <state_estimation_ros/ros_conversions.h>
#include <Eigen/Geometry>

namespace state_estimation {

void toMsg(const Eigen::VectorXd& x, const std::array<int, 3>& map, geometry_msgs::Point* msg) {
    if (map[0] >= 0) {
        msg->x = x(map[0]);
    }
    if (map[1] >= 0) {
        msg->y = x(map[1]);
    }
    if (map[2] >= 0) {
        msg->z = x(map[2]);
    }
}

void toMsg(const Eigen::VectorXd& x, const std::array<int, 3>& map, geometry_msgs::Vector3* msg) {
    if (map[0] >= 0) {
        msg->x = x(map[0]);
    }
    if (map[1] >= 0) {
        msg->y = x(map[1]);
    }
    if (map[2] >= 0) {
        msg->z = x(map[2]);
    }
}

void toMsg(const Eigen::VectorXd& x, const std::array<int, 4>& map,
           geometry_msgs::Quaternion* msg) {
    if (map[0] >= 0) {
        msg->x = x(map[0]);
    }
    if (map[1] >= 0) {
        msg->y = x(map[1]);
    }
    if (map[2] >= 0) {
        msg->z = x(map[2]);
    }
    if (map[3] >= 0) {
        msg->w = x(map[3]);
    }
}

void toMsg(const Eigen::VectorXd& x, const std::array<int, 3>& map,
           geometry_msgs::Quaternion* msg) {
    const double phi = map[0] >= 0 ? x(map[0]) : 0.0;
    const double theta = map[1] >= 0 ? x(map[1]) : 0.0;
    const double psi = map[2] >= 0 ? x(map[2]) : 0.0;

    const Eigen::Quaterniond quat =
        (Eigen::Quaterniond)Eigen::AngleAxisd(psi, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(phi, Eigen::Vector3d::UnitX());

    msg->x = quat.x();
    msg->y = quat.y();
    msg->z = quat.z();
    msg->w = quat.w();
}

void toMsg(const Eigen::VectorXd& x, const std::array<int, 7>& map, geometry_msgs::Pose* msg) {
    toMsg(x, {map[0], map[1], map[2]}, &(msg->position));
    toMsg(x, {map[3], map[4], map[5], map[6]}, &(msg->orientation));
}

void toMsg(const Eigen::VectorXd& x, const std::array<int, 6>& map, geometry_msgs::Pose* msg) {
    toMsg(x, {map[0], map[1], map[2]}, &(msg->position));

    std::array<int, 3> rpy_map = {map[3], map[4], map[5]};
    toMsg(x, rpy_map, &(msg->orientation));
}

void toMsg(const Eigen::VectorXd& x, const std::array<int, 6>& map, geometry_msgs::Twist* msg) {
    toMsg(x, {map[0], map[1], map[2]}, &(msg->linear));
    toMsg(x, {map[3], map[4], map[5]}, &(msg->angular));
}

void toMsg(const Eigen::VectorXd& x, const std::array<int, 6>& map, sensor_msgs::Imu* msg) {
    toMsg(x, {map[0], map[1], map[2]}, &(msg->angular_velocity));
    toMsg(x, {map[3], map[4], map[5]}, &(msg->linear_acceleration));
}

void toMsg(double timestamp, const Eigen::VectorXd& x, const Eigen::MatrixXd& cov,
           std_msgs::Float64MultiArray* msg) {
    msg->layout.data_offset = 0;
    msg->layout.dim.resize(1);
    msg->layout.dim[0].label = "data";
    msg->layout.dim[0].size = 1 + x.size() + cov.rows() * cov.cols();
    msg->data.resize(msg->layout.dim[0].size);

    msg->data[0] = timestamp;

    for (int i = 0; i < x.size(); ++i) {
        msg->data[1 + i] = x(i);
    }

    for (int i = 0; i < cov.rows(); ++i) {
        for (int j = 0; j < cov.cols(); ++j) {
            uint32_t offset = i * cov.rows() + j;
            msg->data[1 + x.size() + offset] = cov(i, j);
        }
    }
}

void fromMsg(const geometry_msgs::Point& msg, const std::array<int, 3>& map, Eigen::VectorXd* x) {
    if (map[0] >= 0) {
        (*x)(map[0]) = msg.x;
    }
    if (map[1] >= 0) {
        (*x)(map[1]) = msg.y;
    }
    if (map[2] >= 0) {
        (*x)(map[2]) = msg.z;
    }
}

void fromMsg(const geometry_msgs::Vector3& msg, const std::array<int, 3>& map, Eigen::VectorXd* x) {
    if (map[0] >= 0) {
        (*x)(map[0]) = msg.x;
    }
    if (map[1] >= 0) {
        (*x)(map[1]) = msg.y;
    }
    if (map[2] >= 0) {
        (*x)(map[2]) = msg.z;
    }
}

void fromMsg(const geometry_msgs::Quaternion& msg, const std::array<int, 4>& map,
             Eigen::VectorXd* x) {
    if (map[0] >= 0) {
        (*x)(map[0]) = msg.x;
    }
    if (map[1] >= 0) {
        (*x)(map[1]) = msg.y;
    }
    if (map[2] >= 0) {
        (*x)(map[2]) = msg.z;
    }
    if (map[3] >= 0) {
        (*x)(map[3]) = msg.w;
    }
}

void fromMsg(const geometry_msgs::Quaternion& msg, const std::array<int, 3>& map,
             Eigen::VectorXd* x) {
    const Eigen::Quaterniond quat(msg.w, msg.x, msg.y, msg.z);
    Eigen::Vector3d rpy = quat.toRotationMatrix().eulerAngles(2, 1, 0);
    (*x)(map[0]) = rpy(2);
    (*x)(map[1]) = rpy(1);
    (*x)(map[2]) = rpy(0);
}

void fromMsg(const geometry_msgs::Pose& msg, const std::array<int, 7>& map, Eigen::VectorXd* x) {
    fromMsg(msg.position, {map[0], map[1], map[2]}, x);
    fromMsg(msg.orientation, {map[3], map[4], map[5], map[6]}, x);
}

void fromMsg(const geometry_msgs::Pose& msg, const std::array<int, 6>& map, Eigen::VectorXd* x) {
    fromMsg(msg.position, {map[0], map[1], map[2]}, x);

    std::array<int, 3> rpy_map = {map[3], map[4], map[5]};
    fromMsg(msg.orientation, rpy_map, x);
}

void fromMsg(const geometry_msgs::Twist& msg, const std::array<int, 6>& map, Eigen::VectorXd* x) {
    fromMsg(msg.linear, {map[0], map[1], map[2]}, x);
    fromMsg(msg.angular, {map[3], map[4], map[5]}, x);
}

void fromMsg(const sensor_msgs::Imu& msg, const std::array<int, 6>& map, Eigen::VectorXd* x) {
    fromMsg(msg.angular_velocity, {map[0], map[1], map[2]}, x);
    fromMsg(msg.linear_acceleration, {map[3], map[4], map[5]}, x);
}

}  // namespace state_estimation
