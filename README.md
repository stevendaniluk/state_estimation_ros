# State Estimation ROS

This is a ROS support library for the [state_estimation](https://github.com/stevendaniluk/state_estimation) library. To compile this you will need to build the `state_estimation` library as a catkin package, see the `state_estimation` readme for instructions.

This provides functionality for converting to and from ROS messages and the state representations used in the `state_estimation` library.

`ros_conversions.h` contains an assortment of functions for converting `Eigen` vectors/matrices to/from common ROS messages. These conversions operate on "message maps", which define the relationship between a ROS container and the state vector indices.

For example, given the state representation:
```
[X, Y, Vx, Vy, theta, w]
```

One would define a message map
```c
const std::array<int, 6> TWIST_MSG_MAP = {2, 3, -1, -1, -1, 5};
```

And convert an `Eigen` state vector to a `Twist` object with:
```c
Eigen::VectorXd x;
geometry_msgs::Twist twist_msg;
toMsg(x, TWIST_MSG_MAP, &twist_msg);
```
