#ifndef QROTOR_GAZEBO_CONTROL_TYPES_HPP
#define QROTOR_GAZEBO_CONTROL_TYPES_HPP
#include <eigen3/Eigen/Dense>

namespace qrotor_gazebo {

static const Eigen::Vector3d gravity{0., 0., 1.};

template <typename T> struct FlatVariable {
  T yaw;
  T yaw_rate;
  T dyaw_rate;
  Eigen::Matrix<T, 3, 1> position;
  Eigen::Matrix<T, 3, 1> velocity;
  Eigen::Matrix<T, 3, 1> accel;
  Eigen::Matrix<T, 3, 1> daccel;
  Eigen::Matrix<T, 3, 1> d2accel;
};

using FlatVariabled = FlatVariable<double>;
using FlatVariablef = FlatVariable<float>;

} // namespace qrotor_gazebo

#endif // QROTOR_GAZEBO_CONTROL_TYPES_HPP
