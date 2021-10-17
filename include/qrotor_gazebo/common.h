/**
 * @file common.h
 * @author kotaru 
 * @date 10/16/21.
*/

#ifndef QROTOR_GAZEBO_COMMON_H
#define QROTOR_GAZEBO_COMMON_H

#include <eigen3/Eigen/Dense>

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/rendering/rendering.hh>
#include <gazebo/sensors/CameraSensor.hh>
#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/sensors/sensors.hh>
#include <ros/ros.h>
#include <gazebo/gazebo.hh>

using GazeboVector = ignition::math::Vector3d;
using GazeboPose = ignition::math::Pose3d;
using GazeboQuaternion = ignition::math::Quaterniond;

namespace qrotor_gazebo {

static Eigen::Vector3d E1{1., 0., 0.};
static Eigen::Vector3d E2{0., 1., 0.};
static Eigen::Vector3d E3{0., 0., 1.};

inline Eigen::Vector3d vec3_to_eigen_from_gazebo(const GazeboVector& vec) {
  return (Eigen::Vector3d() << vec.X(), vec.Y(), vec.Z()).finished();
}

inline GazeboVector vec3_to_gazebo_from_eigen(const Eigen::Vector3d& vec) {
  return GazeboVector(vec(0), vec(1), vec(2));
}

inline Eigen::Matrix3d rotation_to_eigen_from_gazebo(const GazeboQuaternion& quat) {
  return Eigen::Quaternion(quat.W(), quat.X(), quat.Y(), quat.Z()).toRotationMatrix();
}

/**
 * @brief vee map
 * @param[in] input matrix
 * @return
 */
inline Eigen::Vector3d vee3d(Eigen::Matrix3d M) {
  return (Eigen::Vector3d() << -M(1, 2), M(0, 2), -M(0, 1)).finished();
}

/**
 * @brief hat map
 * @param[in] input vector
 * @return
 */
Eigen::Matrix3d hat3d(Eigen::Vector3d v) {
  return (Eigen::Matrix3d() << 0.0, -v(2), v(1), v(2), 0.0, -v(0), -v(1), v(0), 0.0).finished();
}

struct Pose3D {
public:
  Pose3D() = default;
  explicit Pose3D(const gazebo::physics::LinkPtr& link_) {
    gpose = link_->WorldCoGPose();
    gvel = link_->RelativeLinearVel();
    gomega = link_->RelativeAngularVel();

    if ((abs(gpose.Pos().X()) > 100) || (abs(gpose.Pos().Y()) > 100) ||
        (abs(gpose.Pos().Z()) > 100)) {
      gzerr << "bad pose reading" << std::endl;
    }

    pos = vec3_to_eigen_from_gazebo(gpose.Pos());
    rot = rotation_to_eigen_from_gazebo(gpose.Rot());
    vel = vec3_to_eigen_from_gazebo(gvel);
    omega = vec3_to_eigen_from_gazebo(gomega);
  }
  GazeboPose gpose{};
  GazeboVector gvel{};
  GazeboVector gomega{};

  Eigen::Vector3d pos{};
  Eigen::Matrix3d rot{};
  Eigen::Vector3d vel{};
  Eigen::Vector3d omega{};
  double t{0};
};

} // namespace qrotor_gazebo

#endif //QROTOR_GAZEBO_COMMON_H
