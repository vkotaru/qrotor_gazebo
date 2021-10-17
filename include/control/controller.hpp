#ifndef QROTOR_GAZEBO_CONTROLLER_HPP
#define QROTOR_GAZEBO_CONTROLLER_HPP

#include <eigen3/Eigen/Dense>

namespace qrotor_gazebo {

class SO3Controller {
private:
  Eigen::Vector3d kp_;
  Eigen::Vector3d kd_;

public:
  SO3Controller() = default;
  ~SO3Controller() = default;

  void init();
  void run();

  Eigen::Vector3d moment() { return Eigen::Vector3d::Zero(); }
};

class PositionController {
private:
  Eigen::Vector3d kp_;
  Eigen::Vector3d kd_;
  Eigen::Vector3d ki_;

public:
  PositionController() = default;
  ~PositionController() = default;

  void init();
  void run();
};

} // namespace qrotor_gazebo

#endif // QROTOR_GAZEBO_CONTROLLER_HPP