#ifndef QROTOR_GAZEBO_CONTROL_H
#define QROTOR_GAZEBO_CONTROL_H

#include "control/controllers.hpp"

namespace qrotor_gazebo {

class QrotorControl {
public:
  enum Mode {
    PASS_THROUGH,
    ATTITUDE,
    ATTITUDE_RATE,
    THRUST_YAW,
    THRUST_YAW_RATE,
    POSITION,
    POSITION_SPLINE
  };

  struct RigidBodyState {
    RigidBodyState(Eigen::Vector3d x, Eigen::Vector3d v, Eigen::Matrix3d R,
                   Eigen::Vector3d Om)
        : position(x), velocity(v), rotation(R), ang_vel(Om) {}
    RigidBodyState()
        : position(Eigen::Vector3d::Zero()), velocity(Eigen::Vector3d::Zero()),
          rotation(Eigen::Matrix3d::Identity()),
          ang_vel(Eigen::Vector3d::Zero()) {}
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    Eigen::Matrix3d rotation;
    Eigen::Vector3d ang_vel;
  };

private:
  Mode mode_{PASS_THROUGH};
  Eigen::Vector3d thrust_vector_{0., 0., 0.};
  Eigen::Vector3d moment_{0., 0., 0.};
  RigidBodyState state_{};

public:
  QrotorControl() {}
  ~QrotorControl() {}

  PositionController posCtrl_{};
  SO3Controller attCtrl_{};

  void updateParams(const double &mass, const Eigen::Matrix3d &inertia) {
    posCtrl_.updateParams(mass);
    attCtrl_.updateParams(inertia);
  }

  FlatVariabled flats_{};

  void setMode(const int &mode) {
    switch (mode) {
    case 0: mode_ == PASS_THROUGH; break;
    case 1: mode_ = ATTITUDE; break;
    case 2: mode_ = ATTITUDE_RATE; break;
    case 3: mode_ = THRUST_YAW; break;
    case 4: mode_ = THRUST_YAW_RATE; break;
    case 5: mode_ = POSITION; break;
    case 6: mode_ = POSITION_SPLINE; break;
    default: break;
    }
  }
  const Mode &mode() const { return mode_; }

  void computeAttitudeInput(double dt) {
    moment_ = attCtrl_.run(dt, state_.rotation, state_.ang_vel);
  }

  void computePositionInput(double dt) {
    thrust_vector_ = posCtrl_.run(dt, state_.position, state_.velocity);
    attCtrl_.updateCommand(thrust_vector_);
  }

  void skipComputingPosInput(const Eigen::Vector3d &tv) {
    thrust_vector_ = tv;
    attCtrl_.updateCommand(thrust_vector_);
  }

  double thrust() const {
    return thrust_vector_(2);
  }

  Eigen::Vector3d moment() const { return moment_; }

  void updateState(const RigidBodyState &state) {
    state_ = state;
  }
};

} // namespace qrotor_gazebo
#endif // QROTOR_GAZEBO_CONTROL_H
