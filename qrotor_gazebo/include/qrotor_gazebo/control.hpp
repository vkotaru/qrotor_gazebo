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
    POSITION,
    POSITION_SPLINE
  };

private:
  Mode mode_;
  Eigen::Vector3d thrust_vector;
  Eigen::Vector3d moment_;

public:
  QrotorControl() {}
  ~QrotorControl() {}

  PositionController posCtrl_{};
  SO3Controller attCtrl_{};

  void updateParams(const double &mass, const Eigen::Matrix3d &inertia) {
    posCtrl_.updateParams(mass);
    attCtrl_.updateParams(inertia);
  }

  TSO3d des_att_{};
  FlatVariabled flats_{};

  void setMode(const int &mode) {
    switch (mode) {
    case 0:
      mode_ == PASS_THROUGH;
      break;

    case 1:
      mode_ = ATTITUDE;
      break;

    case 2:
      mode_ = POSITION;
      break;

    case 3:
      mode_ = POSITION_SPLINE;
      break;

    default:
      break;
    }
  }
  const Mode &mode() const { return mode_; }
  void computeAttitudeInput() {}
  void computePositionInput() { 
    // thrust_vector = posCtrl_.run(); 
  }

  double thrust() const {
    return thrust_vector(2);
  }
  Eigen::Vector3d moment() const {
    return moment_;
  }
};

} // namespace qrotor_gazebo
#endif // QROTOR_GAZEBO_CONTROL_H
