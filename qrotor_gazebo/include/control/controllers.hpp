#ifndef QROTOR_GAZEBO_CONTROLLER_HPP
#define QROTOR_GAZEBO_CONTROLLER_HPP

#include <eigen3/Eigen/Dense>
#include <iostream>
#include <string>

#include "control/control_types.hpp"

namespace qrotor_gazebo {

class SO3Controller {
private:
  Eigen::Vector3d kp_{0.5, 0.5, 0.5};
  Eigen::Vector3d kd_{0.25, 0.25, 0.25};

  Eigen::Matrix3d inertia_;
  Eigen::Matrix3d inertia_inv_;
  Eigen::Vector3d moment_;

public:
  SO3Controller(const Eigen::Matrix3d &_inertia) {
    inertia_ = _inertia;
    inertia_inv_ = _inertia.inverse();
  }
  SO3Controller()
      : SO3Controller((Eigen::Matrix3d() << 0.0049, 5.5e-06, 5.4e-06, 5.5e-06,
                       0.0053, 2.1e-05, 5.4e-06, 2.1e-05, 0.0098)
                          .finished()) {}
  ~SO3Controller() = default;

  void updateParams(const Eigen::Matrix3d &inertia) {
    inertia_ = inertia;
    inertia_inv_ = inertia.inverse();
  }

  Eigen::Vector3d run(double dt, TSO3d x, TSO3d xd) {
    Eigen::Matrix<double, 6, 1> errors = x - xd;
    Eigen::Vector3d eR = errors.head(3);
    Eigen::Vector3d eOm = errors.tail(3);
    Eigen::Vector3d u = -kp_.cwiseProduct(eR) - kd_.cwiseProduct(eOm);
    u += x.Omega.cross(inertia_ * x.Omega);
    u += -inertia_ * (x.Omega.cross(x.R.transpose() * xd.R * xd.Omega) -
                      x.R.transpose() * xd.R * xd.dOmega);
    return u;
  }
};

class PositionController {
private:
  Eigen::Vector3d kp_;
  Eigen::Vector3d kd_;
  Eigen::Vector3d ki_;

  Eigen::Vector3d pos_integral_err{0., 0., 0.};
  double mass_;
  bool INTEGRAL_UPDATE = false;
  Eigen::Vector3d FORCE_UPPER_BOUND, FORCE_LOWER_BOUND;
  Eigen::Vector3d POS_INTEGRAL_LB, POS_INTEGRAL_UB;

  Eigen::Vector3d xd, vd, ad;

  const double g{9.81};

public:
  PositionController() {
    pos_integral_err.setZero();
    POS_INTEGRAL_LB << -5.0, -5.0, -5.0;
    POS_INTEGRAL_UB << 5.0, 5.0, 5.0;
    FORCE_LOWER_BOUND << -0.5 * g, -0.5 * g, -g;
    FORCE_UPPER_BOUND << 0.5 * g, 0.5 * g, g;
  }
  ~PositionController() = default;

  void updateParams(const double &mass) { mass_ = mass; }

  void updateSetpoint(Eigen::Vector3d xd, Eigen::Vector3d vd,
                      Eigen::Vector3d ad) {
    xd = std::move(xd);
    vd = std::move(vd);
    ad = std::move(ad);
  }

  Eigen::Vector3d run(const double dt, const Eigen::Vector3d &pos,
                      const Eigen::Vector3d &vel) {
    Eigen::Vector3d pos_err = pos - xd;
    Eigen::Vector3d vel_err = vel - vd;
    Eigen::Vector3d thrust_v;
    thrust_v.setZero();

    // feedback: PD input
    thrust_v = -kp_.cwiseProduct(pos_err) - kd_.cwiseProduct(vel_err);

    // integral update
    if (INTEGRAL_UPDATE) {
      pos_integral_err += pos_err * dt;

      // basic anti-windup (bounding the integral error);
      pos_integral_err = (pos_integral_err.cwiseMax(POS_INTEGRAL_LB))
                             .cwiseMin(POS_INTEGRAL_UB);

      // adding integral force
      thrust_v += -ki_.cwiseProduct(pos_integral_err);
    } else {
      pos_integral_err.setZero();
    }

    // input-bounds
    return (thrust_v.cwiseMax(FORCE_LOWER_BOUND)).cwiseMin(FORCE_UPPER_BOUND);
  }
};

} // namespace qrotor_gazebo

#endif // QROTOR_GAZEBO_CONTROLLER_HPP