#ifndef QROTOR_GAZEBO_CONTROLLER_HPP
#define QROTOR_GAZEBO_CONTROLLER_HPP

#include <eigen3/Eigen/Dense>
#include <iostream>
#include <string>

#include "control/control_types.hpp"

namespace qrotor_gazebo {

class SO3Controller {
private:
  Eigen::Vector3d kp_{8.0, 8.0, 3.0};
  Eigen::Vector3d kd_{0.3, 0.3, 0.225};

  Eigen::Matrix3d inertia_{};
  Eigen::Matrix3d inertia_inv_{};
  Eigen::Vector3d moment_{};

  Eigen::Vector3d thrust_vector_{};
  double yaw_sp = 0;

  Eigen::Matrix3d Rd = Eigen::Matrix3d::Identity();
  Eigen::Vector3d Omd = Eigen::Vector3d::Zero();
  Eigen::Vector3d dOmd = Eigen::Vector3d::Zero();

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

  void updateYawSP(const double &_yaw) { yaw_sp = _yaw; }

  void updateCommand(const Eigen::Vector3d &thrust_v) {
    Eigen::Vector3d E1d{E1}, E1c{E1}, E2c{E2}, E3c{E3};

    thrust_vector_ = thrust_v;
    if (!(std::isnan(thrust_vector_.norm()) ||
          (thrust_vector_.norm() < 1e-4))) {
      E3c = thrust_vector_.normalized();
    }
    E1d = Eigen::Vector3d(std::cos(yaw_sp), std::sin(yaw_sp), 0.0);
    E1c = -hat3d(E3c) * hat3d(E3c) * E1d;
    E2c = E3c.cross(E1c);

    Rd << E1c, E2c, E3c;
  }

  Eigen::Vector3d run(double dt, Eigen::Matrix3d R, Eigen::Vector3d Om) {
    Eigen::Vector3d eR = 0.5 * vee3d(Rd.transpose() * R - R.transpose() * Rd);
    Eigen::Vector3d eOm = Om - R.transpose() * Rd * Omd;
    Eigen::Vector3d u = -kp_.cwiseProduct(eR) - kd_.cwiseProduct(eOm);
    u += Om.cross(inertia_ * Om);
    u += -inertia_ *
         (Om.cross(R.transpose() * Rd * Omd) -
          R.transpose() * Rd * dOmd);
    return u;
  }
};

class PositionController {
private:
  Eigen::Vector3d kp_{4.0, 4.0, 8.0};
  Eigen::Vector3d kd_{3.0, 3.0, 6.0};
  Eigen::Vector3d ki_{0.5, 0.5, 0.5};

  Eigen::Vector3d pos_integral_err{0., 0., 0.};
  double mass_{0.8};
  bool INTEGRAL_UPDATE = false;
  Eigen::Vector3d FORCE_UPPER_BOUND{}, FORCE_LOWER_BOUND{};
  Eigen::Vector3d POS_INTEGRAL_LB{}, POS_INTEGRAL_UB{};

  Eigen::Vector3d xd{0., 0., 0.};
  Eigen::Vector3d vd{0., 0., 0.};
  Eigen::Vector3d ad{0., 0., 0.};

  const double g{9.81};
  const Eigen::Vector3d g_v{0., 0., 9.81};

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

  void updateSetpoint(const Eigen::Vector3d &xd) {
    this->xd = xd;
    vd.setZero();
    ad.setZero();
  }
  void updateSetpoint(const Eigen::Vector3d &xd, const Eigen::Vector3d &vd) {
    this->xd = xd;
    this->vd = vd;
    ad.setZero();
  }
  void updateSetpoint(const Eigen::Vector3d &xd, const Eigen::Vector3d &vd,
                      const Eigen::Vector3d &ad) {
    this->xd = xd;
    this->vd = vd;
    this->ad = ad;
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

    // feed-forward input
    thrust_v += (ad + g_v) * mass_;

    // input-bounds
    return (thrust_v.cwiseMax(FORCE_LOWER_BOUND)).cwiseMin(FORCE_UPPER_BOUND);
  }
};

} // namespace qrotor_gazebo

#endif // QROTOR_GAZEBO_CONTROLLER_HPP