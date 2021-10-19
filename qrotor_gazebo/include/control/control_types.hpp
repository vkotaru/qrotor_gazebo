#ifndef QROTOR_GAZEBO_CONTROL_TYPES_HPP
#define QROTOR_GAZEBO_CONTROL_TYPES_HPP
#include <eigen3/Eigen/Dense>

namespace qrotor_gazebo {

static const Eigen::Vector3d gravity{0., 0., 1.};

template <typename T, int _Dim>
class SpecialOrthogonal : public Eigen::Matrix<T, _Dim, _Dim> {
public:
  SpecialOrthogonal(void) : Eigen::Matrix<T, _Dim, _Dim>() {}

  template <typename OtherDerived>
  SpecialOrthogonal(const Eigen::MatrixBase<OtherDerived> &other)
      : Eigen::Matrix<T, _Dim, _Dim>(other) {}

  template <typename OtherDerived>
  SpecialOrthogonal &operator=(const Eigen::MatrixBase<OtherDerived> &other) {
    this->Eigen::Matrix<T, _Dim, _Dim>::operator=(other);
    return *this;
  }
  SpecialOrthogonal inverse() { return this->transpose(); }
};

template <typename T> class SO3 : public SpecialOrthogonal<T, 3> {
public:
  SO3(void) : SpecialOrthogonal<T, 3>() {}

  template <typename OtherDerived>
  SO3(const Eigen::MatrixBase<OtherDerived> &other)
      : SpecialOrthogonal<T, 3>(other) {}

  template <typename OtherDerived>
  SO3 &operator=(const Eigen::MatrixBase<OtherDerived> &other) {
    this->SpecialOrthogonal<T, 3>::operator=(other);
    return *this;
  }

  template <typename T1, typename T2>
  static Eigen::Matrix<T, 3, 1> error(const Eigen::MatrixBase<T1> &R,
                                      const Eigen::MatrixBase<T2> &Rd) {
    // eR = 0.5*vee(Rd'*R-R'*Rd);
    auto eR = 0.5 * (Rd.transpose() * R - R.transpose() * Rd);
    return (Eigen::Matrix<T, 3, 1>() << eR(2, 1), eR(0, 2), eR(1, 0))
        .finished();
  }
  template <typename T1, typename T2>
  static T config_error(const Eigen::MatrixBase<T1> &R,
                        const Eigen::MatrixBase<T2> &Rd) {
    // Psi =  0.5*trace(eye(3)-Rd'*R);
    T Psi =
        0.5 * (Eigen::Matrix<T, 3, 3>::Identity() - Rd.transpose() * R).trace();
    return Psi;
  }

  template <typename OtherDerived>
  Eigen::Matrix<T, 3, 1> error(const SO3<OtherDerived> &other) {
    return SO3<T>::error(*this, other);
  }
  template <typename OtherDerived>
  T config_error(const SO3<OtherDerived> &other) {
    return SO3<T>::config_error(*this, other);
  }
};

template <typename T> class TSO3 {
public:
  TSO3(/* args */) {
    R.setIdentity();
    Omega.setZero();
    dOmega.setZero();
  }
  ~TSO3() = default;
  SO3<T> R;
  Eigen::Matrix<T, 3, 1> Omega;
  Eigen::Matrix<T, 3, 1> dOmega; // feed-forward usage

  template <typename OtherDerived>
  TSO3 &operator=(const TSO3<OtherDerived> &other) {
    this->R = other.R;
    this->Omega = other.Omega;
    this->dOmega = other.dOmega;
  }

  template <typename OtherDerived>
  Eigen::Matrix<T, 6, 1> error(const TSO3<OtherDerived> &other) {
    Eigen::Matrix<T, 6, 1> err_;
    err_ << this->R.error(other.R),
        this->Omega - this->R.transpose() * other.R * other.Omega;
    return err_;
  }

  template <typename OtherDerived>
  Eigen::Matrix<T, 6, 1> operator-(const TSO3<OtherDerived> &other) {
    Eigen::Matrix<T, 6, 1> err_;
    err_ << this->R.error(other.R),
        this->Omega - this->R.transpose() * other.R * other.Omega;
    return err_;
  }

  virtual void print() const {
    std::cout << "rotation: " << this->R << std::endl;
    std::cout << "angular velocity: " << this->Omega.transpose() << std::endl;
  }
};

template <typename T> class TSE3 : public TSO3<T> {
public:
  TSE3() : TSO3<T>() {
    position.setZero();
    velocity.setZero();
    acceleration.setZero();
  }
  ~TSE3() = default;

  Eigen::Matrix<T, 3, 1> position;
  Eigen::Matrix<T, 3, 1> velocity;
  Eigen::Matrix<T, 3, 1> acceleration;

  template <typename OtherDerived>
  TSE3 &operator=(const TSE3<OtherDerived> &other) {
    this->position = other.position;
    this->velocity = other.velocity;
    this->acceleration = other.acceleration;
    this->R = other.R;
    this->Omega = other.Omega;
    this->dOmega = other.dOmega;
  }

  void print() const override {
    std::cout << "position: " << this->position.transpose() << std::endl;
    std::cout << "velocity: " << this->velocity.transpose() << std::endl;
    TSO3<T>::print();
  }

  template <typename OtherDerived>
  Eigen::Matrix<T, 12, 1> error(const TSE3<OtherDerived> &other) {
    auto pos_err = this->position - other.position;
    auto vel_err = this->velocity - other.velocity;
    auto rot_err = this->R.error(other.R);
    auto ang_vel_err =
        this->Omega - this->R.transpose() * other.R * other.Omega;

    Eigen::Matrix<T, 12, 1> err_;
    err_ << pos_err, vel_err, rot_err, ang_vel_err;
    return err_;
  }

  template <typename OtherDerived>
  Eigen::Matrix<T, 12, 1> operator-(const TSE3<OtherDerived> &other) {
    return (Eigen::Matrix<T, 12, 1>() << position - other.position,
            velocity - other.velocity, this->R.error(other.R),
            this->Omega - this->R.transpose() * other.R * other.Omega)
        .finished();
  }

  TSO3<T> extractTSO3() {
    TSO3<T> att;
    att.R = this->R;
    att.Omega = this->Omega;
    return att;
  }
};

using SO3d = SO3<double>;
using SO3f = SO3<float>;
using TSO3d = TSO3<double>;
using TSO3f = TSO3<float>;

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
