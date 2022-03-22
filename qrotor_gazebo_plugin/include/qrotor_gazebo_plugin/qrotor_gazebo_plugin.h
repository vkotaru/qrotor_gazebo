/**
 * @file qrotor_gazebo.h
 * @author kotaru
 * @date 10/16/21.
 */
#ifndef QROTOR_GAZEBO_QROTOR_PLUGIN_H
#define QROTOR_GAZEBO_QROTOR_PLUGIN_H

#include "qrotor_gazebo_plugin/common.h"
#include "qrotor_gazebo_plugin/control.hpp"
#include <eigen_conversions/eigen_msg.h>

#include <nav_msgs/Odometry.h>
#include <qrotor_gazebo_plugin/Command.h>

namespace qrotor_gazebo {

class QrotorGazeboPlugin : public gazebo::ModelPlugin {
public:
  QrotorGazeboPlugin();
  ~QrotorGazeboPlugin();

protected:
  void Reset() override;
  void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) override;
  void OnUpdate(const gazebo::common::UpdateInfo &_info);
  void Init() override;

private:
  gazebo::common::Time plugin_loaded_time, last_plugin_update;

  std::string namespace_;
  std::string link_name_;

  gazebo::physics::WorldPtr world_;
  gazebo::physics::ModelPtr model_;
  gazebo::physics::LinkPtr link_;
  gazebo::physics::JointPtr joint_;
  gazebo::physics::EntityPtr parent_link_;
  // Pointer to the update event connection.
  gazebo::event::ConnectionPtr updateConnection_;

  GazeboPose initial_pose_{};
  void queryState();

  // parameters
  double mass_{};
  Eigen::Matrix3d inertia_{};
  QrotorControl controller_{};

  // ros node
  ros::NodeHandlePtr nh_;
  double pos_loop_freq{100}, att_loop_freq{500};
  ros::NodeHandlePtr ctrl_nh_;
  std::thread ctrlQueueThread;
  RosClock pos_clock_{}, att_clock_{};
  void ctrlThread();
  void posCtrlThread();
  void attCtrlThread();
  ros::Publisher pub_odom_truth_;
  ros::Subscriber sub_command_;

  void commandCallback(const qrotor_gazebo_plugin::Command::ConstPtr &msg);

  // plugin functions
  void applyWrench(const Eigen::Vector3d &thrust_v,
                   const Eigen::Vector3d &moment_v);
  void rosPublish();
};

} // namespace qrotor_gazebo
#endif // QROTOR_GAZEBO_QROTOR_PLUGIN_H
