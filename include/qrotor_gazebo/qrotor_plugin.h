/**
 * @file qrotor_gazebo.h
 * @author kotaru
 * @date 10/16/21.
 */
#ifndef QROTOR_GAZEBO_QROTOR_PLUGIN_H
#define QROTOR_GAZEBO_QROTOR_PLUGIN_H

#include "qrotor_gazebo/common.h"

#include <nav_msgs/Odometry.h>

namespace qrotor_gazebo {

class QrotorPlugin : public gazebo::ModelPlugin {
public:
  QrotorPlugin();
  ~QrotorPlugin();

protected:
  void Reset() override;
  void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) override;
  void OnUpdate(const gazebo::common::UpdateInfo &_info);
  void Init() override;

private:
  gazebo::common::Time plugin_loaded_time, last_plugin_update;  
  double ros_start_time_s = 0.0;
  double ros_last_update_s = 0.0;
  double ros_now_s = 0.0;
  float ros_dt = 0.0;
  double dtsum = 0;

  std::string namespace_;
  std::string link_name_;

  gazebo::physics::WorldPtr world_;
  gazebo::physics::ModelPtr model_;
  gazebo::physics::LinkPtr link_;
  gazebo::physics::JointPtr joint_;
  gazebo::physics::EntityPtr parent_link_;
  // Pointer to the update event connection.
  gazebo::event::ConnectionPtr updateConnection_;

  // states
  Pose3D state_;
  GazeboPose initial_pose_;

  // parameters
  double mass_;
  Eigen::Matrix3d inertia_;
  // input variables
  double thrust;
  Eigen::Vector3d moment;

  // ros node
  ros::NodeHandlePtr nh_;

  double ctrl_loop_freq_{500};
  // ros node control
  ros::NodeHandlePtr ctrl_nh_;
  /// \brief A thread the keeps running the rosQueue
  std::thread ctrlQueueThread;  
  /// \brief run firmware_node in parallel
  void controlThread();
  ros::Publisher pub_odom_truth_;

  double ctrl_start_time_s{0}, ctrl_last_update_s{0};

  // plugin functions
  void computeInput();
  void applyWrench(const Eigen::Vector3d &thrust_v,
                   const Eigen::Vector3d &moment_v);
  void rosPublish();
};

} // namespace qrotor_gazebo
#endif // QROTOR_GAZEBO_QROTOR_PLUGIN_H
