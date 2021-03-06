/**
 * @file qrotor_gazebo_plugin.cpp
 * @author kotaru
 * @date 10/16/21.
 */

#include "qrotor_gazebo_plugin/qrotor_gazebo_plugin.h"

namespace qrotor_gazebo {

QrotorGazeboPlugin::QrotorGazeboPlugin() = default;

QrotorGazeboPlugin::~QrotorGazeboPlugin() {
  updateConnection_.reset();
  if (nh_) {
    nh_->shutdown();
  }
}

void QrotorGazeboPlugin::Reset() {
  link_->SetWorldPose(initial_pose_);
  link_->ResetPhysicsStates();
  // controller_.posCtrl_.updateSetpoint(Eigen::Vector3d::Zero(),
  //                                     Eigen::Vector3d::Zero(),
  //                                     Eigen::Vector3d::Zero());
  controller_.attCtrl_.updateCommand(Eigen::Vector3d::Zero());
  gzdbg << "[QrotorGazeboPlugin] Reset!" << std::endl;
}

void QrotorGazeboPlugin::Load(gazebo::physics::ModelPtr _model,
                              sdf::ElementPtr _sdf) {

  if (!ros::isInitialized()) {
    gzerr << "A ROS node for Gazebo has not been "
             "initialized, unable to load "
             "plugin"
          << std::endl;
    return;
  }
  gzdbg << "[QrotorGazeboPlugin] loading... " << std::endl;

  model_ = _model;
  world_ = model_->GetWorld();
  namespace_.clear();

  gzdbg << "[QrotorGazeboPlugin] searching for namespace... ";
  if (_sdf->HasElement("namespace")) {
    namespace_ = _sdf->GetElement("namespace")->Get<std::string>();
    std::cout << namespace_ << std::endl;
  } else {
    gzerr << "[QrotorGazeboPlugin] please specify a namespace.\n";
  }

  // ros node handle
  nh_ = std::make_unique<ros::NodeHandle>(namespace_);

  if (_sdf->HasElement("linkName")) {
    link_name_ = _sdf->GetElement("linkName")->Get<std::string>();
  } else {
    gzerr << "[QrotorGazeboPlugin] Please specify a linkName of the forces and"
             "moments plugin.\n";
  }
  link_ = model_->GetLink(link_name_);
  if (link_ == NULL) {
    gzthrow("[QrotorGazeboPlugin] Couldn't find specified link \"" << link_name_
                                                                   << "\".");
  }

  if (!_sdf->HasElement("updateRate")) {
    gzdbg << "[QrotorGazeboPlugin] missing <updateRate>, "
             "defaults to 0.0"
             " (as fast as possible)\n";
    this->update_rate_ = 0;
  } else {
    this->update_rate_ = _sdf->GetElement("updateRate")->Get<double>();
    gzdbg << "[QrotorGazeboPlugin] update rate " << update_rate_ << std::endl;
  }
  MAX_POS_CTRL_COUNT = std::ceil(update_rate_ / pos_loop_freq);
  pos_ctrl_counter = MAX_POS_CTRL_COUNT;

  // Inertial setup
  inertia_ << link_->GetInertial()->IXX(), link_->GetInertial()->IXY(),
      link_->GetInertial()->IXZ(), link_->GetInertial()->IXY(),
      link_->GetInertial()->IYY(), link_->GetInertial()->IYZ(),
      link_->GetInertial()->IXZ(), link_->GetInertial()->IYZ(),
      link_->GetInertial()->IZZ();
  mass_ = link_->GetInertial()->Mass();
  gzdbg << "mass: " << mass_ << "\ninertia: \n" << inertia_ << std::endl;
  controller_.posCtrl_.updateParams(mass_);
  controller_.attCtrl_.updateParams(inertia_);

  // Connect the update function to the simulation
  updateConnection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      boost::bind(&QrotorGazeboPlugin::OnUpdate, this, _1));

  // Getting the initial pose of the link
  initial_pose_ = link_->WorldCoGPose();

  plugin_loaded_time = world_->SimTime();
  last_plugin_update = plugin_loaded_time;

  // Initialize ros, if it has not already initialized.
  if (!ros::isInitialized()) {
    int argc = 0;
    char **argv = nullptr;
    ros::init(argc, argv, "gazebo_client");
  }
  this->ctrl_nh_ = boost::make_shared<ros::NodeHandle>(namespace_);
  pub_odom_truth_ = this->ctrl_nh_->advertise<nav_msgs::Odometry>(
      namespace_ + "/qrotor_plugin/odometry", 1);
  sub_command_ = this->ctrl_nh_->subscribe(
      "command", 10, &QrotorGazeboPlugin::commandCallback, this);

#if GAZEBO_MAJOR_VERSION >= 8
  this->last_time_ = this->world_->SimTime();
#else
  this->last_time_ = this->world_->GetSimTime();
#endif

  gzdbg << "[QrotorGazeboPlugin]... loaded!";
}

void QrotorGazeboPlugin::Init() { queryState(); }

void QrotorGazeboPlugin::OnUpdate(const gazebo::common::UpdateInfo &_info) {
  double dt = _info.simTime.Double() - last_plugin_update.Double();
  last_plugin_update = _info.simTime.Double();

#if GAZEBO_MAJOR_VERSION >= 8
  gazebo::common::Time cur_time = this->world_->SimTime();
#else
  gazebo::common::Time cur_time = this->world_->GetSimTime();
#endif

  if (cur_time < last_time_) {
    gzdbg << "[QrotorGazeboPlugin] Negative update time difference detected.\n";
    last_time_ = cur_time;
  }
  // rate control
  if (update_rate_ > 0 &&
      (cur_time - last_time_).Double() < (1.0 / this->update_rate_)) {

  } else {
    // compute "dt"
    double dt = cur_time.Double() - last_time_.Double();
    queryState();
    if (pos_ctrl_counter == MAX_POS_CTRL_COUNT) {
      computePosInput();
      pos_ctrl_counter = 0;
    } else {
      pos_ctrl_counter++;
    }
    computeAttInput();
  }

  applyWrench(controller_.thrust() * E3, controller_.moment());
}

void QrotorGazeboPlugin::commandCallback(
    const qrotor_gazebo_plugin::Command::ConstPtr &msg) {

  switch (msg->mode) {
  case qrotor_gazebo_plugin::Command::MODE_POSITION: {
    controller_.setMode(msg->mode);
    switch (msg->command.size()) {
    case 3:
      controller_.posCtrl_.updateSetpoint(
          Eigen::Vector3d(msg->command[0].x, msg->command[0].y,
                          msg->command[0].z),
          Eigen::Vector3d(msg->command[1].x, msg->command[1].y,
                          msg->command[1].z),
          Eigen::Vector3d(msg->command[2].x, msg->command[2].y,
                          msg->command[2].z));
      break;

    case 2:
      controller_.posCtrl_.updateSetpoint(
          Eigen::Vector3d(msg->command[0].x, msg->command[0].y,
                          msg->command[0].z),
          Eigen::Vector3d(msg->command[1].x, msg->command[1].y,
                          msg->command[1].z));
      break;

    case 1:
      controller_.posCtrl_.updateSetpoint(Eigen::Vector3d(
          msg->command[0].x, msg->command[0].y, msg->command[0].z));
      break;

    default:
      break;
    }
  } break;

  case qrotor_gazebo_plugin::Command::MODE_THRUST_YAW: {
    controller_.setMode(msg->mode);
    controller_.skipComputingPosInput(Eigen::Vector3d(
        msg->command[0].x, msg->command[0].y, msg->command[0].z));
    controller_.attCtrl_.updateYawSP(msg->yaw[0]);
    break;
  }

  default:
    ROS_WARN("mode: %d is not implement at this time", msg->mode);
    break;
  }
}

void QrotorGazeboPlugin::queryState() {

  auto gpose = link_->WorldCoGPose();
  auto gvel = link_->RelativeLinearVel();
  auto gomega = link_->RelativeAngularVel();

  controller_.updateState(QrotorControl::RigidBodyState(
      vec3_to_eigen_from_gazebo(gpose.Pos()), vec3_to_eigen_from_gazebo(gvel),
      rotation_to_eigen_from_gazebo(gpose.Rot()),
      vec3_to_eigen_from_gazebo(gomega)));
}

void QrotorGazeboPlugin::computePosInput() {
  double dt = pos_clock_.time();
  // compute input
  if (controller_.mode() == QrotorControl::POSITION ||
      controller_.mode() == QrotorControl::POSITION_SPLINE) {
    controller_.computePositionInput(dt);
  }
  // publish pose
  rosPublish();
}

void QrotorGazeboPlugin::computeAttInput() {
  double dt = att_clock_.time();
  // compute input
  if (controller_.mode() != QrotorControl::PASS_THROUGH) {
    controller_.computeAttitudeInput(dt);
  }
}

void QrotorGazeboPlugin::applyWrench(const Eigen::Vector3d &thrust_v,
                                     const Eigen::Vector3d &moment_v) {

  GazeboVector force = vec3_to_gazebo_from_eigen(thrust_v);
  GazeboVector torque = vec3_to_gazebo_from_eigen(moment_v);
  link_->AddRelativeForce(force);
  link_->AddRelativeTorque(torque - link_->GetInertial()->CoG().Cross(force));
}

void QrotorGazeboPlugin::rosPublish() {
  auto pose = qrotor_gazebo::Pose3D(link_);
  nav_msgs::Odometry odometry_;
  odometry_.header.stamp.sec = world_->SimTime().sec;
  odometry_.header.stamp.nsec = world_->SimTime().nsec;
  odometry_.header.frame_id = "world";
  odometry_.child_frame_id = namespace_ + "/base_link";
  tf::quaternionEigenToMsg(quat_to_eigen_from_gazebo(pose.gpose.Rot()),
                           odometry_.pose.pose.orientation);
  tf::pointEigenToMsg(pose.pos, odometry_.pose.pose.position);
  tf::vectorEigenToMsg(pose.vel, odometry_.twist.twist.linear);
  tf::vectorEigenToMsg(pose.omega, odometry_.twist.twist.angular);
  pub_odom_truth_.publish(odometry_);
}

GZ_REGISTER_MODEL_PLUGIN(QrotorGazeboPlugin);
} // namespace qrotor_gazebo
