/**
 * @file qrotor_plugin.cpp
 * @author kotaru
 * @date 10/16/21.
 */

#include "qrotor_gazebo/qrotor_plugin.h"

namespace qrotor_gazebo {

QrotorPlugin::QrotorPlugin() = default;

QrotorPlugin::~QrotorPlugin() {
  updateConnection_.reset();
  if (nh_) {
    nh_->shutdown();
  }
}

void QrotorPlugin::Reset() {
  link_->SetWorldPose(initial_pose_);
  link_->ResetPhysicsStates();
  gzdbg << "[QrotorPlugin] Reset!" << std::endl;
}

void QrotorPlugin::Load(gazebo::physics::ModelPtr _model,
                        sdf::ElementPtr _sdf) {

  if (!ros::isInitialized()) {
    gzerr << "A ROS node for Gazebo has not been "
             "initialized, unable to load "
             "plugin"
          << std::endl;
    return;
  }
  gzdbg << "[QrotorPlugin] loading... " << std::endl;

  model_ = _model;
  world_ = model_->GetWorld();
  namespace_.clear();

  gzdbg << "[QrotorPlugin] searching for namespace... ";
  if (_sdf->HasElement("namespace")) {
    namespace_ = _sdf->GetElement("namespace")->Get<std::string>();
    std::cout << namespace_ << std::endl;
  } else {
    gzerr << "[QrotorPlugin] please specify a namespace.\n";
  }

  // ros node handle
  nh_ = std::make_unique<ros::NodeHandle>(namespace_);

  if (_sdf->HasElement("linkName")) {
    link_name_ = _sdf->GetElement("linkName")->Get<std::string>();
  } else {
    gzerr << "[QrotorPlugin] Please specify a linkName of the forces and"
             "moments plugin.\n";
  }
  link_ = model_->GetLink(link_name_);
  if (link_ == NULL) {
    gzthrow("[QrotorPlugin] Couldn't find specified link \"" << link_name_
                                                             << "\".");
  }

  // Inertial setup
  inertia_ << link_->GetInertial()->IXX(), link_->GetInertial()->IXY(),
      link_->GetInertial()->IXZ(), link_->GetInertial()->IXY(),
      link_->GetInertial()->IYY(), link_->GetInertial()->IYZ(),
      link_->GetInertial()->IXZ(), link_->GetInertial()->IYZ(),
      link_->GetInertial()->IZZ();
  mass_ = link_->GetInertial()->Mass();
  gzdbg << "mass: " << mass_ << "\ninertia: \n" << inertia_ << std::endl;

  // Connect the update function to the simulation
  updateConnection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      boost::bind(&QrotorPlugin::OnUpdate, this, _1));

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

  pub_odom_truth_ = this->nh_->advertise<nav_msgs::Odometry>(
      namespace_ + "/qrotor_plugin/odometry", 1);
  sub_command_ = this->nh_->subscribe(namespace_ + "/command", 10,
                                      &QrotorPlugin::commandCallback, this);

  this->ctrl_nh_ = boost::make_shared<ros::NodeHandle>(namespace_);
  this->ctrlQueueThread =
      std::thread(std::bind(&QrotorPlugin::ctrlThread, this));

  gzdbg << "[QrotorPlugin]... loaded!";
}

void QrotorPlugin::Init() {
  // set initial pose
  state_ = Pose3D(link_);
}

void QrotorPlugin::commandCallback(
    const qrotor_gazebo::Command::ConstPtr &msg) {

  controller_.setMode(msg->mode);
}

void QrotorPlugin::OnUpdate(const gazebo::common::UpdateInfo &_info) {
  double dt = _info.simTime.Double() - last_plugin_update.Double();
  last_plugin_update = _info.simTime.Double();
  applyWrench(controller_.thrust() * E3, controller_.moment());
}

void QrotorPlugin::ctrlThread() {
  ros::Rate loop_handle(att_loop_freq);
  int MAX_POS_CTRL_COUNT = std::ceil(att_loop_freq / pos_loop_freq);
  int pos_ctrl_counter = MAX_POS_CTRL_COUNT;
  while (this->ctrl_nh_->ok()) {

    if (pos_ctrl_counter == MAX_POS_CTRL_COUNT) {
      posCtrlThread();
      pos_ctrl_counter = 0;
    } else {
      pos_ctrl_counter++;
    }
    attCtrlThread();
    // loop handle
    loop_handle.sleep();
  }
}

void QrotorPlugin::posCtrlThread() {
  double dt = pos_clock_.time();
  // compute input
  if (controller_.mode() == QrotorControl::POSITION ||
      controller_.mode() == QrotorControl::POSITION_SPLINE) {
    controller_.computePositionInput();
  }
  // publish pose
  rosPublish();
}

void QrotorPlugin::attCtrlThread() {
  double dt = att_clock_.time();
  // compute input
  controller_.computeAttitudeInput();
}

void QrotorPlugin::applyWrench(const Eigen::Vector3d &thrust_v,
                               const Eigen::Vector3d &moment_v) {
  GazeboVector force = vec3_to_gazebo_from_eigen(thrust_v);
  GazeboVector torque = vec3_to_gazebo_from_eigen(moment_v);
  link_->AddRelativeForce(force);
  link_->AddRelativeTorque(torque - link_->GetInertial()->CoG().Cross(force));
}

void QrotorPlugin::rosPublish() {
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

GZ_REGISTER_MODEL_PLUGIN(QrotorPlugin);
} // namespace qrotor_gazebo
