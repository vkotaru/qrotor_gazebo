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
    ros::init(argc, argv, "gazebo_client", ros::init_options::NoSigintHandler);
  }

  // Create ROS node. This acts in a similar manner to the Gazebo node
  this->ctrl_nh_ = boost::make_shared<ros::NodeHandle>(namespace_);

  ctrl_start_time_s = ros::Time::now().toSec();
  ctrl_last_update_s = ros::Time::now().toSec();

  // Spin up the queue helper thread.
  this->ctrlQueueThread =
      std::thread(std::bind(&QrotorPlugin::controlThread, this));

  gzdbg << "[QrotorPlugin]... loaded!";
}

void QrotorPlugin::Init() {
  // set initial pose
  state_ = Pose3D(link_);
}

void QrotorPlugin::OnUpdate(
    const gazebo::common::UpdateInfo &_info) { // time update
  double dt = _info.simTime.Double() - last_plugin_update.Double();
  last_plugin_update = _info.simTime.Double();
  //  gzmsg << "dt_s: " << dt_s << std::endl;

  // apply wrench
  applyWrench(thrust*E3, moment);
}

void QrotorPlugin::controlThread() {
  // ros publishers
  pub_odom_truth_ = this->ctrl_nh_->advertise<nav_msgs::Odometry>(
      "/qrotor_plugin/odometry", 1);

  double ros_start_s = ros::Time::now().toSec();
  ros::Rate loop_handle(ctrl_loop_freq_);
  while (this->ctrl_nh_->ok()) {
    // update time
    ros_now_s = ros::Time::now().toSec();
    ros_dt = (float)(ros_now_s - ros_last_update_s);
    ros_last_update_s = ros_now_s;

    computeInput();
    rosPublish();

    // loop handle
    loop_handle.sleep();
  }
}

void QrotorPlugin::applyWrench(const Eigen::Vector3d &thrust_v,
                               const Eigen::Vector3d &moment_v) {
  GazeboVector force = vec3_to_gazebo_from_eigen(thrust_v);
  GazeboVector torque = vec3_to_gazebo_from_eigen(moment_v);
  link_->AddRelativeForce(force);
  link_->AddRelativeTorque(torque - link_->GetInertial()->CoG().Cross(force));
}

void QrotorPlugin::rosPublish() {}

GZ_REGISTER_MODEL_PLUGIN(QrotorPlugin);
} // namespace qrotor_gazebo