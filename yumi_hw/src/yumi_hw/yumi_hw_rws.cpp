#include <yumi_hw/yumi_hw_rws.h>
#include <yumi_hw/arm_rws_interface.h>

YumiHWRWS::YumiHWRWS(const double &exponential_smoothing_alpha)
    : YumiHW(exponential_smoothing_alpha), is_initialized_(false),
      is_setup_(false), first_run_in_position_mode_(true) {}

YumiHWRWS::~YumiHWRWS() { rws_interface_.stopThreads(); }

void YumiHWRWS::setup(std::string ip, int port) {
  ip_ = ip;
  port_ = port;
  is_setup_ = true;
}

bool YumiHWRWS::init() {
  if (is_initialized_) {
    return false;
  }

  if (!is_setup_) {
    ROS_ERROR("IP and port of controller are not set up!");
    return false;
  }

  rws_interface_.init(ip_, port_);
  rws_interface_.startThreads();
  is_initialized_ = true;

  return true;
}

void YumiHWRWS::read(ros::Time time, ros::Duration period) {
  if (!is_initialized_) {
    return;
  }

  // ROS_INFO("reading joints");
  data_buffer_mutex_.lock();
  rws_interface_.getCurrentJointStates(read_joint_position_);

  for (int j = 0; j < n_joints_; j++) {
    joint_position_prev_[j] = joint_position_[j];
    joint_position_[j] = read_joint_position_[j];
    // joint_effort_[j] = readJntEffort[j]; //TODO: read effort
    joint_velocity_[j] = filters::exponentialSmoothing(
        (joint_position_[j] - joint_position_prev_[j]) / period.toSec(),
        joint_velocity_[j], exponential_smoothing_alpha_);
    if (first_run_in_position_mode_) {
      joint_position_command_[j] = read_joint_position_[j];
    }
  }
  first_run_in_position_mode_ = false;
  data_buffer_mutex_.unlock();
  ROS_INFO("read joints");

  return;
}

void YumiHWRWS::write(ros::Time time, ros::Duration period) {
  if (!is_initialized_)
    return;

  enforceLimits(period);

  // ROS_INFO("writing joints");
  data_buffer_mutex_.lock();
  switch (getControlStrategy()) {
  case JOINT_POSITION:
    for (int j = 0; j < n_joints_; j++) {
      new_joint_position_[j] = joint_position_command_[j];
    }
    break;

  case JOINT_VELOCITY:
    // std::cerr<<"ASS = ";
    for (int j = 0; j < n_joints_; j++) {
      // std::cerr<<joint_velocity_command_[j]<<"*"<<period.toSec()<<" +
      // "<<joint_position_[j]<<" ::: ";
      new_joint_position_[j] =
          joint_velocity_command_[j]; //*period.toSec() + joint_position_[j];
    }
    // std::cerr<<std::endl;
    first_run_in_position_mode_ = true;
    break;
  // case JOINT_EFFORT:
  // break;
  default:
    break;
  }

  rws_interface_.setJointTargets(new_joint_position_, getControlStrategy());
  data_buffer_mutex_.unlock();
  // ROS_INFO("wrote joints");

  return;
}
