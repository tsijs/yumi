
#include <yumi_hw/yumi_egm.h>

YumiHWEGM::YumiHWEGM(const double &exponential_smoothing_alpha)
    : YumiHW(exponential_smoothing_alpha), is_initialized_(false) {}

YumiHWEGM::~YumiHWEGM() { yumi_egm_interface_.stop(); }

void YumiHWEGM::setup(const std::string &ip, const std::string &port_rws,
                      const int &port_egm_l, const int &port_egm_r) {
  ip_ = ip;
  port_rws_ = atoi(port_rws.c_str());
  port_egm_l_ = port_egm_l;
  port_egm_r_ = port_egm_r;
}

bool YumiHWEGM::init() {
  if (is_initialized_) {
    return false;
  }
  current_strategy_ = JOINT_VELOCITY; // TODO: get this from parameters

  bool is_initialized_r = yumi_egm_interface_r_.init(ip_, port_rws_, port_egm_r_);
  bool is_initialized_l = yumi_egm_interface_l_.init(ip_, port_rws_, port_egm_l_);
  is_initialized_ = is_initialized_l && is_initialized_r;

  return is_initialized_;
}

void YumiHWEGM::read(ros::Time time, ros::Duration period) {
  if (!is_initialized_) {
    ROS_WARN("Called read, but YumiHWEGM not initialized");
    return;
  }

  data_buffer_mutex_.lock();

  yumi_egm_interface_.getCurrentJointStates(joint_pos_, joint_vel_);

  for (int j = 0; j < n_joints_; j++) {
    joint_position_prev_[j] = joint_position_[j]; 
    joint_position_[j] = joint_pos_[j];
    joint_velocity_[j] = joint_vel_[j];
  }

  ROS_DEBUG("The received positions are: %f, %f, %f, %f, %f, %f, %f ", \
  joint_position_[0], joint_position_[1], joint_position_[2], joint_position_[3], joint_position_[4], joint_position_[5], joint_position_[6]);
  ROS_DEBUG("The received velocities are: %f, %f, %f, %f, %f, %f, %f ", \
  joint_velocity_[0], joint_velocity_[1], joint_velocity_[2], joint_velocity_[3], joint_velocity_[4], joint_velocity_[5], joint_velocity_[6] );

  data_buffer_mutex_.unlock();
}

void YumiHWEGM::write(ros::Time time, ros::Duration period) {
  if (!is_initialized_) {
    ROS_WARN("Called write, but YumiHWEGM not initialized");
    return;
  }

  enforceLimits(period);

  data_buffer_mutex_.lock();
  // TODO: when the params are used at init --> control strategy, this needs to
  // be changed as well
  for (int j = 0; j < n_joints_; j++) {
    joint_vel_targets_[j] =
        joint_velocity_command_[j]; // where is command set? controller manager?
  }
   ROS_DEBUG("targets: %f, %f, %f, %f, %f, %f,
   %f",joint_vel_targets_[0],joint_vel_targets_[1],joint_vel_targets_[2],joint_vel_targets_[3],
   joint_vel_targets_[4],joint_vel_targets_[5],joint_vel_targets_[6]);

  yumi_egm_interface_.setJointVelTargets(joint_vel_targets_);

  data_buffer_mutex_.unlock();
}
