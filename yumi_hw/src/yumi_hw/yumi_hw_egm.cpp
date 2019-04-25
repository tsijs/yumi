
#include <yumi_hw/yumi_hw_egm.h>

YumiHWEGM::YumiHWEGM(const double &exponential_smoothing_alpha)
    : YumiHW(exponential_smoothing_alpha), is_initialized_(false) {}

YumiHWEGM::~YumiHWEGM() { yumi_egm_interface_r_.stop(); yumi_egm_interface_l_.stop(); }

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
  for (int j = 0; j < N_JOINTS_ARM; j++) {
    joint_position_prev_r_[j] = joint_pos_r_[j]; 
    joint_velocity_prev_r_[j] = joint_vel_r_[j]; 
    joint_position_prev_l_[j] = joint_pos_l_[j]; 
    joint_velocity_prev_l_[j] = joint_vel_l_[j]; 
  }

  yumi_egm_interface_r_.getCurrentJointStates(joint_pos_r_, joint_vel_r_);
  yumi_egm_interface_l_.getCurrentJointStates(joint_pos_l_, joint_vel_l_);

  data_buffer_mutex_.unlock();

  ROS_DEBUG("The received positions right are: %f, %f, %f, %f, %f, %f, %f ", \
  joint_pos_r_[0], joint_pos_r_[1], joint_pos_r_[2], joint_pos_r_[3], joint_pos_r_[4], joint_pos_r_[5], joint_pos_r_[6]);
  ROS_DEBUG("The received velocities right are: %f, %f, %f, %f, %f, %f, %f ", \
  joint_vel_r_[0], joint_vel_r_[1], joint_vel_r_[2], joint_vel_r_[3], joint_vel_r_[4], joint_vel_r_[5], joint_vel_r_[6] );
  ROS_DEBUG("The received positions left are: %f, %f, %f, %f, %f, %f, %f ", \
  joint_pos_l_[0], joint_pos_l_[1], joint_pos_l_[2], joint_pos_l_[3], joint_pos_l_[4], joint_pos_l_[5], joint_pos_l_[6]);
  ROS_DEBUG("The received velocities left are: %f, %f, %f, %f, %f, %f, %f ", \
  joint_vel_l_[0], joint_vel_l_[1], joint_vel_l_[2], joint_vel_l_[3], joint_vel_l_[4], joint_vel_l_[5], joint_vel_l_[6] );

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
  for (int j = 0; j < n_joints_; j++) { // TODO add a switch statement on which policy is used
    if (j < N_JOINTS_ARM)
      joint_vel_targets_l_[j] = joint_velocity_command_[j]; 
    else
      joint_vel_targets_r_[j] = joint_velocity_command_[j]; 
  }
   ROS_DEBUG("targets: %f, %f, %f, %f, %f, %f,"
   "%f",joint_vel_targets_l_[0],joint_vel_targets_l_[1],joint_vel_targets_l_[2],joint_vel_targets_l_[3],
   joint_vel_targets_l_[4],joint_vel_targets_l_[5],joint_vel_targets_l_[6]);
   ROS_DEBUG("targets: %f, %f, %f, %f, %f, %f,"
   "%f",joint_vel_targets_r_[0],joint_vel_targets_r_[1],joint_vel_targets_r_[2],joint_vel_targets_r_[3],
   joint_vel_targets_r_[4],joint_vel_targets_r_[5],joint_vel_targets_r_[6]);

  yumi_egm_interface_l_.setJointVelTargets(joint_vel_targets_l_);
  yumi_egm_interface_r_.setJointVelTargets(joint_vel_targets_r_);

  data_buffer_mutex_.unlock();
}
