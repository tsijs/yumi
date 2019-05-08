
#include <yumi_hw/yumi_hw_egm.h>

YumiHWEGM::YumiHWEGM()
    : is_initialized_(false) {}

YumiHWEGM::~YumiHWEGM() { 
  rws_interface_->stopEGM(); 
  yumi_egm_interface_r_->stop(); 
  yumi_egm_interface_l_->stop(); 
}

void YumiHWEGM::setup(const std::string &ip, const std::string &port_rws,
                      const int &port_egm_l, const int &port_egm_r) {
  ip_ = ip;
  port_rws_ = atoi(port_rws.c_str());
  port_egm_l_ = port_egm_l;
  port_egm_r_ = port_egm_r;

  // create instances of the interfaces
  // FIXME Note: The RWS interface is not thread safe (yet)! 
  rws_interface_ = std::make_shared<YumiRWSforEGMWrapper>(ip_, port_rws_);
  yumi_egm_interface_r_.reset(new YumiEGMInterface(SystemConstants::RAPID::TASK_ROB_R, port_egm_r_, rws_interface_));
  yumi_egm_interface_l_.reset(new YumiEGMInterface(SystemConstants::RAPID::TASK_ROB_L, port_egm_l_, rws_interface_));
}

bool YumiHWEGM::init() {
  if (is_initialized_) {
    ROS_WARN_STREAM("Called init on YumiHWEGM, but is already initialized.");
    return false;
  }
  current_strategy_ = JOINT_VELOCITY; // TODO: get this from parameter?
  ROS_WARN_STREAM("IP:" << ip_); // debug

  if (!rws_interface_->startEGM())
    return false;
    
  is_initialized_ = yumi_egm_interface_r_->initEGM() && yumi_egm_interface_l_->initEGM();

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

  yumi_egm_interface_r_->getCurrentJointStates(joint_pos_r_, joint_vel_r_);
  yumi_egm_interface_l_->getCurrentJointStates(joint_pos_l_, joint_vel_l_);

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
  switch (getControlStrategy()) {
  case JOINT_POSITION:
    for (int j = 0; j < n_joints_; j++) { 
      if (j < N_JOINTS_ARM)
        joint_targets_l_[j] = joint_position_command_[j]; 
      else
        joint_targets_r_[j] = joint_position_command_[j]; 
    }
    break;

  case JOINT_VELOCITY:
    // TODO: when the params are used at init --> control strategy, this needs to
    // be changed as well
    for (int j = 0; j < n_joints_; j++) { 
      if (j < N_JOINTS_ARM)
        joint_targets_l_[j] = joint_velocity_command_[j]; 
      else
        joint_targets_r_[j] = joint_velocity_command_[j]; 
    }
    
    break;
  default:
    break;
  }
  
  ROS_DEBUG("targets Left: %f, %f, %f, %f, %f, %f,"
  "%f",joint_targets_l_[0],joint_targets_l_[1],joint_targets_l_[2],joint_targets_l_[3],
  joint_targets_l_[4],joint_targets_l_[5],joint_targets_l_[6]);

  ROS_DEBUG("targets Right: %f, %f, %f, %f, %f, %f,"
  "%f",joint_targets_r_[0],joint_targets_r_[1],joint_targets_r_[2],joint_targets_r_[3],
  joint_targets_r_[4],joint_targets_r_[5],joint_targets_r_[6]);

  yumi_egm_interface_l_->setJointTargets(joint_targets_l_, getControlStrategy());
  yumi_egm_interface_r_->setJointTargets(joint_targets_r_, getControlStrategy());

  data_buffer_mutex_.unlock(); // FIXME: can I put this up to after the switch statement?
}