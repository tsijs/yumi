/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017, Francisco Vina, francisco.vinab@gmail.com
 *               2018, Yoshua Nava, yoshua.nava.chocron@gmail.com
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *       * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *       * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *       * Neither the name of the Southwest Research Institute, nor the names
 *       of its contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "yumi_hw/yumi_hw_egm.h"

#include <string>

#include <curl/curl.h>
#include <ros/ros.h>

#include <abb_libegm/egm_common.h>
#include <abb_libegm/egm_controller_interface.h>
#include <abb_librws/rws_common.h>

using namespace abb::egm;
using namespace abb::rws;

YumiEGMInterface::YumiEGMInterface(const double &exponential_smoothing_alpha)
    : has_params_(false), rws_connection_ready_(false) {
  left_arm_input_.reset(new ::wrapper::Input());
  left_arm_status_.reset(new ::wrapper::Status());
  right_arm_input_.reset(new ::wrapper::Input());
  right_arm_status_.reset(new ::wrapper::Status());

  left_arm_output_.reset(new ::wrapper::Output());
  right_arm_output_.reset(new ::wrapper::Output());

  // preallocate memory for feedback/command messages
  // initEGMFeedbackMessage(left_arm_input_->mutable_feedback());
  // initEGMFeedbackMessage(right_arm_input_->mutable_feedback());

  // initEGMOutputMessage(left_arm_joint_vel_targets_->mutable_velocity(),
  // left_arm_joint_vel_targets_->mutable_external_speed());
  // initEGMOutputMessage(right_arm_joint_vel_targets_->mutable_velocity(),
  // right_arm_joint_vel_targets_->mutable_external_speed());

  getParams();
}

YumiEGMInterface::~YumiEGMInterface() {}

void YumiEGMInterface::getParams() {
  ros::NodeHandle nh("~");

  // RWS parameters
  nh.param("rws/delay_time", rws_delay_time_, 1.0);
  nh.param("rws/max_signal_retries", rws_max_signal_retries_, 5);

  // EGM parameters
  double timeout;
  nh.param("egm/comm_timeout", timeout, 30.0);
  egm_activate_params_.comm_timeout = timeout;

  std::string tool_name;
  nh.param("egm/tool_name", tool_name, std::string("tool0"));
  egm_activate_params_.tool_name = tool_name;

  std::string wobj_name;
  nh.param("egm/wobj_name", wobj_name, std::string("wobj0"));
  egm_activate_params_.wobj_name = wobj_name;

  double cond_min_max;
  nh.param("egm/cond_min_max", cond_min_max, 0.5);
  egm_activate_params_.cond_min_max = cond_min_max;

  double lp_filter;
  nh.param("egm/lp_filter", lp_filter, 0.0);
  egm_activate_params_.lp_filter = lp_filter;

  double max_speed_deviation;
  nh.param("egm/max_speed_deviation", max_speed_deviation, 400.0);
  egm_activate_params_.max_speed_deviation = max_speed_deviation;

  double condition_time;
  nh.param("egm/condition_time", condition_time, 10.0);
  egm_run_params_.cond_time = condition_time;

  double ramp_in_time;
  nh.param("egm/ramp_in_time", ramp_in_time, 0.1);
  egm_run_params_.ramp_in_time = ramp_in_time;

  double pos_corr_gain;
  nh.param("egm/pos_corr_gain", pos_corr_gain, 0.0);
  egm_run_params_.pos_corr_gain = pos_corr_gain;

  has_params_ = true;
}

bool YumiEGMInterface::init(const std::string &ip, const unsigned short &port,
                            const int &port_l,
                            const int &port_r) { // TODO add extra port info
  if (!has_params_) {
    ROS_ERROR_STREAM(ros::this_node::getName()
                     << ": missing EGM/RWS parameters.");
    return false;
  }

  rws_ip_ = ip;
  rws_port_ = port;
  egm_port_left_ = port_l;
  egm_port_right_ = port_r;

  if (!initRWS()) {
    return false;
  }

  if (!initEGM()) {
    return false;
  }

  return true;
}

bool YumiEGMInterface::stop() {
  if (!stopEGM()) {
    return false;
  }

  io_service_.stop();
  io_service_threads_.join_all();

  return true;
}

void YumiEGMInterface::getCurrentJointStates(
    float (&joint_pos)[N_YUMI_JOINTS], float (&joint_vel)[N_YUMI_JOINTS]) {
  left_arm_egm_interface_->waitForMessage();
  left_arm_egm_interface_->read(left_arm_input_.get());

  right_arm_egm_interface_->waitForMessage();
  right_arm_egm_interface_->read(right_arm_input_.get());

  copyEGMInputToArray(&*left_arm_input_, joint_pos, joint_vel);
  copyEGMInputToArray(&*right_arm_input_, &joint_pos[7], &joint_vel[7]);
}

void YumiEGMInterface::setJointVelTargets(
    float (&joint_vel_targets)[N_YUMI_JOINTS]) {
  copyArrayToEGMOutput(joint_vel_targets, left_arm_output_.get());
  copyArrayToEGMOutput(&joint_vel_targets[7], right_arm_output_.get());

  left_arm_egm_interface_->write(*left_arm_output_);
  right_arm_egm_interface_->write(*right_arm_output_);
}

void YumiEGMInterface::copyEGMInputToArray(::wrapper::Input *input,
                                           float *joint_pos,
                                           float *joint_vel) const {

  for (int i = 0; i < 6; ++i) {
    joint_vel[i] = (float)input->mutable_feedback()
                       ->mutable_robot()
                       ->mutable_joints()
                       ->mutable_velocity()
                       ->values(i);
    joint_pos[i] = (float)input->mutable_feedback()
                       ->mutable_robot()
                       ->mutable_joints()
                       ->mutable_position()
                       ->values(i);
  }
  joint_vel[6] = (float)input->mutable_feedback()
                     ->mutable_external()
                     ->mutable_joints()
                     ->mutable_velocity()
                     ->values(0);
  joint_pos[6] = (float)input->mutable_feedback()
                     ->mutable_external()
                     ->mutable_joints()
                     ->mutable_position()
                     ->values(0);
}

void YumiEGMInterface::copyArrayToEGMOutput(float *joint_array,
                                            ::wrapper::Output *output) const {
  ROS_DEBUG("TESTING IF THIS GETS REACHED TOO, mutable vel output no. = %i",
            output->mutable_robot()
                ->mutable_joints()
                ->mutable_velocity()
                ->values_size());
  if (output->mutable_robot()
          ->mutable_joints()
          ->mutable_velocity()
          ->values_size() > 3) // prevents seg fault
  {
    ROS_INFO("TESTING IF THIS GETS REACHED 1");
    output->mutable_robot()->mutable_joints()->mutable_velocity()->set_values(
        0, (double)joint_array[0] * 180.0 / M_PI);
    output->mutable_robot()->mutable_joints()->mutable_velocity()->set_values(
        1, (double)joint_array[1] * 180.0 / M_PI);
    output->mutable_robot()->mutable_joints()->mutable_velocity()->set_values(
        2, (double)joint_array[3] * 180.0 / M_PI);
    output->mutable_robot()->mutable_joints()->mutable_velocity()->set_values(
        3, (double)joint_array[4] * 180.0 / M_PI);
    output->mutable_robot()->mutable_joints()->mutable_velocity()->set_values(
        4, (double)joint_array[5] * 180.0 / M_PI);
    output->mutable_robot()->mutable_joints()->mutable_velocity()->set_values(
        5, (double)joint_array[6] * 180.0 / M_PI);
  }
  if (output->mutable_external()
          ->mutable_joints()
          ->mutable_velocity()
          ->values_size() > 0) // prevents seg fault
  {
    ROS_INFO("TESTING IF THIS GETS REACHED 2");
    output->mutable_external()
        ->mutable_joints()
        ->mutable_velocity()
        ->set_values(0, (double)joint_array[2] * 180.0 / M_PI);
  }
}

bool YumiEGMInterface::initRWS() {
  ROS_INFO_STREAM(ros::this_node::getName()
                  << " starting RWS connection with IP & PORT: " << rws_ip_
                  << " / " << rws_port_);

  // rws_interface_.reset(new RWSInterfaceYuMi(rws_ip_, rws_port_));
  rws_interface_.reset(new RWSSimpleStateMachineInterface(rws_ip_, rws_port_));
  ros::Duration(rws_delay_time_).sleep();
  std::cout << "  RWS interface created" << std::endl;

  // Check that RAPID is running on the robot and that robot is in AUTO mode
  // tribool problem
  /*
  if(!rws_interface_->isRAPIDRunning())
  {
    ROS_ERROR_STREAM(ros::this_node::getName() << ": robot unavailable, make
  sure that the RAPID program is running on the flexpendant.");
    return false;
  }
    std::cout << "  RAPID running" << std::endl;
  ros::Duration(rws_delay_time_).sleep();

  if(!rws_interface_->isAutoMode())
  {
    ROS_ERROR_STREAM(ros::this_node::getName() << ": robot unavailable, make
  sure to set the robot to AUTO mode on the flexpendant.");
    return false;
  }
    std::cout << "  Auto mode" << std::endl;
  ros::Duration(rws_delay_time_).sleep();

  if(!sendEGMParams())
  {
    return false;
  }
*/
  rws_connection_ready_ = true;
  std::cout << "Connection ready" << std::endl;
  ros::Duration(rws_delay_time_).sleep();

  if (!startEGM())
    return false;

  // ros::NodeHandle nh;
  // rws_watchdog_timer_ = nh.createTimer(ros::Duration(rws_watchdog_period_),
  // &YumiEGMInterface::rwsWatchdog, this);

  return true;
}

bool YumiEGMInterface::initEGMArm(
    boost::shared_ptr<EGMControllerInterface> &interface) {
  bool wait = true;
  ROS_INFO("left: Wait for an EGM communication session to start...");
  while (ros::ok() && wait) {
    ROS_INFO("The interface is: %s.\n",
             interface->isConnected() ? "connected" : "not connected");
    if (interface->isConnected()) {
      if (interface->getStatus().rapid_execution_state() ==
          abb::egm::wrapper::Status_RAPIDExecutionState_RAPID_UNDEFINED) {
        ROS_WARN("RAPID execution state is UNDEFINED for arm (might happen "
                 "first time after controller start/restart). Try to restart "
                 "the RAPID program.");
      } else {
        wait = interface->getStatus().rapid_execution_state() !=
               abb::egm::wrapper::Status_RAPIDExecutionState_RAPID_RUNNING;
        ROS_INFO("The wait bool = %s", wait ? "true" : "false");
      }
    }
    ros::Duration(0.5).sleep();
  }
  left_arm_egm_interface_->read(left_arm_input_.get());
}

bool YumiEGMInterface::initOutput(
    boost::shared_ptr<abb::egm::wrapper::Output> &output,
    boost::shared_ptr<abb::egm::wrapper::Input> &input) {
  output->Clear();
  output->mutable_robot()->mutable_joints()->mutable_velocity()->CopyFrom(
      input->feedback().robot().joints().velocity());
  output->mutable_external()->mutable_joints()->mutable_velocity()->CopyFrom(
      input->feedback().external().joints().velocity());
}

bool YumiEGMInterface::initEGM() {
  BaseConfiguration configuration;
  configuration.use_logging = true;
  configuration.use_velocity_outputs = true;
  left_arm_egm_interface_.reset(new EGMControllerInterface(
      io_service_, egm_port_left_, configuration)); // egm_port_left_
  right_arm_egm_interface_.reset(new EGMControllerInterface(
      io_service_, egm_port_right_, configuration)); // egm_port_right_
  std::cout << "  EGM init started" << std::endl;
  // create threads for EGM communication
  for (size_t i = 0; i < MAX_NUMBER_OF_EGM_CONNECTIONS; i++) {
    io_service_threads_.create_thread(
        boost::bind(&boost::asio::io_service::run, &io_service_));
  }

  initEGMArm(left_arm_egm_interface_);
  initEGMArm(right_arm_egm_interface_);
  left_arm_egm_interface_->read(left_arm_input_.get());
  right_arm_egm_interface_->read(right_arm_input_.get());
  initOutput(left_arm_output_, left_arm_input_);
  initOutput(right_arm_output_, right_arm_input_);

  return true;
}

#if 0
bool YumiEGMInterface::sendEGMParams()
{
  /*
  EGMRunData left_arm_egm_run_params, right_arm_egm_run_params;
  EGMActivateData left_arm_egm_act_params, right_arm_egm_act_params;
  bool right_arm_success = rws_interface_->getRAPIDSymbolData(SystemConstants::RAPID::TASK_ROB_L,
                                                              RWSSimpleStateMachineInterface::ProgramConstants::RAPID::Symbols::RAPID_EGM_DATA,
                                                              &right_arm_egm_act_params);
  if (right_arm_success)
  {
    bool right_arm_success = rws_interface_->getRAPIDSymbolData(SystemConstants::RAPID::TASK_ROB_L,
                                                            RWSSimpleStateMachineInterface::ProgramConstants::RAPID::Symbols::RAPID_EGM_DATA,
                                                            &right_arm_egm_run_params);
  }

  bool left_arm_success = rws_interface_->getRAPIDSymbolData(SystemConstants::RAPID::TASK_ROB_R,
                                                             RWSSimpleStateMachineInterface::ProgramConstants::RAPID::Symbols::RAPID_EGM_DATA,
                                                             &left_arm_egm_act_params);
  if (left_arm_success)
  {
    bool left_arm_success = rws_interface_->getRAPIDSymbolData(SystemConstants::RAPID::TASK_ROB_R,
                                                             RWSSimpleStateMachineInterface::ProgramConstants::RAPID::Symbols::RAPID_EGM_DATA,
                                                             &left_arm_egm_run_params);                                                         
  }
  
    std::cout << "  Left arm EGM Parameters" << std::endl;
    std::cout << "    success? " << right_arm_success << std::endl;
    std::cout << "    egm_data: " << left_arm_egm_params.constructRWSValueString() << std::endl;
    std::cout << "  Right arm EGM Parameters" << std::endl;
    std::cout << "    success? " << left_arm_success << std::endl;
    std::cout << "    egm_data: " << right_arm_egm_params.constructRWSValueString() << std::endl;

  if(!left_arm_success || !right_arm_success)
  {
    ROS_ERROR_STREAM(ros::this_node::getName() << ": robot unavailable, make sure to set the robot to AUTO mode on the flexpendant.");
    return false;
  }

  setEGMParams(&left_arm_egm_params);
  setEGMParams(&right_arm_egm_params);

  left_arm_success = rws_interface_->setRAPIDSymbolData(SystemConstants::RAPID::TASK_ROB_L,
                                                        RWSSimpleStateMachineInterface::ProgramConstants::RAPID::Symbols::RAPID_EGM_DATA,
                                                        left_arm_egm_params.);
  right_arm_success = rws_interface_->setRAPIDSymbolData(SystemConstants::RAPID::TASK_ROB_R,
                                                         RWSSimpleStateMachineInterface::ProgramConstants::RAPID::Symbols::RAPID_EGM_DATA,
                                                         right_arm_egm_params);

  if(left_arm_success && right_arm_success)
  {
    ROS_INFO("EGM parameters correctly set.");
    return true;
  }

  return false;
  */
  return true;
}

void YumiEGMInterface::setEGMParams(EGMData* egm_data)
{
  egm_data->comm_timeout = egm_params_.comm_timeout;
  egm_data->tool_name = egm_params_.tool_name;
  egm_data->wobj_name = egm_params_.wobj_name;
  egm_data->cond_min_max = egm_params_.cond_min_max;
  egm_data->lp_filter = egm_params_.lp_filter;
  egm_data->max_speed_deviation = egm_params_.max_speed_deviation;
  egm_data->cond_time = egm_params_.cond_time;
  egm_data->ramp_in_time = egm_params_.ramp_in_time;
  egm_data->pos_corr_gain = egm_params_.pos_corr_gain;
}
#endif

void YumiEGMInterface::configureEGM(
    boost::shared_ptr<EGMControllerInterface> egm_interface) { // Unused
  BaseConfiguration configuration;
  configuration.use_logging = true;
  configuration.use_velocity_outputs = true;

  // configuration.basic.use_conditions = false;
  // configuration.axes = EGMInterfaceConfiguration::Seven;
  // configuration.basic.execution_mode = EGMInterfaceConfiguration::Direct;

  egm_interface->setConfiguration(configuration);
}

bool YumiEGMInterface::startEGM() {
  bool egm_started = false;

  if (rws_interface_ && rws_connection_ready_) {
    for (int i = 0; i < rws_max_signal_retries_ && !egm_started; ++i) {
      egm_started = rws_interface_->signalEGMStartJoint();

      if (!egm_started) {
        ROS_ERROR_STREAM(ros::this_node::getName()
                         << ": failed to send EGM start signal! [Attempt "
                         << i + 1 << "/" << rws_max_signal_retries_ << "]");
      }
    }
  }

  return egm_started;
}

bool YumiEGMInterface::stopEGM() {
  bool egm_stopped = false;

  if (rws_interface_ && rws_connection_ready_) {
    for (int i = 0; i < rws_max_signal_retries_ && !egm_stopped; ++i) {
      egm_stopped = rws_interface_->signalEGMStop();
      if (!egm_stopped) {
        ROS_ERROR_STREAM(ros::this_node::getName()
                         << ": failed to send EGM stop signal! [Attempt "
                         << i + 1 << "/" << rws_max_signal_retries_ << "]");
      }
    }
  }

  return egm_stopped;
}

YumiHWEGM::YumiHWEGM(const double &exponential_smoothing_alpha)
    : YumiHW(exponential_smoothing_alpha), is_initialized_(false) {}

YumiHWEGM::~YumiHWEGM() { yumi_egm_interface_.stop(); }

void YumiHWEGM::setup(const std::string &ip, const std::string &port,
                      const int &port_l, const int &port_r) {
  ip_ = ip;
  port_ = atoi(port.c_str());
  port_l_ = port_l;
  port_r_ = port_r;
}

bool YumiHWEGM::init() {
  if (is_initialized_) {
    return false;
  }
  current_strategy_ = JOINT_VELOCITY;

  bool success = yumi_egm_interface_.init(ip_, port_, port_l_, port_r_);

  if (!success) {
    return false;
  }

  is_initialized_ = true;
  return true;
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

  //ROS_INFO("The received positions are: %f, %f, %f, %f, %f, %f, %f ", \
  joint_position_[0], joint_position_[1], joint_position_[2], joint_position_[3], joint_position_[4], joint_position_[5], joint_position_[6]);
  //ROS_INFO("The received velocities are: %f, %f, %f, %f, %f, %f, %f ", \
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

  for (int j = 0; j < n_joints_; j++) {
    joint_vel_targets_[j] =
        joint_velocity_command_[j]; // where is command set? controller manager?
  }
  //  ROS_INFO("targets: %f, %f, %f, %f, %f, %f,
  //  %f",joint_vel_targets_[0],joint_vel_targets_[1],joint_vel_targets_[2],joint_vel_targets_[3],
  //  joint_vel_targets_[4],joint_vel_targets_[5],joint_vel_targets_[6]);
  yumi_egm_interface_.setJointVelTargets(joint_vel_targets_);

  data_buffer_mutex_.unlock();
}
