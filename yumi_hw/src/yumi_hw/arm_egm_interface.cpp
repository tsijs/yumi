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

#include <yumi_hw/arm_egm_interface.h>

#include <string>

#include <curl/curl.h>
#include <ros/ros.h>

#include <abb_libegm/egm_common.h>
#include <abb_libegm/egm_controller_interface.h>
#include <abb_librws/rws_common.h>

using namespace abb::egm;
using namespace abb::rws;
//TODO move default arguments to some general header?

YumiEGMInterface::YumiEGMInterface(const std::string task, const unsigned int egm_port, const std::string rws_ip,
                                   const unsigned int rws_port)
    : has_params_(false), task_(task), egm_port_(egm_port) {
  
  ROS_INFO("rws interface not passed to EGM Interface. This is bad if multiple EGM tasks are used in parallel.");
  rws_interface_.reset(new YumiRWSforEGMWrapper(rws_ip, rws_port));
  init();
}

YumiEGMInterface::YumiEGMInterface(std::string task, const unsigned int egm_port, const std::shared_ptr<YumiRWSforEGMWrapper>& rws_interface)
    : has_params_(false), rws_interface_(rws_interface), task_(task), egm_port_(egm_port) {

  init();
}

YumiEGMInterface::~YumiEGMInterface() {}

void YumiEGMInterface::init() { 
  // init that is shared over all constructors
  input_.reset(new ::wrapper::Input());
  status_.reset(new ::wrapper::Status()); //unused
  output_.reset(new ::wrapper::Output());
  getParams();

  if (!has_params_) {
    ROS_ERROR_STREAM(ros::this_node::getName()
                     << ": missing EGM/RWS parameters.");
    return;
  }
  
  if (!initRWS()) {
    ROS_ERROR_STREAM("RWS interface not ready. Initialization of EGM for task "<< task_ << " failed.");
  }
}

void YumiEGMInterface::getParams() {
  ros::NodeHandle nh("~");

  // RWS parameters


  // EGM parameters
  // with local variables due to problems with rapid type defs. 
  bool use_presync;
  nh.param("egm/use_presync", use_presync, false);
  egm_settings_.use_presync = use_presync;

  bool allow_egm_motions;
  nh.param("egm/allow_egm_motions", allow_egm_motions, true);
  egm_settings_.allow_egm_motions = allow_egm_motions;
  
  bool use_filtering;
  nh.param("egm/use_filtering", use_filtering, false);
  egm_settings_.setup_uc.use_filtering = use_filtering;

  double timeout;
  nh.param("egm/comm_timeout", timeout, 30.0);
  egm_settings_.setup_uc.comm_timeout = timeout;

  double sample_rate;
  nh.param("egm/sample_rate", sample_rate, 4.0); // only multiples of 4 allowed. 
  egm_settings_.activate.sample_rate = sample_rate;

  // std::string tool_name;
  // nh.param("egm/tool_name", tool_name, std::string("tool0"));
  // egm_settings_.activate.tool = tool_name;

  // std::string wobj_name;
  // nh.param("egm/wobj_name", wobj_name, std::string("wobj0"));
  // egm_settings_.wobj_name = wobj_name;

  double cond_min_max;
  nh.param("egm/cond_min_max", cond_min_max, 0.5);
  egm_settings_.activate.cond_min_max = cond_min_max;

  double lp_filter;
  nh.param("egm/lp_filter", lp_filter, 0.0);
  egm_settings_.activate.lp_filter = lp_filter;

  double max_speed_deviation;
  nh.param("egm/max_speed_deviation", max_speed_deviation, 10.0);
  egm_settings_.activate.max_speed_deviation = max_speed_deviation;

  double condition_time;
  nh.param("egm/condition_time", condition_time, 10.0);
  egm_settings_.run.cond_time = condition_time;

  double ramp_in_time;
  nh.param("egm/ramp_in_time", ramp_in_time, 0.1);
   egm_settings_.run.ramp_in_time = ramp_in_time;

  double pos_corr_gain;
  nh.param("egm/pos_corr_gain", pos_corr_gain, 0.0);
  egm_settings_.run.pos_corr_gain = pos_corr_gain;

  double ramp_out_time;
  nh.param("egm/ramp_out_time", ramp_out_time, 10.0);
  egm_settings_.stop.ramp_out_time = ramp_out_time;

  has_params_ = true;
}

bool YumiEGMInterface::stop() {
  io_service_.stop();
  io_service_threads_.join_all();
  return true;
}

void YumiEGMInterface::getCurrentJointStates(float (&joint_pos)[N_JOINTS_ARM],
                                             float (&joint_vel)[N_JOINTS_ARM]) {
  egm_interface_->waitForMessage();
  egm_interface_->read(input_.get());
  copyEGMInputToArray(&*input_, joint_pos, joint_vel);
}

void YumiEGMInterface::setJointTargets(
    const float (&joint_targets)[N_JOINTS_ARM], const YumiHW::ControlStrategy mode) {

  switch (mode) {
    case YumiHW::ControlStrategy::JOINT_VELOCITY:
      copyVelocityArrayToEGMOutput(joint_targets, output_.get());
      egm_interface_->write(*output_);
      break;
    case YumiHW::ControlStrategy::JOINT_POSITION:
      copyPositionArrayToEGMOutput(joint_targets, output_.get());
      egm_interface_->write(*output_);
      break;
    default:
      break;
  }
}

void YumiEGMInterface::copyEGMInputToArray( ::wrapper::Input *const input,
                                            float *const joint_pos,
                                            float *const joint_vel) const {

  for (int i = 0; i < N_JOINTS_ARM - 1; ++i) {
    joint_vel[i] = (float)input->mutable_feedback()
                       ->mutable_robot()
                       ->mutable_joints()
                       ->mutable_velocity()
                       ->values(i) / 180.0 * M_PI;
    joint_pos[i] = (float)input->mutable_feedback()
                       ->mutable_robot()
                       ->mutable_joints()
                       ->mutable_position()
                       ->values(i) / 180.0 * M_PI;
  }

  // FIXME: this external stuff should not be necessary anymore here with Seven
  // declaration?
  joint_vel[N_JOINTS_ARM - 1] = (float)input->mutable_feedback()
                                    ->mutable_external()
                                    ->mutable_joints()
                                    ->mutable_velocity()
                                    ->values(0) / 180.0 * M_PI;
  joint_pos[N_JOINTS_ARM - 1] = (float)input->mutable_feedback()
                                    ->mutable_external()
                                    ->mutable_joints()
                                    ->mutable_position()
                                    ->values(0) / 180.0 * M_PI;
}

void YumiEGMInterface::copyVelocityArrayToEGMOutput( const float *const joint_array,
                                             ::wrapper::Output *const output) const {
  ROS_DEBUG("TESTING IF THIS GETS REACHED TOO, mutable vel output no. = %i",
            output->mutable_robot()
                ->mutable_joints()
                ->mutable_velocity()
                ->values_size());
  if (output->mutable_robot()
          ->mutable_joints()
          ->mutable_velocity()
          ->values_size() > N_JOINTS_ARM - 1) // prevents seg fault
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
    // ROS_INFO("TESTING IF THIS GETS REACHED 2");
    output->mutable_external()
        ->mutable_joints()
        ->mutable_velocity()
        ->set_values(0, (double)joint_array[2] * 180.0 / M_PI);
  }
}

// TODO test this
void YumiEGMInterface::copyPositionArrayToEGMOutput( const float *const joint_array,
                             ::wrapper::Output *const output) const {
ROS_DEBUG("TESTING IF THIS GETS REACHED TOO, mutable pos output no. = %i",
            output->mutable_robot()
                ->mutable_joints()
                ->mutable_position()
                ->values_size());
  if (output->mutable_robot()
          ->mutable_joints()
          ->mutable_position()
          ->values_size() > N_JOINTS_ARM - 1) // prevents seg fault
  {
    ROS_INFO("TESTING IF THIS GETS REACHED 1. POS");
    output->mutable_robot()->mutable_joints()->mutable_position()->set_values(
        0, (double)joint_array[0] * 180.0 / M_PI);
    output->mutable_robot()->mutable_joints()->mutable_position()->set_values(
        1, (double)joint_array[1] * 180.0 / M_PI);
    output->mutable_robot()->mutable_joints()->mutable_position()->set_values(
        2, (double)joint_array[3] * 180.0 / M_PI);
    output->mutable_robot()->mutable_joints()->mutable_position()->set_values(
        3, (double)joint_array[4] * 180.0 / M_PI);
    output->mutable_robot()->mutable_joints()->mutable_position()->set_values(
        4, (double)joint_array[5] * 180.0 / M_PI);
    output->mutable_robot()->mutable_joints()->mutable_position()->set_values(
        5, (double)joint_array[6] * 180.0 / M_PI);
  }
  if (output->mutable_external()
          ->mutable_joints()
          ->mutable_position()
          ->values_size() > 0) // prevents seg fault
  {
    ROS_INFO("TESTING IF THIS GETS REACHED 2. POS");
    output->mutable_external()
        ->mutable_joints()
        ->mutable_position()
        ->set_values(0, (double)joint_array[2] * 180.0 / M_PI);
  }
}

bool YumiEGMInterface::initRWS() {

  // rws_interface_.reset(new RWSInterfaceYuMi(rws_ip_, rws_port_));
  
  ros::Duration(rws_interface_->getDelayTime()).sleep();

  // Check that RAPID is running on the robot and that robot is in AUTO mode
  // tribool problem
  
  if (!rws_interface_->isRAPIDRunning().isTrue()) {
    ROS_ERROR_STREAM(ros::this_node::getName() << ": robot unavailable, "
  "RAPIDRunning==false. Make sure that the RAPID program is running on the flexpendant.");
    return false;
  }
  std::cout << "  RAPID running" << std::endl;
  ros::Duration(rws_interface_->getDelayTime()).sleep();

  if (!rws_interface_->isAutoMode().isTrue()) {
    ROS_ERROR_STREAM(ros::this_node::getName() << ": robot unavailable, "
  "AUTOMode==false. Make sure to set the robot to AUTO mode on the flexpendant.");
    return false;
  }

  std::cout << "  Auto mode" << std::endl;
  ros::Duration(rws_interface_->getDelayTime()).sleep();

  if (!sendEGMParams()) {
    return false;
  }

  std::cout << "Connection ready" << std::endl;
  ros::Duration(rws_interface_->getDelayTime()).sleep();

  return true;
}

bool YumiEGMInterface::initOutput(
    std::shared_ptr<abb::egm::wrapper::Output> &output,
    const std::shared_ptr<abb::egm::wrapper::Input> &input) {
  output->Clear();
  output->mutable_robot()->mutable_joints()->mutable_velocity()->CopyFrom(
      input->feedback().robot().joints().velocity());
  output->mutable_external()->mutable_joints()->mutable_velocity()->CopyFrom(
      input->feedback().external().joints().velocity());
}

bool YumiEGMInterface::initEGM() {
  // TODO: call configureEGM here and get this from params
  BaseConfiguration configuration;
  configuration.use_logging = true;
  configuration.use_velocity_outputs = true; 
  


  egm_interface_.reset(
      new EGMControllerInterface(io_service_, egm_port_, configuration));
  std::cout << "  EGM init started" << std::endl;
  
  // configureEGM(egm_interface_);

  // create threads for EGM communication
  for (size_t i = 0; i < MAX_NUMBER_OF_EGM_CONNECTIONS; i++) {
    io_service_threads_.create_thread(
        boost::bind(&boost::asio::io_service::run, &io_service_));
  }

  bool wait = true;
  ROS_INFO("Wait for an EGM communication session to start...");
  while (ros::ok() && wait) {
    ROS_INFO_STREAM("The interface " << task_ << " is " <<
             (egm_interface_->isConnected() ? "connected" : "not connected"));
    if (egm_interface_->isConnected()) {
      if (egm_interface_->getStatus().rapid_execution_state() ==
          abb::egm::wrapper::Status_RAPIDExecutionState_RAPID_UNDEFINED) {
        ROS_WARN("RAPID execution state is UNDEFINED for arm (might happen "
                 "first time after controller start/restart). Try to restart "
                 "the RAPID program.");
      } else {
        wait = egm_interface_->getStatus().rapid_execution_state() !=
               abb::egm::wrapper::Status_RAPIDExecutionState_RAPID_RUNNING;
        ROS_INFO("The wait bool = %s", wait ? "true" : "false");
      }
    }
    ros::Duration(0.5).sleep();
  }

  egm_interface_->read(input_.get()); // first read for init output (might be a hack)

  initOutput(output_, input_);
  return true;
}


bool YumiEGMInterface::sendEGMParams() {
  YumiRWSforEGMWrapper::EGMSettings egm_setting_rapid;
  
  bool success = rws_interface_->getSettings(task_, &egm_setting_rapid);
      

  ROS_INFO_STREAM("  get EGM Parameters"
                  << "\n");
  ROS_INFO_STREAM("    success? " << success << "\n");
  ROS_INFO_STREAM("    egm_data: " << egm_setting_rapid.constructString()
                                   << "\n");

  if (!success) {
    ROS_ERROR_STREAM(ros::this_node::getName()
                     << ": robot unavailable, make sure to set the robot to "
                        "AUTO mode on the flexpendant.");
    return false;
  }

  getEGMParams(egm_setting_rapid);

  success = rws_interface_->setSettings(task_, egm_setting_rapid);

  if (success) {
    ROS_INFO("EGM parameters correctly set.");
    return true;
  }

  return false;
}

void YumiEGMInterface::getEGMParams(YumiRWSforEGMWrapper::EGMSettings& egm_settings) {
  egm_settings.allow_egm_motions  =   egm_settings_.allow_egm_motions ;
  egm_settings.use_presync =   egm_settings_.use_presync;
  egm_settings.setup_uc.use_filtering =   egm_settings_.setup_uc.use_filtering;
  egm_settings.setup_uc.comm_timeout =   egm_settings_.setup_uc.comm_timeout;
  // egm_settings.activate.tool =   egm_settings_.activate.tool; // this is not used for now. It seems to me that this should just be et properly in RS
  // egm_settings.activate.wobj =   egm_settings_.activate.wobj; // and not get touched from here?
  egm_settings.activate.correction_frame =   egm_settings_.activate.correction_frame;
  egm_settings.activate.sensor_frame =   egm_settings_.activate.sensor_frame;
  egm_settings.activate.cond_min_max =   egm_settings_.activate.cond_min_max;
  egm_settings.activate.lp_filter =   egm_settings_.activate.lp_filter;
  egm_settings.activate.sample_rate =   egm_settings_.activate.sample_rate;
  egm_settings.activate.max_speed_deviation =   egm_settings_.activate.max_speed_deviation;
  egm_settings.run.cond_time =   egm_settings_.run.cond_time;
  egm_settings.run.ramp_in_time =   egm_settings_.run.ramp_in_time;
  egm_settings.run.offset =   egm_settings_.run.offset;
  egm_settings.run.pos_corr_gain =   egm_settings_.run.pos_corr_gain;
  egm_settings.stop.ramp_out_time =   egm_settings_.stop.ramp_out_time;
}

void YumiEGMInterface::configureEGM(const 
    std::shared_ptr<EGMControllerInterface>& egm_interface) { 
  BaseConfiguration configuration;
  // make this come from parameter server /  update settings to new structure
  configuration.use_logging = true;
  configuration.use_velocity_outputs = true;
  // configuration.basic.use_conditions = false;
  // configuration.axes = EGMInterfaceConfiguration::Seven;
  // configuration.basic.execution_mode = EGMInterfaceConfiguration::Direct;
  egm_interface->setConfiguration(configuration);
}

YumiRWSforEGMWrapper::YumiRWSforEGMWrapper(const std::string ip_address, const unsigned short port):
    RWSStateMachineInterface(ip_address, port) {
  getParams();

}


void YumiRWSforEGMWrapper::getParams() {
  ros::NodeHandle nh("~");
  nh.param("rws/delay_time", rws_delay_time_, 1.0);
  nh.param("rws/max_signal_retries", max_signal_retries_, 5);
}

bool YumiRWSforEGMWrapper::startEGM() {
  bool egm_started = false;

  for (int i = 0; i < max_signal_retries_ && !egm_started; ++i) {
    boost::mutex::scoped_lock lock(rws_mutex_);
    egm_started = services().egm().signalEGMStartJoint();

    if (!egm_started) {
      ROS_ERROR_STREAM(ros::this_node::getName()
                        << ": failed to send EGM start signal! [Attempt "
                         << i + 1 << "/" << max_signal_retries_ << "]");
    }
  }


  return egm_started;
}

bool YumiRWSforEGMWrapper::getSettings(const std::string task, EGMSettings* p_settings) {
  bool result = services().egm().getSettings(task, p_settings);

  return result;
}

bool YumiRWSforEGMWrapper::setSettings(const std::string task, EGMSettings settings) {
  boost::mutex::scoped_lock lock(rws_mutex_);
  bool result = services().egm().setSettings( task, settings);

  return result;
}

bool YumiRWSforEGMWrapper::stopEGM() {
  bool egm_stopped = false;

  for (int i = 0; i < max_signal_retries_ && !egm_stopped; ++i) {
    boost::mutex::scoped_lock lock(rws_mutex_);
    egm_stopped = services().egm().signalEGMStop();
    
    if (!egm_stopped) {
      ROS_ERROR_STREAM(ros::this_node::getName()
                        << ": failed to send EGM stop signal! [Attempt "
                        << i + 1 << "/" << max_signal_retries_ << "]");
    }
  }


  return egm_stopped;
}