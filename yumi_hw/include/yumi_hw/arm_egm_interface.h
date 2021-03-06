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

#ifndef YUMI_HW_EGM_H
#define YUMI_HW_EGM_H

#include <yumi_hw/yumi_hw.h>

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <abb_libegm/egm_controller_interface.h>
#include <abb_librws/rws_interface.h>
#include <abb_librws/rws_state_machine_interface.h>

#include <ros/ros.h>

#ifndef MAX_NUMBER_OF_EGM_CONNECTIONS
#define MAX_NUMBER_OF_EGM_CONNECTIONS 2 // per arm in this case. Actually 4 I think? TODO: get from ros param. 
#endif

using namespace abb::egm;
using namespace abb::rws;

// wrapper for RWS interface used for EGM. 
class YumiRWSforEGMWrapper: public RWSStateMachineInterface {
  public:
    // TODO 
    // - make rws_max_tries a parameter?
    // - have more constructor mappings with username and passwords as well?
    // - make mutex locks less conservative? But maybe not..
    YumiRWSforEGMWrapper(const std::string ip_address, const unsigned short port);
    
    ~YumiRWSforEGMWrapper() {}

    bool startEGM();
    bool stopEGM();
    void getParams();
    bool setSettings(const std::string task, EGMSettings settings);
    bool getSettings(const std::string task, EGMSettings* p_settings);
    double getDelayTime() {return rws_delay_time_;}

  private:
    int max_signal_retries_;
    double rws_delay_time_;
    boost::mutex rws_mutex_;
};

// Wrapper class for setting up EGM connection to the Yumi robot
// with their corresponding IO service threads
class YumiEGMInterface {

public:
  YumiEGMInterface(const std::string task, const unsigned int egm_port, const std::string rws_ip=std::string("192.168.125.1"),
                                   const unsigned int rws_port=80); 
  YumiEGMInterface(std::string task, const unsigned int egm_port, const std::shared_ptr<YumiRWSforEGMWrapper>& rws_interface);
  ~YumiEGMInterface();
  // TODO: update these comments. 
  /** \brief Gets parameters for EGM & RWS connections from the ROS parameter
   * server.
   * The RAPID module TRobEGM.sys contains a brief description of the EGM
   * parameters:
   * LOCAL RECORD EGM_RECORD
   *   num comm_timeout;         ! Communication timeout [s].
   *
   *   string tool_name;         ! Name of a defined tool (e.g. "tool0").
   *   string wobj_name;         ! Name of a defined work object (e.g. "wobj0").
   *   num cond_min_max;         ! Condition value [deg or mm] for when the EGM
   * correction is considered
   *                             ! to be finished. E.g. for joint mode, then the
   * condition is fulfilled when
   *                             ! the joints are within [-MinMax, MinMax].
   *   num lp_filter;            ! Low pass filer bandwidth of the EGM
   * controller [Hz].
   *   num max_speed_deviation;  ! Maximum admitted joint speed change [deg/s]:
   *                             !   Note: Should not be set higher than the
   * lowest max speed [deg/s],
   *                             !         out of all the axis max speeds (found
   * in the robot's data sheet).
   *
   *   num cond_time;            ! Condition time [s].
   *   num ramp_in_time;         ! Duration for ramp in [s].
   *   num pos_corr_gain;        ! Position correction gain of the EGM
   * controller.
   * ENDRECORD
   *
   */
  void getParams();

  // part of the constructor that is the same for all constructors
  void init();

  /** \brief Initializes EGM connection to the robot
   *
   */
  bool initEGM();

  bool stop();

  /** \brief gets the current joint positions/velocities/accelerations from the
   * robot.
   *
   * \param joint_* the joint positions/velocities/accelerations. Elements 0-6
   * correspond to the
   *  left arm and elements 7-13 to the right arm. The joints are ordered as in
   * the URDF,
   * starting from the shoulder until the wrist.
   */
  void getCurrentJointStates(float (&joint_pos)[N_JOINTS_ARM],
                             float (&joint_vel)[N_JOINTS_ARM]);

  /** \brief sets the joint velocities to command to the robot.
   *
   * \param joint_vel_targets Array with the joint velocity commands, where
   * elements 0-6
   * correspond to the left arm and elements 7-13 correspond to the right arm.
   */
  void setJointTargets(const float (&joint_vel_targets)[N_JOINTS_ARM], const YumiHW::ControlStrategy mode);

protected:
  /** \brief Copies EGM protobuf joint states (pos, vel or acc) collected from
   * the EGM interface to an array.
   *
   * \param joint_states These correspond to YuMi's joints 1,2,4,5,6,7.
   *
   * \param external_joint_states This corresponds to YuMi's 3rd joint, the
   * redudancy joint.
   *
   * \param joint_array The array at which the joint states are copied, in the
   * same order
   * as the robot's URDF, starting from the shoulder until the wrist.
   */
  /** \brief Extracts joint pos, vel and acc from an EGM protobuf joint space
   * message
   * and copies them to separate float arrays.
   *
   * \param joint_space EGM joint space protobuf message from which to extract
   * the joint variables
   *
   * \param joint_pos
   *
   * \param joint_vel
   *
   * \param joint_acc
   */
  void copyEGMInputToArray(::wrapper::Input *const input, float *const joint_pos,
                           float *const joint_vel) const;

  /** \brief Copies protobuf joint states (pos, vel or acc) collected from
   * the EGM interface to an array.
   *
   * \param joint_states These correspond to YuMi's joints 1,2,4,5,6,7.
   *
   * \param external_joint_states This corresponds to YuMi's 3rd joint, the
   * redudancy joint.
   *
   * \param joint_array The array at which the joint states are copied, in the
   * same order
   * as the robot's URDF, starting from the shoulder until the wrist.
   */
  void copyVelocityArrayToEGMOutput(const float *const joint_array,
                            ::wrapper::Output *output) const;
  
  void copyPositionArrayToEGMOutput(const float *const  joint_array,
                            ::wrapper::Output *const output) const;

  bool initRWS();

  bool initOutput(std::shared_ptr<abb::egm::wrapper::Output> &output,
                  const std::shared_ptr<abb::egm::wrapper::Input> &input);

  bool initEGMArm(const 
    std::shared_ptr<EGMControllerInterface>&interface);

/** \brief Sends EGM parameters collected from the ROS parameter server to the
 * robot controller through RWS (TCP connection), such as
 * LP filter bandwidth, condition time, etc. For more details see getParams()
 */

  bool sendEGMParams();

  void getEGMParams(YumiRWSforEGMWrapper::EGMSettings& egm_settings);

  void configureEGM(const 
    std::shared_ptr<EGMControllerInterface>& egm_interface);

  bool startEGM();

  bool stopEGM();

  /* RWS */
  // RWS interface which uses TCP communication for starting the EGM joint mode
  // on YuMi
  std::shared_ptr<YumiRWSforEGMWrapper> rws_interface_;

  // RWS connection parameters
  double rws_delay_time_;
  
  const std::string task_;
  /* EGM */
  // EGM interface which uses UDP communication for realtime robot control @ 250
  // Hz
  unsigned short egm_port_;
  std::shared_ptr<EGMControllerInterface> egm_interface_;

  // feedback and status read from the egm interface
  std::shared_ptr<abb::egm::wrapper::Input> input_;
  std::shared_ptr<abb::egm::wrapper::Status> status_;
  std::shared_ptr<abb::egm::wrapper::Output> output_;

  // joint velocity commands sent to the egm interface

  // io service used for EGM
  boost::asio::io_service io_service_;
  boost::thread_group io_service_threads_;

  YumiRWSforEGMWrapper::EGMSettings egm_settings_;

  double max_joint_velocity_;

  bool has_params_;
};



#endif // YUMI_HW_EGM_H