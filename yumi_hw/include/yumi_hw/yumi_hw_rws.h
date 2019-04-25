#pragma once

#include <yumi_hw/yumi_hw.h>
#include <yumi_hw/arm_rws_interface.h>

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <ros/ros.h>
#include <simple_message/message_handler.h>
#include <simple_message/message_manager.h>
#include <simple_message/messages/joint_message.h>
#include <simple_message/smpl_msg_connection.h>
#include <simple_message/socket/tcp_client.h>
#include <simple_message/socket/tcp_socket.h>


/**
  * RobotHW interface class that connects to YuMi over RWS
  */
class YumiHWRWS : public YumiHW {
public:
  YumiHWRWS(const double &exponential_smoothing_alpha = 0.04);

  ~YumiHWRWS();

  void setup(std::string ip = "",
             int port = industrial::simple_socket::StandardSocketPorts::STATE);

  // Init, read, and write, with FRI hooks
  bool init();

  /// Copies the last received joint state out to the controller manager
  void read(ros::Time time, ros::Duration period);

  /// Caches the most recent joint commands into the robot interface
  void write(ros::Time time, ros::Duration period);

private:
  ///
  YumiRWSInterface rws_interface_;

  std::string ip_;
  int port_;

  bool is_initialized_;
  bool is_setup_;
  bool first_run_in_position_mode_;

  boost::mutex data_buffer_mutex_;

  /// command buffers
  float new_joint_position_[N_JOINTS_ARM];
  /// data buffers
  float read_joint_position_[N_JOINTS_ARM];
  float last_comm_[N_JOINTS_ARM];
};
