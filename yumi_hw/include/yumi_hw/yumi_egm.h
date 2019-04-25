#pragma once
#include <yumi_hw/yumi_hw.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <yumi_hw/arm_hw_egm.h>

class YumiHWEGM : public YumiHW {
public:
  YumiHWEGM(const double &exponential_smoothing_alpha = 0.04);

  ~YumiHWEGM();

  void setup(const std::string &ip, const std::string &port_rws,
             const int &port_egm_l, const int &port_egm_r);

  void setup(const std::string &ip, const std::string &port_rws,
             const int &port_egm_l);

  bool init();

  void read(ros::Time time, ros::Duration period);

  void write(ros::Time time, ros::Duration period);

private:
  bool is_initialized_;

  YumiEGMInterface yumi_egm_interface_r_;
  YumiEGMInterface yumi_egm_interface_l_;

  boost::mutex data_buffer_mutex_;

  std::string ip_;
  unsigned short port_rws_;
  unsigned short port_egm_r_;
  unsigned short port_egm_l_;

  // command buffers
  // TODO: see if other containers that are dynamically allocated could be more appropriate here?
  float joint_vel_targets_r_[N_JOINTS_ARM];
  float joint_vel_targets_l_[N_JOINTS_ARM];// new

  // data buffers
  float joint_pos_[N_JOINTS_ARM];
  float joint_vel_[N_JOINTS_ARM];
};
