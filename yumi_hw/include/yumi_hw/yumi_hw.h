#ifndef YUMI_HW_H
#define YUMI_HW_H

// boost
#include <boost/scoped_ptr.hpp>

// ROS headers
#include <std_msgs/Duration.h>
#include <urdf/model.h>

// ROS control
#include <control_toolbox/filters.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <transmission_interface/transmission_info.h>
#include <transmission_interface/transmission_parser.h>

#ifndef N_JOINTS_ARM
#define N_JOINTS_ARM 7
#endif

#ifndef N_ARMS
#define N_ARMS 2
#endif

/**
  * Base class for yumi hw interface. Extended later for gazebo and for real
 * robot over rapid
  */

class YumiHW : public hardware_interface::RobotHW {
public:
  YumiHW();//const double &exponential_smoothing_alpha = 0.04

  virtual ~YumiHW() {}

  void create(std::string name, std::string urdf_string);

  /* Control strategies definition */
  /* JOINT_POSITION -> strategy 10 -> triggered with PoitionJointInterface */
  /* JOINT_VELOCITY -> strategy 15 -> triggered with VelJointInterface */
  /* JOINT_EFFORT -> strategy 20 -> TODO */
  enum ControlStrategy {
    JOINT_POSITION = 10,
    JOINT_VELOCITY = 15,
    JOINT_EFFORT = 20
  };

  /* Control strategy get/set */
  ControlStrategy getControlStrategy() { return current_strategy_; };
  void setControlStrategy(ControlStrategy strategy) {
    current_strategy_ = strategy;
  };

  // Before write, you can use this function to enforce limits for all values
  void enforceLimits(ros::Duration period);

  /* Set all members to default values */
  void reset();

  /* RobotHW primitives */
  /* This functions must be implemented depending on the outlet (Real, Gazebo,
   * etc.) */
  virtual bool init() = 0;
  virtual void read(ros::Time time, ros::Duration period) = 0;
  virtual void write(ros::Time time, ros::Duration period) = 0;
  virtual bool canSwitch(
      const std::list<hardware_interface::ControllerInfo> &start_list,
      const std::list<hardware_interface::ControllerInfo> &stop_list) const;
  virtual void
  doSwitch(const std::list<hardware_interface::ControllerInfo> &start_list,
           const std::list<hardware_interface::ControllerInfo> &stop_list);

  std::string robot_namespace_;

  /* Robot Model from URDF */
  int n_joints_;
  std::string urdf_string_;
  urdf::Model urdf_model_;
  std::vector<std::string> joint_names_;
  // TODO checkout where this is used
  // double exponential_smoothing_alpha_;

  /* Transmissions in this plugin's scope */
  // Only for grippers?
  std::vector<transmission_interface::TransmissionInfo> transmissions_;

  /* Hardware interfaces */
  hardware_interface::JointStateInterface state_interface_;
  // hardware_interface::EffortJointInterface effort_interface_; //TODO
  hardware_interface::PositionJointInterface position_interface_;
  hardware_interface::VelocityJointInterface velocity_interface_;

  ControlStrategy current_strategy_;

  /* Joint limits interfaces */
  // joint_limits_interface::EffortJointSaturationInterface ej_sat_interface_;
  // joint_limits_interface::EffortJointSoftLimitsInterface
  // ej_limits_interface_;
  joint_limits_interface::VelocityJointSaturationInterface vj_sat_interface_;
  joint_limits_interface::VelocityJointSoftLimitsInterface vj_limits_interface_;
  joint_limits_interface::PositionJointSaturationInterface pj_sat_interface_;
  joint_limits_interface::PositionJointSoftLimitsInterface pj_limits_interface_;

  // SystemConstants::RAPID::TASK_ROB_R; // todo
  
      /* Joint limits */
      std::vector<double>
          joint_lower_limits_, joint_upper_limits_;

  /* Joint state */
  std::vector<double> joint_position_, joint_position_prev_, joint_velocity_,
      joint_effort_;

  /* Joint commands */
  std::vector<double> joint_position_command_, joint_velocity_command_;

private:
  /* Get Transmissions from the URDF */
  bool parseTransmissionsFromURDF(const std::string &urdf_string);

  /* Register all interfaces */
  void registerInterfaces(
      const urdf::Model *const urdf_model,
      std::vector<transmission_interface::TransmissionInfo> transmissions);

  /* Helper function to register limit interfaces */
  void registerJointLimits(
      const std::string &joint_name,
      //  const hardware_interface::JointHandle& joint_handle_effort,
      const hardware_interface::JointHandle &joint_handle_position,
      const hardware_interface::JointHandle &joint_handle_velocity,
      const urdf::Model *const urdf_model, double *const lower_limit,
      double *const upper_limit);

}; // class

#endif // YUMI_HW_H