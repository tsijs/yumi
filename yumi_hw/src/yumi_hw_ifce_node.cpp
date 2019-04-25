/* System dependencies */
#include <cmath>
#include <signal.h>
#include <stdexcept>
#include <sys/mman.h>
#include <time.h>

/* ROS headers */
#include <boost/interprocess/smart_ptr/unique_ptr.hpp>
#include <controller_manager/controller_manager.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>

/* The Yumi EGM and RWS interfaces */
#include <yumi_hw/yumi_hw_egm.h>
#include <yumi_hw/yumi_hw_rws.h>

bool g_quit = false;

void quitRequested(int sig) { g_quit = true; }

/* Get the URDF XML from the parameter server */
std::string getURDF(ros::NodeHandle &model_nh_, std::string param_name) {
  std::string urdf_string;
  std::string robot_description = "/robot_description";

  /* Search and wait for robot_description on param server */
  while (urdf_string.empty()) {
    std::string search_param_name;
    if (model_nh_.searchParam(param_name, search_param_name)) {
      ROS_INFO_ONCE_NAMED("yumi_hw_ifce_node",
                          "Yumi Interface node is waiting for model"
                          " URDF in parameter [%s] on the ROS param server.",
                          search_param_name.c_str());

      model_nh_.getParam(search_param_name, urdf_string);
    } else {
      ROS_INFO_ONCE_NAMED("yumi_hw_ifce_node",
                          "Yumi Interface Node is waiting for model"
                          " URDF in parameter [%s] on the ROS param server.",
                          robot_description.c_str());

      model_nh_.getParam(param_name, urdf_string);
    }

    usleep(100000); // 0.1 s
  }
  ROS_DEBUG_STREAM_NAMED("yumi_hw_ifce_node",
                         "Received URDF from param server, parsing...");

  return urdf_string;
}

int main(int argc, char **argv) {
  /* Init ROS node */
  ros::init(argc, argv, "yumi_hw_interface",
            ros::init_options::NoSigintHandler);

  /* ROS spinner */
  ros::AsyncSpinner spinner(4); // should be 4 eventually?s
  spinner.start();

  /* Custom operating system signal handlers */
  // signal(SIGTERM, quitRequested);
  signal(SIGINT, quitRequested);
  signal(SIGHUP, quitRequested);

  /* Create node handler */
  ros::NodeHandle yumi_nh("~");

  /* Get params or give default values */
  int port;
  std::string ip;
  std::string name;
  bool use_egm;
  yumi_nh.param("port", port, 80);
  yumi_nh.param(
      "ip", ip,
      std::string(
          "192.168.125.1")); // simulation. For live robot: 192.168.125.1
  yumi_nh.param("name", name, std::string("yumi"));
  yumi_nh.param("use_egm", use_egm, true);

  /* Get the general robot description, the YumiHW class will take care of
   * parsing what's useful to itself */
  std::string urdf_string = getURDF(yumi_nh, "/robot_description");

  std::shared_ptr<YumiHW> yumi_robot;
  // TODO make rws interface work standalone as well
  // if (!use_egm) {
  //   ROS_INFO("Use RWS");
  //   // yumi_robot = new YumiHWRWS();
  //   yumi_robot = std::make_shared<YumiHWRWS>();
  //   std::shared_ptr<YumiHWRWS> yumi_robot_rws = std::dynamic_pointer_cast<YumiHWRWS>(yumi_robot);
  //   yumi_robot_rws->setup(ip);
  //   ROS_INFO("Setting up connection to YuMi over RWS");
  // } else {
    ROS_INFO("Use EGM");
    yumi_robot = std::make_shared<YumiHWEGM>();
    std::shared_ptr<YumiHWEGM> yumi_robot_egm = std::dynamic_pointer_cast<YumiHWEGM>(yumi_robot);
    std::stringstream port_ss;
    port_ss << port;
    const int port_l = 6511;
    const int port_r = 6512; // TODO: make parameter and get from config file
    yumi_robot_egm->setup(ip, port_ss.str(), port_l, port_r);
    ROS_INFO("Setting up connection to YuMi over EGM");
  // }

  yumi_robot->create(name, urdf_string);

  if (!yumi_robot->init()) // initializes egm interface communication
  {
    ROS_FATAL_NAMED("yumi_hw", "Could not initialize robot hardware interface");
    return -1;
  }

  /* Timer variables */
  struct timespec ts = {0, 0};
  ros::Time last(ts.tv_sec, ts.tv_nsec), curr(ts.tv_sec, ts.tv_nsec),
      now(ts.tv_sec, ts.tv_nsec);
  ros::Duration period(1.0);

  /* The controller manager */
  controller_manager::ControllerManager manager(yumi_robot.get());

  /* Publisher of the control loop period */
  ros::Publisher control_period_pub;
  control_period_pub =
      yumi_nh.advertise<std_msgs::Float64>("/yumi/arms_control_period", 1000);

  /* Main control loop */
  while (!g_quit) {
    // estimate the control period
    now = ros::Time::now();
    if (!clock_gettime(CLOCK_MONOTONIC, &ts)) {
      curr.sec = ts.tv_sec;
      curr.nsec = ts.tv_nsec;
      period = curr - last;
      last = curr;
    } else {
      ROS_FATAL("Failed to poll realtime clock!");
      break;
    }

    /* Read the state from YuMi */
    yumi_robot->read(now, period);

    /* Update the controllers */
    manager.update(now, period);

    /* Write the command to YuMi */
    yumi_robot->write(now, period);

    // std::cout << "Control loop period is " << period.toSec() * 1000 << " ms"
    // << std::endl;
    control_period_pub.publish(period.toSec());
  }

  ROS_INFO("Stopping asynchronous spinner...");
  spinner.stop();

  return 0;
}
