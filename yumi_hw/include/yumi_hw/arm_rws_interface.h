/**
  * Overrides message handler: keeps joint states thread-safe.
  */
class YumiJointStateHandler
    : public industrial::message_handler::MessageHandler {
public:
  using industrial::message_handler::MessageHandler::init;

  bool init(industrial::smpl_msg_connection::SmplMsgConnection *connection);

  bool getJointStates(float (&jnts)[N_JOINTS_ARM]);

  bool setJointCommands(float (&jnts)[N_JOINTS_ARM], int mode);

protected:
  bool internalCB(industrial::simple_message::SimpleMessage &in);

private:
  int mode_;

  bool first_iteration_;

  float joint_positions_[N_JOINTS_ARM];
  float joint_command_[N_JOINTS_ARM];

  bool joint_state_received_;
  bool joint_commands_set_;

  boost::mutex data_buffer_mutex_;
  boost::condition_variable c_joint_state_received_;
  boost::condition_variable c_joint_commands_set_;
};

/**
  * Keep a connection to the robot and send and receive joint states
  */
class YumiRWSInterface {
public:
  YumiRWSInterface();

  ~YumiRWSInterface();

  bool init(std::string ip = "",
            int port = industrial::simple_socket::StandardSocketPorts::STATE);

  void stopThreads();

  void startThreads();

  void getCurrentJointStates(float (&joints)[N_JOINTS_ARM]);

  void setJointTargets(float (&joints)[N_JOINTS_ARM], int mode);

private:
  virtual void rwsCommThreadCallback();

  YumiJointStateHandler js_handler_;

  bool stop_comm_;
  boost::thread rws_comm_thread_;

  // Connection over ros industrial sockets
  industrial::tcp_client::TcpClient default_tcp_connection_; //?
  // industrial::tcp_client::RobotStatusRelayHandler
  // default_robot_status_handler_; //?
  industrial::smpl_msg_connection::SmplMsgConnection *connection_;
  industrial::message_manager::MessageManager manager_;
};
