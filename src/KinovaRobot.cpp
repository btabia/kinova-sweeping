#include "KinovaRobot.h"



#define PORT 10000
#define PORT_REAL_TIME 10001

using namespace Kinova::Api;

KinovaRobot::KinovaRobot(std::string name_, std::string address,
                         std::string username, std::string password)
    : name(name_), step(0) {
  // Create API objects
  setState("unitialised");

  auto error_callback = [](KError err) {
    cout << "_________ callback error _________" << err.toString();
  };

  std::cout << "Creating transport objects" << std::endl;
  transport = new TransportClientTcp();
  router = new RouterClient(transport, error_callback);
  transport->connect(address, PORT);

  std::cout << "Creating transport real time objects" << std::endl;
  transport_real_time = new TransportClientUdp();
  router_real_time = new RouterClient(transport_real_time, error_callback);
  transport_real_time->connect(address, PORT_REAL_TIME);

  // Set session data connection information
  create_session_info = new Session::CreateSessionInfo();
  create_session_info->set_username(username);
  create_session_info->set_password(password);
  create_session_info->set_session_inactivity_timeout(60000);
  create_session_info->set_connection_inactivity_timeout(2000);

  // Session manager service wrapper
  std::cout << "Creating sessions for communication" << std::endl;
  session_manager = new SessionManager(router);
  session_manager->CreateSession(*create_session_info);
  session_manager_real_time = new SessionManager(router_real_time);
  session_manager_real_time->CreateSession(*create_session_info);
  std::cout << "Sessions created" << std::endl;

  // Create services
  base = new Base::BaseClient(router);
  base_cyclic = new BaseCyclic::BaseCyclicClient(router_real_time);
  actuator_config = new ActuatorConfig::ActuatorConfigClient(router);

  base_command = new BaseCyclic::Command();

  auto notification_callback =
      [](Kinova::Api::Base::ConfigurationChangeNotification data) {
        std::string jsonString = "";
        google::protobuf::util::MessageToJsonString(data, &jsonString);
        std::cout << "*************************************" << std::endl;
        std::cout << "**  Configuration Callback called  **" << std::endl;
        std::cout << jsonString << std::endl;
        std::cout << "*************************************" << std::endl;
      };

  // Subscribe to ConfigurationChange notifications
  auto notifHandle = base->OnNotificationConfigurationChangeTopic(
      notification_callback, Kinova::Api::Common::NotificationOptions());
}

KinovaRobot::~KinovaRobot() {
  // Close API session
  session_manager->CloseSession();
  session_manager_real_time->CloseSession();

  // Deactivate the router and cleanly disconnect from the transport object
  router->SetActivationStatus(false);
  transport->disconnect();
  router_real_time->SetActivationStatus(false);
  transport_real_time->disconnect();

  // Destroy the API
  delete base;
  delete base_cyclic;
  delete actuator_config;
  delete session_manager;
  delete session_manager_real_time;
  delete router;
  delete router_real_time;
  delete transport;
  delete transport_real_time;
  //delete base_feedback;
  delete base_command;
}

// Create an event listener that will set the promise action
// event to the exit value Will set promise to either END or ABORT
// Use finish_promise.get_future.get() to wait and get the value
std::function<void(Base::ActionNotification)> create_event_listener_by_promise(
    std::promise<Base::ActionEvent> &finish_promise) {
  return [&finish_promise](Base::ActionNotification notification) {
    const auto action_event = notification.action_event();
    switch (action_event) {
    case Base::ActionEvent::ACTION_END:
    case Base::ActionEvent::ACTION_ABORT:
      finish_promise.set_value(action_event);
      break;
    default:
      break;
    }
  };
}

constexpr auto TIMEOUT_PROMISE_DURATION = std::chrono::seconds{20};


bool KinovaRobot::home() {
  setState("Homing");
  // Make sure the arm is in Single Level Servoing before executing an Action
  auto servoingMode = Base::ServoingModeInformation();
  servoingMode.set_servoing_mode(Base::ServoingMode::SINGLE_LEVEL_SERVOING);
  base->SetServoingMode(servoingMode);
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  // Move arm to ready position
  std::cout << "Moving the arm to a safe position" << std::endl;
  auto action_type = Base::RequestedActionType();
  action_type.set_action_type(Base::REACH_JOINT_ANGLES);
  auto action_list = base->ReadAllActions(action_type);
  auto action_handle = Base::ActionHandle();
  action_handle.set_identifier(0);
  for (auto action : action_list.action_list()) {
    if (action.name() == "CMS Home") {
      action_handle = action.handle();
    }
  }

  if (action_handle.identifier() == 0) {
    std::cout << "Can't reach safe position, exiting" << std::endl;
    return false;
  } else {
    // Connect to notification action topic
    std::promise<Base::ActionEvent> finish_promise;
    auto finish_future = finish_promise.get_future();
    auto promise_notification_handle = base->OnNotificationActionTopic(
        create_event_listener_by_promise(finish_promise),
        Common::NotificationOptions());

    // Execute action
    base->ExecuteActionFromReference(action_handle);

    // Wait for future value from promise
    const auto status = finish_future.wait_for(TIMEOUT_PROMISE_DURATION);
    base->Unsubscribe(promise_notification_handle);

    if (status != std::future_status::ready) {
      std::cout << "Timeout on action notification wait" << std::endl;
      return false;
    }
    const auto promise_event = finish_future.get();

    std::cout << "Move to Home completed" << std::endl;
    std::cout << "Promise value : " << Base::ActionEvent_Name(promise_event)
              << std::endl;
    setState("Homed");
    return true;
  }
}

void KinovaRobot::getAllDevices() {
  /* API initialisation */
  auto device_manager = new DeviceManager::DeviceManagerClient(router);

  // Get all device routing information (from DeviceManagerClient service)
  auto allDevicesInfo = device_manager->ReadAllDevices();

  std::string ou;
  ou = allDevicesInfo.DebugString();

  std::cout << ou << std::endl;
}


void KinovaRobot::get(const Kinova::Api::BaseCyclic::Feedback *base_feedback) {
  Eigen7f actual_pose, actual_velocity, actual_torque;
  for (unsigned int i = 0; i < KinovaRobot::actuator_count; ++i) {
    auto actuator = base_feedback->actuators(i);
    actual_pose[i] = fmod(actuator.position(), 360.0f);
    actual_velocity[i] = actuator.velocity();
    actual_torque[i] = actuator.torque();
  }

   getGripper(base_feedback);

  set_pose(actual_pose);
  set_velocity(actual_velocity);
  set_torque(actual_torque);

  if(first)
  {
    setFirstPosition(actual_pose);
    setTorqueOffset(actual_torque);
    std::cout << "First position: " << std::endl << actual_pose << std::endl;
    first = false;
  }
  
}

void KinovaRobot::set(Eigen7f command, GripperAction gripperAction ) {
  auto frame_id = base_command->frame_id() + 1;
  base_command->set_frame_id(frame_id);
  if (frame_id > 65535)
    base_command->set_frame_id(0);
  auto p = this->get_pose();
  Eigen7f to = this->getTorqueOffset();
  for (unsigned int i = 0; i < KinovaRobot::actuator_count; i++) {
    base_command->mutable_actuators(i)->set_command_id(frame_id);
    base_command->mutable_actuators(i)->set_position(fmod(p[i], 360.0f));
    base_command->mutable_actuators(i)->set_torque_joint(command[i] - to(i));
  }
  doGripperAction(gripperAction);
  auto lambda_fct_callback =
      [this](const Kinova::Api::Error &err,
             const Kinova::Api::BaseCyclic::Feedback data) {
        // Set current pose, and velocity
        this->get(&data);
      };

  base_cyclic->Refresh_callback(*base_command, lambda_fct_callback, 0);

}

void KinovaRobot::torqueMode() 
{
  setState("Torque Mode");
  auto servoing_mode = Base::ServoingModeInformation();
  // Set the base in low-level servoing mode
  servoing_mode.set_servoing_mode(Base::ServoingMode::LOW_LEVEL_SERVOING);
  base->SetServoingMode(servoing_mode);
  using namespace std::chrono;
  auto start = high_resolution_clock::now();
  auto base_feedback = base_cyclic->RefreshFeedback();
  auto stop = high_resolution_clock::now();
  auto duration = duration_cast<microseconds>(stop - start);
  RCLCPP_INFO_STREAM(rclcpp::get_logger("Runner"),
                     "Time to get feedback for " << this->name << ":"
                                                 << duration.count() << "us");
  this->get(&base_feedback);  

  auto p = get_pose();

  delete  base_command;
  base_command = new BaseCyclic::Command();



  // Initialize each actuator to their current position
  for (unsigned int i = 0; i < actuator_count; i++) {
    // Save the current actuator position, to avoid a following error
    p[i] = fmod(p[i], 360.0f);
    base_command->add_actuators()->set_position(p[i]);
  }
  auto lambda_fct_callback =
      [this](const Kinova::Api::Error &err,
             const Kinova::Api::BaseCyclic::Feedback data) {
        // set cached pose and velocity
        this->get(&data);
      };
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("Runner"),
                      "First position sending... :" << p);
  // Send a first frame
  base_cyclic->Refresh_callback(*base_command, lambda_fct_callback, 0);

  auto control_mode_message = ActuatorConfig::ControlModeInformation();
  control_mode_message.set_control_mode(ActuatorConfig::ControlMode::TORQUE);
  for (unsigned int i = 1; i <= KinovaRobot::actuator_count; ++i)
    actuator_config->SetControlMode(control_mode_message, i);
}
constexpr auto TIMEOUT_DURATION = std::chrono::seconds{2};

void KinovaRobot::positionMode() 
{
  setState("Position Mode");
  auto control_mode_message = ActuatorConfig::ControlModeInformation();
  control_mode_message.set_control_mode(ActuatorConfig::ControlMode::POSITION);
  for (unsigned int i = 1; i <= KinovaRobot::actuator_count; ++i)
    actuator_config->SetControlMode(control_mode_message, i);

  auto servoing_mode = Base::ServoingModeInformation();
  // Set the servoing mode back to Single Level
  servoing_mode.set_servoing_mode(Base::ServoingMode::SINGLE_LEVEL_SERVOING);
  base->SetServoingMode(servoing_mode);

}

bool KinovaRobot::clearFault() 
{
  // Clearing faults
  try {
    base->ClearFaults();
    return true;
  } catch (...) {
    std::cout << "Unable to clear robot faults" << std::endl;
    return false;
  }
}

void KinovaRobot::setupGripper()
{
  if(!has_gripper) return;
    RCLCPP_INFO(rclcpp::get_logger("Runner"), "Setting up gripper.");
    base_command->mutable_interconnect()->mutable_command_id()->set_identifier(0);

    gripper_motor_command = base_command->mutable_interconnect()->mutable_gripper_command()->add_motor_cmd();
    gripper_motor_command->set_position( get_gripper() );
    gripper_motor_command->set_velocity(0.0);
    gripper_motor_command->set_force(100.0);
}

  void KinovaRobot::openGripper()
  {
    if(!has_gripper) return;

        // RCLCPP_INFO(rclcpp::get_logger("Runner"), "open up gripper.");

    float v = 10;
    gripper_motor_command->set_position( get_gripper() - v);
    gripper_motor_command->set_velocity(v);
    gripper_motor_command->set_force(100.0);

  }
  void KinovaRobot::closeGripper()
  {
    if(!has_gripper) return;
    float v = 10;

    // RCLCPP_INFO(rclcpp::get_logger("Runner"), "close up gripper.");


    gripper_motor_command->set_position( get_gripper() + v );
    gripper_motor_command->set_velocity(v);
    gripper_motor_command->set_force(100.0);

  }
  void KinovaRobot::stopGripper()
  {
    if(!has_gripper) return;
    gripper_motor_command->set_position( get_gripper() );
    gripper_motor_command->set_velocity(0.0);
    gripper_motor_command->set_force(100.0);

  }

  void KinovaRobot::doGripperAction(GripperAction gripperAction)
  {
    if(!has_gripper) return;

    switch (gripperAction)
    {
    case GripperAction::Stop:
      stopGripper();
      break;
    case GripperAction::Open:
      openGripper();
      break;
    case GripperAction::Close:
      closeGripper();
      break;
    default:
      stopGripper();
      break;
    }
  }

  void KinovaRobot::getGripper(const Kinova::Api::BaseCyclic::Feedback *base_feedback)
  {
      if(!has_gripper) return;
        
      set_gripper(base_feedback->interconnect().gripper_feedback().motor()[0].position()  );

  }