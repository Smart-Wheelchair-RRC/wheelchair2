/*
 * ElevatorSupervisor.cpp
 */

#include "ElevatorSupervisor.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <rcl_interfaces/msg/parameter.hpp>
#include <rcl_interfaces/msg/parameter_type.hpp>

namespace elevator_nav {

// =============================================================================
// Constructor
// =============================================================================
ElevatorSupervisor::ElevatorSupervisor(const rclcpp::NodeOptions &options)
    : Node("elevator_supervisor", options)
{
  loadConfig();

  // ── Subscribers ──────────────────────────────────────────────────────────
  floor_request_sub_ = this->create_subscription<std_msgs::msg::Int32>(
    "/elevator/request_floor", rclcpp::QoS(rclcpp::KeepLast(1)),
    std::bind(&ElevatorSupervisor::cbFloorRequest, this, std::placeholders::_1));

  door_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "/elevator/door_open", rclcpp::QoS(rclcpp::KeepLast(1)),
    std::bind(&ElevatorSupervisor::cbDoorOpen, this, std::placeholders::_1));

  // ── Nav2 action client ───────────────────────────────────────────────────
  nav2_client_ = rclcpp_action::create_client<NavigateToPose>(
    this, "navigate_to_pose");

  // ── Service clients ──────────────────────────────────────────────────────
  icp_pause_client_    = this->create_client<std_srvs::srv::SetBool>("/pause_localization");
  icp_reset_client_    = this->create_client<std_srvs::srv::Empty>("/reset_localization");
  icp_load_map_client_ = this->create_client<std_srvs::srv::Trigger>("/load_map");
  bbs_pause_client_    = this->create_client<std_srvs::srv::SetBool>("/bbs/pause");
  bbs_reload_client_   = this->create_client<std_srvs::srv::Trigger>("/bbs/reload_map");

  icp_set_param_client_ = this->create_client<rcl_interfaces::srv::SetParameters>(
    "/icp_localization/set_parameters");
  bbs_set_param_client_ = this->create_client<rcl_interfaces::srv::SetParameters>(
    "/gpu_ros2_test_rviz2/set_parameters");

  RCLCPP_INFO(this->get_logger(),
    "ElevatorSupervisor ready. Floor: %d. State: NORMAL_NAV.", current_floor_);
}

// =============================================================================
// loadConfig
// =============================================================================
void ElevatorSupervisor::loadConfig() {
  current_floor_ = this->declare_parameter("current_floor", 1);
  const int n    = this->declare_parameter("num_floors", 0);

  for (int i = 0; i < n; ++i) {
    const std::string p = "floors." + std::to_string(i) + ".";
    FloorConfig fc;
    fc.floor_id  = this->declare_parameter<int>        (p + "id",           0);
    fc.pcd_path  = this->declare_parameter<std::string>(p + "pcd_path",     "");
    fc.wait_pose.x   = this->declare_parameter<double> (p + "wait.x",       0.0);
    fc.wait_pose.y   = this->declare_parameter<double> (p + "wait.y",       0.0);
    fc.wait_pose.yaw = this->declare_parameter<double> (p + "wait.yaw",     0.0);
    fc.inside_pose.x   = this->declare_parameter<double>(p + "inside.x",   0.0);
    fc.inside_pose.y   = this->declare_parameter<double>(p + "inside.y",   0.0);
    fc.inside_pose.yaw = this->declare_parameter<double>(p + "inside.yaw", 0.0);
    floors_[fc.floor_id] = fc;
    RCLCPP_INFO(this->get_logger(),
      "Floor %d loaded: pcd=%s wait=(%.2f,%.2f) inside=(%.2f,%.2f)",
      fc.floor_id, fc.pcd_path.c_str(),
      fc.wait_pose.x, fc.wait_pose.y,
      fc.inside_pose.x, fc.inside_pose.y);
  }
}

// =============================================================================
// stateToStr / transitionTo
// =============================================================================
std::string ElevatorSupervisor::stateToStr(State s) const {
  switch (s) {
    case State::NORMAL_NAV:             return "NORMAL_NAV";
    case State::GOING_TO_WAIT:          return "GOING_TO_WAIT";
    case State::WAITING_FOR_DOOR:       return "WAITING_FOR_DOOR";
    case State::ENTERING:               return "ENTERING";
    case State::LOADING_MAPS:           return "LOADING_MAPS";
    case State::WAITING_FOR_DEST_FLOOR: return "WAITING_FOR_DEST_FLOOR";
    case State::EXITING:                return "EXITING";
  }
  return "UNKNOWN";
}

void ElevatorSupervisor::transitionTo(State s) {
  RCLCPP_INFO(this->get_logger(), "── State: %s → %s ──",
    stateToStr(state_).c_str(), stateToStr(s).c_str());
  state_ = s;
}

// =============================================================================
// cbFloorRequest — app publishes destination floor
// =============================================================================
void ElevatorSupervisor::cbFloorRequest(const std_msgs::msg::Int32::SharedPtr msg) {
  if (state_ != State::NORMAL_NAV) {
    RCLCPP_WARN(this->get_logger(),
      "Floor request ignored — already in transition (%s).",
      stateToStr(state_).c_str());
    return;
  }
  if (floors_.find(msg->data) == floors_.end()) {
    RCLCPP_ERROR(this->get_logger(), "Floor %d not in config.", msg->data);
    return;
  }
  if (msg->data == current_floor_) {
    RCLCPP_WARN(this->get_logger(), "Already on floor %d.", msg->data);
    return;
  }
  target_floor_ = msg->data;
  RCLCPP_INFO(this->get_logger(), "Request: floor %d → %d",
    current_floor_, target_floor_);
  phase_goToWait();
}

// =============================================================================
// cbDoorOpen — camera publishes door state
//
// The meaning of a door_open=true signal depends entirely on which state
// we are in:
//
//   WAITING_FOR_DOOR       → door open on current floor, robot can enter
//   WAITING_FOR_DEST_FLOOR → door open on destination floor, confirmed arrival
//                            ONLY NOW do we resume nodes and send exit goal
//
// In all other states the signal is ignored.
// =============================================================================
void ElevatorSupervisor::cbDoorOpen(const std_msgs::msg::Bool::SharedPtr msg) {
  if (!msg->data) return;  // only care about open events

  if (state_ == State::WAITING_FOR_DOOR) {
    RCLCPP_INFO(this->get_logger(), "Door open — entering elevator.");
    phase_enter();
  }
  else if (state_ == State::WAITING_FOR_DEST_FLOOR) {
    RCLCPP_INFO(this->get_logger(),
      "Camera confirmed destination floor %d. Resuming nodes and exiting.",
      target_floor_);
    phase_exit();
  }
  // ENTERING, LOADING_MAPS, EXITING — ignore duplicate signals
}

// =============================================================================
// phase_goToWait
// Send Nav2 goal to P1 (wait point outside elevator on current floor).
// =============================================================================
void ElevatorSupervisor::phase_goToWait() {
  transitionTo(State::GOING_TO_WAIT);
  const FloorConfig &from = floors_.at(current_floor_);

  RCLCPP_INFO(this->get_logger(),
    "Driving to elevator wait point floor %d: (%.2f, %.2f)",
    current_floor_, from.wait_pose.x, from.wait_pose.y);

  sendNav2Goal(from.wait_pose,
    "wait_floor" + std::to_string(current_floor_),
    [this](bool ok) {
      if (!ok) {
        RCLCPP_ERROR(this->get_logger(),
          "Failed to reach wait point. Aborting floor change.");
        transitionTo(State::NORMAL_NAV);
        return;
      }
      phase_waitForDoor();
    });
}

// =============================================================================
// phase_waitForDoor
// Robot is at P1. Just update state — cbDoorOpen will fire when ready.
// =============================================================================
void ElevatorSupervisor::phase_waitForDoor() {
  transitionTo(State::WAITING_FOR_DOOR);
  RCLCPP_INFO(this->get_logger(),
    "At wait point. Camera watching for elevator door to open...");
}

// =============================================================================
// phase_enter
// Door is open on current floor. Send robot straight into elevator to P2.
// Do NOT pause localization yet — the robot still needs accurate pose
// to drive into the elevator correctly.
// Pause happens AFTER the robot confirms it is inside (Nav2 goal reached).
// =============================================================================
void ElevatorSupervisor::phase_enter() {
  transitionTo(State::ENTERING);
  const FloorConfig &from = floors_.at(current_floor_);

  RCLCPP_INFO(this->get_logger(),
    "Driving into elevator: (%.2f, %.2f)",
    from.inside_pose.x, from.inside_pose.y);

  sendNav2Goal(from.inside_pose,
    "inside_floor" + std::to_string(current_floor_),
    [this](bool ok) {
      if (!ok) {
        RCLCPP_ERROR(this->get_logger(),
          "Failed to drive inside elevator. Aborting.");
        transitionTo(State::NORMAL_NAV);
        return;
      }
      RCLCPP_INFO(this->get_logger(), "Robot is inside elevator.");
      phase_loadMaps();
    });
}

// =============================================================================
// phase_loadMaps
// Robot is confirmed inside. Now:
//   1. Pause ICP and BBS (stop matching against elevator interior)
//   2. Set map_path on BBS → reload BBS map → BBS self-localizes on new floor
//      (BBS runs click_callback internally and gets the new /initialpose ready)
//   3. Set map_path on ICP → reload ICP map
//   4. Reset ICP internal state (clears stale scans, isFirstScanMatch_=true)
//   5. DO NOT resume yet — transition to WAITING_FOR_DEST_FLOOR
//      Nodes stay paused. Robot sits still inside elevator.
//      Resume only happens in phase_exit() after camera confirms the floor.
// =============================================================================
void ElevatorSupervisor::phase_loadMaps() {
  transitionTo(State::LOADING_MAPS);

  const FloorConfig &to = floors_.at(target_floor_);
  RCLCPP_INFO(this->get_logger(),
    "Loading maps for floor %d. Nodes will resume after camera confirmation.",
    target_floor_);

  // Step 1: Pause ICP
  callSetBool(icp_pause_client_, true, "ICP PAUSE",
  [this, &to]() {

    // Step 2: Pause BBS
    callSetBool(bbs_pause_client_, true, "BBS PAUSE",
    [this, &to]() {

      // Step 3: Reload BBS map
      // Temporarily unpause BBS just so its internal click_callback
      // can execute during reload. It will be re-paused right after.
      callSetBool(bbs_pause_client_, false, "BBS UNPAUSE for reload",
      [this, &to]() {

        setParamThenTrigger(
          bbs_set_param_client_, "map_path", to.pcd_path,
          bbs_reload_client_, "BBS RELOAD MAP",
          [this, &to](bool bbs_ok) {

            if (!bbs_ok) {
              RCLCPP_WARN(this->get_logger(),
                "BBS map reload failed — will still try ICP reload.");
            }

            // Re-pause BBS after reload
            callSetBool(bbs_pause_client_, true, "BBS PAUSE after reload",
            [this, &to]() {

              // Step 4: Reload ICP map
              setParamThenTrigger(
                icp_set_param_client_, "map_path", to.pcd_path,
                icp_load_map_client_, "ICP LOAD MAP",
                [this](bool /*icp_ok*/) {

                  // Step 5: Reset ICP state
                  // isFirstScanMatch_ = true so ICP will use the
                  // /initialpose BBS published as its first seed.
                  callEmpty(icp_reset_client_, "ICP RESET",
                  [this]() {

                    // Maps are loaded. Both nodes are paused.
                    // Robot sits still inside elevator.
                    // We wait here for camera to confirm the floor.
                    phase_waitForDestFloor();
                  });
                });
            });
          });
      });
    });
  });
}

// =============================================================================
// phase_waitForDestFloor
// Maps are loaded. Both ICP and BBS are paused. Robot is stationary.
// Just update state — cbDoorOpen will fire when camera confirms arrival.
// =============================================================================
void ElevatorSupervisor::phase_waitForDestFloor() {
  transitionTo(State::WAITING_FOR_DEST_FLOOR);
  RCLCPP_INFO(this->get_logger(),
    "Maps loaded. Robot stationary. "
    "Waiting for camera to confirm floor %d...", target_floor_);
}

// =============================================================================
// phase_exit
// Camera has confirmed the destination floor. NOW resume nodes, then exit.
//
// Order:
//   1. Resume ICP  — it wakes up with isFirstScanMatch_=true and uses the
//                    /initialpose BBS already published → snaps to new map
//   2. Resume BBS  — starts continuous localization on new floor map
//   3. Send Nav2 goal to P4 (outside elevator on destination floor)
// =============================================================================
void ElevatorSupervisor::phase_exit() {
  transitionTo(State::EXITING);

  // Step 1: Resume ICP
  callSetBool(icp_pause_client_, false, "ICP RESUME",
  [this]() {

    // Step 2: Resume BBS
    callSetBool(bbs_pause_client_, false, "BBS RESUME",
    [this]() {

      current_floor_ = target_floor_;
      const FloorConfig &to = floors_.at(current_floor_);

      RCLCPP_INFO(this->get_logger(),
        "Nodes resumed on floor %d. Driving out of elevator to (%.2f, %.2f).",
        current_floor_, to.wait_pose.x, to.wait_pose.y);

      // Step 3: Drive straight out to P4
      sendNav2Goal(to.wait_pose,
        "exit_floor" + std::to_string(current_floor_),
        [this](bool ok) {
          if (!ok) {
            RCLCPP_WARN(this->get_logger(),
              "Exit goal failed — robot may need manual correction.");
          }
          phase_done();
        });
    });
  });
}

// =============================================================================
// phase_done — back to normal navigation
// =============================================================================
void ElevatorSupervisor::phase_done() {
  transitionTo(State::NORMAL_NAV);
  target_floor_ = -1;
  RCLCPP_INFO(this->get_logger(),
    "Floor transition complete. Now navigating floor %d.", current_floor_);
}

// =============================================================================
// sendNav2Goal
// =============================================================================
void ElevatorSupervisor::sendNav2Goal(
    const Pose2D &pose,
    const std::string &label,
    std::function<void(bool)> on_done)
{
  if (!nav2_client_->wait_for_action_server(std::chrono::seconds(5))) {
    RCLCPP_ERROR(this->get_logger(),
      "Nav2 not available for goal: %s", label.c_str());
    if (on_done) on_done(false);
    return;
  }

  NavigateToPose::Goal goal;
  goal.pose.header.frame_id = "map";
  goal.pose.header.stamp    = this->now();
  goal.pose.pose.position.x = pose.x;
  goal.pose.pose.position.y = pose.y;
  goal.pose.pose.position.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, pose.yaw);
  goal.pose.pose.orientation = tf2::toMsg(q);

  RCLCPP_INFO(this->get_logger(),
    "Nav2 → [%s] (%.2f, %.2f, yaw=%.2f rad)",
    label.c_str(), pose.x, pose.y, pose.yaw);

  auto opts = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
  opts.result_callback =
    [this, label, on_done](const GoalHandle::WrappedResult &result) {
      bool ok = (result.code == rclcpp_action::ResultCode::SUCCEEDED);
      RCLCPP_INFO(this->get_logger(),
        "Nav2 [%s] %s.", label.c_str(), ok ? "SUCCEEDED" : "FAILED");
      if (on_done) on_done(ok);
    };

  nav2_client_->async_send_goal(goal, opts);
}

// =============================================================================
// setParamThenTrigger
// Sets a ROS param on a remote node, then calls a Trigger service.
// =============================================================================
void ElevatorSupervisor::setParamThenTrigger(
    rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr param_client,
    const std::string &param_name,
    const std::string &param_value,
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr svc,
    const std::string &label,
    std::function<void(bool)> on_done)
{
  if (!param_client || !param_client->service_is_ready()) {
    RCLCPP_WARN(this->get_logger(),
      "%s: set_parameters not ready, calling service directly.", label.c_str());
    callTrigger(svc, label, on_done);
    return;
  }

  auto req = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
  rcl_interfaces::msg::Parameter p;
  p.name = param_name;
  p.value.type         = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
  p.value.string_value = param_value;
  req->parameters.push_back(p);

  param_client->async_send_request(req,
    [this, svc, label, on_done](
      rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedFuture) {
      callTrigger(svc, label, on_done);
    });
}

// =============================================================================
// callSetBool
// =============================================================================
void ElevatorSupervisor::callSetBool(
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client,
    bool val, const std::string &label,
    std::function<void()> on_done)
{
  if (!client || !client->service_is_ready()) {
    RCLCPP_WARN(this->get_logger(), "%s: not ready, skipping.", label.c_str());
    if (on_done) on_done();
    return;
  }
  auto req  = std::make_shared<std_srvs::srv::SetBool::Request>();
  req->data = val;
  client->async_send_request(req,
    [this, label, on_done](
      rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture f) {
      try {
        auto r = f.get();
        RCLCPP_INFO(this->get_logger(), "%s → %s", label.c_str(), r->message.c_str());
      } catch (...) {}
      if (on_done) on_done();
    });
}

// =============================================================================
// callEmpty
// =============================================================================
void ElevatorSupervisor::callEmpty(
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr client,
    const std::string &label,
    std::function<void()> on_done)
{
  if (!client || !client->service_is_ready()) {
    RCLCPP_WARN(this->get_logger(), "%s: not ready, skipping.", label.c_str());
    if (on_done) on_done();
    return;
  }
  client->async_send_request(
    std::make_shared<std_srvs::srv::Empty::Request>(),
    [this, label, on_done](
      rclcpp::Client<std_srvs::srv::Empty>::SharedFuture) {
      RCLCPP_INFO(this->get_logger(), "%s done.", label.c_str());
      if (on_done) on_done();
    });
}

// =============================================================================
// callTrigger
// =============================================================================
void ElevatorSupervisor::callTrigger(
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client,
    const std::string &label,
    std::function<void(bool)> on_done)
{
  if (!client || !client->service_is_ready()) {
    RCLCPP_WARN(this->get_logger(), "%s: not ready, skipping.", label.c_str());
    if (on_done) on_done(false);
    return;
  }
  client->async_send_request(
    std::make_shared<std_srvs::srv::Trigger::Request>(),
    [this, label, on_done](
      rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture f) {
      bool ok = false;
      try {
        auto r = f.get();
        ok = r->success;
        RCLCPP_INFO(this->get_logger(),
          "%s → %s: %s", label.c_str(), ok ? "OK" : "FAIL", r->message.c_str());
      } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "%s exception: %s", label.c_str(), e.what());
      }
      if (on_done) on_done(ok);
    });
}

} // namespace elevator_nav

// =============================================================================
// main
// =============================================================================
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<elevator_nav::ElevatorSupervisor>(
    rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}