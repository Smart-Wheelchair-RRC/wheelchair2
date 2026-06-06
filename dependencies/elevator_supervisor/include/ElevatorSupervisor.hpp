/*
 * ElevatorSupervisor.hpp
 *
 * Revised state machine — maps load as soon as robot is inside,
 * robot exits only after camera confirms the destination floor.
 *
 *  NORMAL_NAV
 *    │  app publishes /elevator/request_floor
 *    ▼
 *  GOING_TO_WAIT          Nav2 drives robot to P1 (outside lift, floor N)
 *    │  Nav2 goal reached
 *    ▼
 *  WAITING_FOR_DOOR       camera watches elevator door
 *    │  /elevator/door_open = true
 *    ▼
 *  ENTERING               Nav2 drives robot straight into elevator (P2)
 *    │  Nav2 goal reached
 *    ▼
 *  LOADING_MAPS           pause ICP+BBS → reload both maps → resume both
 *    │  maps loaded, BBS localized on new floor
 *    ▼
 *  WAITING_FOR_DEST_FLOOR robot sits still, camera watches for dest floor
 *    │  /elevator/door_open = true  (confirmed on dest floor)
 *    ▼
 *  EXITING                Nav2 drives robot straight out (P4 on dest floor)
 *    │  Nav2 goal reached
 *    ▼
 *  NORMAL_NAV
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <rcl_interfaces/srv/set_parameters.hpp>

#include <string>
#include <unordered_map>
#include <functional>

namespace elevator_nav {

struct Pose2D {
  double x   = 0.0;
  double y   = 0.0;
  double yaw = 0.0;  // radians
};

struct FloorConfig {
  int         floor_id    = 0;
  std::string pcd_path;       // absolute path to this floor's .pcd
  Pose2D      wait_pose;      // P1/P4: outside lift — robot waits here, camera watches
  Pose2D      inside_pose;    // P2: robot drives straight in to here
};

class ElevatorSupervisor : public rclcpp::Node {
public:
  explicit ElevatorSupervisor(const rclcpp::NodeOptions &options);

private:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandle     = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  // ── States ─────────────────────────────────────────────────────────────────
  enum class State {
    NORMAL_NAV,
    GOING_TO_WAIT,
    WAITING_FOR_DOOR,
    ENTERING,
    LOADING_MAPS,
    WAITING_FOR_DEST_FLOOR,
    EXITING
  };
  State       state_{State::NORMAL_NAV};
  std::string stateToStr(State s) const;
  void        transitionTo(State s);

  // ── Config ─────────────────────────────────────────────────────────────────
  void loadConfig();
  std::unordered_map<int, FloorConfig> floors_;
  int current_floor_ = 1;
  int target_floor_  = -1;

  // ── Subscribers ────────────────────────────────────────────────────────────
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr floor_request_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr  door_sub_;
  void cbFloorRequest(const std_msgs::msg::Int32::SharedPtr msg);
  void cbDoorOpen(const std_msgs::msg::Bool::SharedPtr msg);

  // ── Nav2 action client ─────────────────────────────────────────────────────
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav2_client_;
  void sendNav2Goal(const Pose2D &pose, const std::string &label,
                    std::function<void(bool)> on_done);

  // ── Service clients ────────────────────────────────────────────────────────
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr icp_pause_client_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr   icp_reset_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr icp_load_map_client_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr bbs_pause_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr bbs_reload_client_;
  rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr icp_set_param_client_;
  rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr bbs_set_param_client_;

  void callSetBool(rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr c,
                   bool val, const std::string &label,
                   std::function<void()> on_done = nullptr);
  void callEmpty  (rclcpp::Client<std_srvs::srv::Empty>::SharedPtr c,
                   const std::string &label,
                   std::function<void()> on_done = nullptr);
  void callTrigger(rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr c,
                   const std::string &label,
                   std::function<void(bool)> on_done = nullptr);
  void setParamThenTrigger(
      rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr param_client,
      const std::string &param_name,
      const std::string &param_value,
      rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr svc,
      const std::string &label,
      std::function<void(bool)> on_done);

  // ── State transition actions ───────────────────────────────────────────────
  void phase_goToWait();         // NORMAL_NAV      → GOING_TO_WAIT
  void phase_waitForDoor();      // GOING_TO_WAIT   → WAITING_FOR_DOOR
  void phase_enter();            // WAITING_FOR_DOOR→ ENTERING
  void phase_loadMaps();         // ENTERING        → LOADING_MAPS
  void phase_waitForDestFloor(); // LOADING_MAPS    → WAITING_FOR_DEST_FLOOR
  void phase_exit();             // WAITING_FOR_DEST→ EXITING
  void phase_done();             // EXITING         → NORMAL_NAV
};

} // namespace elevator_nav