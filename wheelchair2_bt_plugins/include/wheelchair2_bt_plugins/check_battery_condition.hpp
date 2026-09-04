#pragma once

#include <string>
#include <memory>
#include "behaviortree_cpp_v3/condition_node.h"
#include "sensor_msgs/msg/battery_state.hpp"
#include "rclcpp/rclcpp.hpp"

namespace wheelchair2_bt_plugins
{
class CheckBatteryCondition : public BT::ConditionNode
{
public:
  CheckBatteryCondition(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ConditionNode(name, config)
  {
    node_ = rclcpp::Node::make_shared("bt_battery_checker");
    sub_ = node_->create_subscription<sensor_msgs::msg::BatteryState>(
      "/wheelchair/battery_state", 10,
      [this](const sensor_msgs::msg::BatteryState::SharedPtr msg) {
        current_battery_ = msg->percentage;
        received_battery_ = true;
      });
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("min_battery", 0.30, "Minimum required battery percentage (0.0 - 1.0)")
    };
  }

  BT::NodeStatus tick() override
  {
    rclcpp::spin_some(node_);

    double min_battery = 0.30;
    getInput("min_battery", min_battery);

    if (!received_battery_) {
      return BT::NodeStatus::SUCCESS;
    }

    if (current_battery_ < min_battery) {
      RCLCPP_ERROR_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 2000,
        "====================================================\n"
        "[ERROR] Battery is low (%.1f%% < %.1f%%)! Goal aborted.\n"
        "====================================================",
        current_battery_ * 100.0, min_battery * 100.0);
      return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::SUCCESS;
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr sub_;
  double current_battery_{1.0};
  bool received_battery_{false};
};
}  // namespace wheelchair2_bt_plugins
