#pragma once

#include <string>
#include <memory>
#include "behaviortree_cpp_v3/action_node.h"
#include "std_msgs/msg/bool.hpp"
#include "rclcpp/rclcpp.hpp"

namespace wheelchair2_bt_plugins
{
class DockApprovalAction : public BT::ActionNodeBase
{
public:
  DockApprovalAction(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ActionNodeBase(name, config)
  {
    node_ = rclcpp::Node::make_shared("bt_dock_approval");
    sub_ = node_->create_subscription<std_msgs::msg::Bool>(
      "/wheelchair/dock_approval", 10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        approved_ = msg->data;
        decision_made_ = true;
      });
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("topic_name", "/wheelchair/dock_approval", "Approval topic from App/CLI")
    };
  }

  BT::NodeStatus tick() override
  {
    rclcpp::spin_some(node_);

    if (decision_made_) {
      if (approved_) {
        RCLCPP_INFO(node_->get_logger(), "User APPROVED return to dock.");
        reset();
        return BT::NodeStatus::SUCCESS;
      } else {
        RCLCPP_WARN(node_->get_logger(), "User REJECTED return to dock.");
        reset();
        return BT::NodeStatus::FAILURE;
      }
    }

    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 4000,
      "Battery low (<30%%). Waiting for user approval on /wheelchair/dock_approval...");
    return BT::NodeStatus::RUNNING;
  }

  void halt() override
  {
    reset();
  }

private:
  void reset()
  {
    decision_made_ = false;
    approved_ = false;
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_;
  bool decision_made_{false};
  bool approved_{false};
};
}  // namespace wheelchair2_bt_plugins
