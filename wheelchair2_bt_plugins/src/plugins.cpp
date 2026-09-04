#include "wheelchair2_bt_plugins/check_battery_condition.hpp"
#include "wheelchair2_bt_plugins/dock_approval_action.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"

extern "C" __attribute__((visibility("default"))) void BT_RegisterNodesFromPlugin(BT::BehaviorTreeFactory& factory)
{
  // 1. Current active node
  factory.registerNodeType<wheelchair2_bt_plugins::CheckBatteryCondition>("CheckBatteryCondition");

  // 2. Future app integration node
  factory.registerNodeType<wheelchair2_bt_plugins::DockApprovalAction>("DockApprovalAction");
}
