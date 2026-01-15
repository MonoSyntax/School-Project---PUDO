// Copyright 2025 OTAGG Team
// Licensed under Apache 2.0

#include "behaviortree_cpp_v3/bt_factory.h"
#include "navigation_2025/bt_custom_nodes.hpp"

BT_REGISTER_NODES(factory) {
  factory.registerNodeType<navigation_2025::DetectOscillation>(
      "DetectOscillation");
  factory.registerNodeType<navigation_2025::PathDivergenceCheck>(
      "PathDivergenceCheck");
  factory.registerNodeType<navigation_2025::LowVelocityCheck>(
      "LowVelocityCheck");
  factory.registerNodeType<navigation_2025::IsSafeToBackup>("IsSafeToBackup");
  factory.registerNodeType<navigation_2025::HasClearanceForManeuver>(
      "HasClearanceForManeuver");
}
