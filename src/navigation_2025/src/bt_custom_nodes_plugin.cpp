// Copyright 2025 OTAGG Team
// Licensed under Apache 2.0

#include "behaviortree_cpp_v3/bt_factory.h"
#include "navigation_2025/bt_custom_nodes.hpp"

BT_REGISTER_NODES(factory) {
  // Original Ackermann recovery nodes
  factory.registerNodeType<navigation_2025::DetectOscillation>(
      "DetectOscillation");
  factory.registerNodeType<navigation_2025::PathDivergenceCheck>(
      "PathDivergenceCheck");
  factory.registerNodeType<navigation_2025::LowVelocityCheck>(
      "LowVelocityCheck");
  factory.registerNodeType<navigation_2025::IsSafeToBackup>("IsSafeToBackup");
  factory.registerNodeType<navigation_2025::HasClearanceForManeuver>(
      "HasClearanceForManeuver");

  // Traffic-aware condition nodes
  factory.registerNodeType<navigation_2025::IsRedLightDetected>(
      "IsRedLightDetected");
  factory.registerNodeType<navigation_2025::IsYellowLightDetected>(
      "IsYellowLightDetected");
  factory.registerNodeType<navigation_2025::IsGreenLightDetected>(
      "IsGreenLightDetected");
  factory.registerNodeType<navigation_2025::IsStopSignDetected>(
      "IsStopSignDetected");
  factory.registerNodeType<navigation_2025::IsTurnRestricted>(
      "IsTurnRestricted");
  factory.registerNodeType<navigation_2025::IsBusStopDetected>(
      "IsBusStopDetected");
  factory.registerNodeType<navigation_2025::IsSpeedLimitActive>(
      "IsSpeedLimitActive");
}
