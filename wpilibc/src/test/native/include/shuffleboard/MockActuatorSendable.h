// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string_view>

#include "frc/smartdashboard/Sendable.h"
#include "frc/smartdashboard/SendableBuilder.h"

/**
 * A mock sendable that marks itself as an actuator.
 */
class MockActuatorSendable : public frc::Sendable {
 public:
  explicit MockActuatorSendable(std::string_view name);

  void InitSendable(frc::SendableBuilder& builder) override;
};
