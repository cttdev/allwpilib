// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "frc/GenericHID.h"

#include <hal/DriverStation.h>

#include "frc/DriverStation.h"
#include "frc/Errors.h"

using namespace frc;

GenericHID::GenericHID(int port) : m_ds(&DriverStation::GetInstance()) {
  if (port < 0 || port >= DriverStation::kJoystickPorts) {
    throw FRC_MakeError(warn::BadJoystickIndex, "port {} out of range", port);
  }
  m_port = port;
}

bool GenericHID::GetRawButton(int button) const {
  return m_ds->GetStickButton(m_port, button);
}

bool GenericHID::GetRawButtonPressed(int button) {
  return m_ds->GetStickButtonPressed(m_port, button);
}

bool GenericHID::GetRawButtonReleased(int button) {
  return m_ds->GetStickButtonReleased(m_port, button);
}

double GenericHID::GetRawAxis(int axis) const {
  return m_ds->GetStickAxis(m_port, axis);
}

int GenericHID::GetPOV(int pov) const {
  return m_ds->GetStickPOV(m_port, pov);
}

int GenericHID::GetAxisCount() const {
  return m_ds->GetStickAxisCount(m_port);
}

int GenericHID::GetPOVCount() const {
  return m_ds->GetStickPOVCount(m_port);
}

int GenericHID::GetButtonCount() const {
  return m_ds->GetStickButtonCount(m_port);
}

bool GenericHID::IsConnected() const {
  return m_ds->IsJoystickConnected(m_port);
}

GenericHID::HIDType GenericHID::GetType() const {
  return static_cast<HIDType>(m_ds->GetJoystickType(m_port));
}

std::string GenericHID::GetName() const {
  return m_ds->GetJoystickName(m_port);
}

int GenericHID::GetAxisType(int axis) const {
  return m_ds->GetJoystickAxisType(m_port, axis);
}

int GenericHID::GetPort() const {
  return m_port;
}

void GenericHID::SetOutput(int outputNumber, bool value) {
  m_outputs =
      (m_outputs & ~(1 << (outputNumber - 1))) | (value << (outputNumber - 1));

  HAL_SetJoystickOutputs(m_port, m_outputs, m_leftRumble, m_rightRumble);
}

void GenericHID::SetOutputs(int value) {
  m_outputs = value;
  HAL_SetJoystickOutputs(m_port, m_outputs, m_leftRumble, m_rightRumble);
}

void GenericHID::SetRumble(RumbleType type, double value) {
  if (value < 0) {
    value = 0;
  } else if (value > 1) {
    value = 1;
  }
  if (type == kLeftRumble) {
    m_leftRumble = value * 65535;
  } else {
    m_rightRumble = value * 65535;
  }
  HAL_SetJoystickOutputs(m_port, m_outputs, m_leftRumble, m_rightRumble);
}
