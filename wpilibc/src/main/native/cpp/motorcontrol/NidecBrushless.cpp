// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "frc/motorcontrol/NidecBrushless.h"

#include <fmt/format.h>
#include <hal/FRCUsageReporting.h>

#include "frc/smartdashboard/SendableBuilder.h"
#include "frc/smartdashboard/SendableRegistry.h"

using namespace frc;

NidecBrushless::NidecBrushless(int pwmChannel, int dioChannel)
    : m_dio(dioChannel), m_pwm(pwmChannel) {
  auto& registry = SendableRegistry::GetInstance();
  registry.AddChild(this, &m_dio);
  registry.AddChild(this, &m_pwm);
  SetExpiration(0_s);
  SetSafetyEnabled(false);

  // the dio controls the output (in PWM mode)
  m_dio.SetPWMRate(15625);
  m_dio.EnablePWM(0.5);

  HAL_Report(HALUsageReporting::kResourceType_NidecBrushless, pwmChannel + 1);
  registry.AddLW(this, "Nidec Brushless", pwmChannel);
}

void NidecBrushless::Set(double speed) {
  if (!m_disabled) {
    m_speed = speed;
    m_dio.UpdateDutyCycle(0.5 + 0.5 * (m_isInverted ? -speed : speed));
    m_pwm.SetRaw(0xffff);
  }
  Feed();
}

double NidecBrushless::Get() const {
  return m_speed;
}

void NidecBrushless::SetInverted(bool isInverted) {
  m_isInverted = isInverted;
}

bool NidecBrushless::GetInverted() const {
  return m_isInverted;
}

void NidecBrushless::Disable() {
  m_disabled = true;
  m_dio.UpdateDutyCycle(0.5);
  m_pwm.SetDisabled();
}

void NidecBrushless::Enable() {
  m_disabled = false;
}

void NidecBrushless::StopMotor() {
  m_dio.UpdateDutyCycle(0.5);
  m_pwm.SetDisabled();
}

std::string NidecBrushless::GetDescription() const {
  return fmt::format("Nidec {}", GetChannel());
}

int NidecBrushless::GetChannel() const {
  return m_pwm.GetChannel();
}

void NidecBrushless::InitSendable(SendableBuilder& builder) {
  builder.SetSmartDashboardType("Nidec Brushless");
  builder.SetActuator(true);
  builder.SetSafeState([=]() { StopMotor(); });
  builder.AddDoubleProperty(
      "Value", [=]() { return Get(); }, [=](double value) { Set(value); });
}
