/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/SwerveModule.h"

#include <frc/geometry/Rotation2d.h>
#include <wpi/math>

#include "Constants.h"

SwerveModule::SwerveModule(int driveMotorChannel, int turningMotorChannel,
                           std::array<int, 2> driveEncoderPorts,
                           std::array<int, 2> turningEncoderPorts,
                           bool driveEncoderReversed,
                           bool turningEncoderReversed)
    : m_driveMotor(driveMotorChannel),
      m_turningMotor(turningMotorChannel),
      m_driveEncoder(driveEncoderPorts[0], driveEncoderPorts[1]),
      m_turningEncoder(turningEncoderPorts[0], turningEncoderPorts[1]),
      m_reverseDriveEncoder(driveEncoderReversed),
      m_reverseTurningEncoder(turningEncoderReversed) {
  // Set the distance per pulse for the drive encoder. We can simply use the
  // distance traveled for one rotation of the wheel divided by the encoder
  // resolution.
  m_driveEncoder.SetDistancePerPulse(ModuleConstants::kEncoderDistancePerPulse);

  // Set the distance (in this case, angle) per pulse for the turning encoder.
  // This is the the angle through an entire rotation (2 * wpi::math::pi)
  // divided by the encoder resolution.
  m_turningEncoder.SetDistancePerPulse(
      ModuleConstants::kEncoderDistancePerPulse);

  // Limit the PID Controller's input range between -pi and pi and set the input
  // to be continuous.
  m_turningPIDController.EnableContinuousInput(-wpi::math::pi, wpi::math::pi);
}

frc::SwerveModuleState SwerveModule::GetState() {
  return {units::meters_per_second_t{m_driveEncoder.GetRate()},
          frc::Rotation2d(units::radian_t(m_turningEncoder.Get()))};
}

void SwerveModule::SetDesiredState(frc::SwerveModuleState& state) {
  // Calculate the drive output from the drive PID controller.
  const auto driveOutput = m_drivePIDController.Calculate(
      m_driveEncoder.GetRate(), state.speed.to<double>());

  // Calculate the turning motor output from the turning PID controller.
  auto turnOutput = m_turningPIDController.Calculate(
      m_turningEncoder.Get(),
      // We have to convert to the meters type here because that's what
      // ProfiledPIDController wants.
      units::meter_t(state.angle.Radians().to<double>()));

  // Set the motor outputs.
  m_driveMotor.Set(driveOutput);
  m_turningMotor.Set(turnOutput);
}

void SwerveModule::ResetEncoders() {
  m_driveEncoder.Reset();
  m_turningEncoder.Reset();
}
