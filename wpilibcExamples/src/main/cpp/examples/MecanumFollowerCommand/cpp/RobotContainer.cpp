/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "RobotContainer.h"

#include <frc/controller/PIDController.h>
#include <frc/controller/RamseteController.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/geometry/Translation2d.h>
#include <frc/trajectory/constraint/DifferentialDriveKinematicsConstraint.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/MecanumFollowerCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/button/JoystickButton.h>

#include "Constants.h"

RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here

  // Configure the button bindings
  ConfigureButtonBindings();

  // Set up default drive command
  m_drive.SetDefaultCommand(frc2::RunCommand(
      [this] {
        m_drive.Drive(
            m_driverController.GetY(frc::GenericHID::kLeftHand),
            m_driverController.GetX(frc::GenericHID::kRightHand),
            m_driverController.GetX(frc::GenericHID::kLeftHand),
            false);
      },
      {&m_drive}));
}

void RobotContainer::ConfigureButtonBindings() {
  // Configure your button bindings here

  // While holding the shoulder button, drive at half speed
  frc2::JoystickButton(&m_driverController, 6)
      .WhenPressed(&m_driveHalfSpeed)
      .WhenReleased(&m_driveFullSpeed);
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  // An example trajectory to follow.  All units in meters.
  auto exampleTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
      // Start at the origin facing the +X direction
      frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)),
      // Pass through these two interior waypoints, making an 's' curve path
      {frc::Translation2d(1_m, 1_m), frc::Translation2d(2_m, -1_m)},
      // End 3 meters straight ahead of where we started, facing forward
      frc::Pose2d(3_m, 0_m, frc::Rotation2d(0_deg)),
      // Pass the drive kinematics to ensure constraints are obeyed
      DriveConstants::kDriveKinematics,
      // Start stationary
      0_m / 1_s,
      // End stationary
      0_m / 1_s,
      // Apply max speed constraint
      AutoConstants::kMaxSpeed,
      // Apply max acceleration constraint
      AutoConstants::kMaxAcceleration,
      // Not reversed
      false);

  frc2::MecanumFollowerCommand mecanumFollowerCommand(
      exampleTrajectory,
      [this]() { return m_drive.GetPose(); },

      DriveConstants::ks,
      DriveConstants::kv,
      DriveConstants::ka,
      DriveConstants::kDriveKinematics,

      frc2::PIDController(AutoConstants::kPXController, 0, 0),
      frc2::PIDController(AutoConstants::kPYController, 0, 0),
      frc::ProfiledPIDController(AutoConstants::kPThetaController, 0, 0, AutoConstants::kThetaControllerConstraints),

      AutoConstants::kMaxSpeed,

      [this]() {
        return frc::MecanumDriveWheelSpeeds{units::meters_per_second_t(m_drive.GetFrontLeftEncoder().GetRate()),
          units::meters_per_second_t(m_drive.GetFrontRightEncoder().GetRate()),
          units::meters_per_second_t(m_drive.GetRearLeftEncoder().GetRate()),
          units::meters_per_second_t(m_drive.GetRearRightEncoder().GetRate())};
      },

      frc2::PIDController(DriveConstants::kPFrontLeftVel, 0, 0),
      frc2::PIDController(DriveConstants::kPRearLeftVel, 0, 0),
      frc2::PIDController(DriveConstants::kPFrontRightVel, 0, 0),
      frc2::PIDController(DriveConstants::kPRearRightVel, 0, 0),

    
      [this](auto frontLeft, auto rearLeft, auto frontRight, auto rearRight) {
        m_drive.SetSpeedControllers(double(frontLeft) / 12. , double(rearLeft) / 12. , double(frontRight) / 12. , double(rearRight) / 12.);
        },

      {&m_drive});

  // no auto
  return new frc2::SequentialCommandGroup(
      std::move(mecanumFollowerCommand),
      frc2::InstantCommand([this]() { m_drive.Drive(0, 0, 0, false); }, {}));
}
