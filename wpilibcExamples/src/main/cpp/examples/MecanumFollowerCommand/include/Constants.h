/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <units/units.h>

#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/MecanumDriveKinematics.h>
#include <frc/trajectory/TrapezoidProfile.h>

#pragma once

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or bool constants.  This should not be used for any other purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

namespace DriveConstants {
const int kFrontLeftMotorPort = 0;
const int kRearLeftMotorPort = 1;
const int kFrontRightMotorPort = 2;
const int kRearRightMotorPort = 3;

const int kFrontLeftEncoderPorts [] = {0, 1};
const int kRearLeftEncoderPorts [] = {2, 3};
const int kFrontRightEncoderPorts [] = {4, 5};
const int kRearRightEncoderPorts [] = {5, 6};
    
const bool kFrontLeftEncoderReversed = false;
const bool kRearLeftEncoderReversed = true;
const bool kFrontRightEncoderReversed = false;
const bool kRearRightEncoderReversed = true;

const auto kTrackWidth = .5_m; //Distance between centers of right and left wheels on robot
const auto kTrackLength = .7_m; //Distance between centers of front and back wheels on robot
const frc::MecanumDriveKinematics kDriveKinematics(
    frc::Translation2d(kTrackLength/2, kTrackWidth/2),
    frc::Translation2d(kTrackLength/2, -kTrackWidth/2), 
    frc::Translation2d(-kTrackLength/2, kTrackWidth/2), 
    frc::Translation2d(-kTrackLength/2, -kTrackWidth/2)); 

const int kEncoderCPR = 1024;
const double kWheelDiameterMeters = .15;
const double kEncoderDistancePerPulse =
    // Assumes the encoders are directly mounted on the wheel shafts
    (kWheelDiameterMeters * 3.142) / static_cast<double>(kEncoderCPR);

const bool kGyroReversed = false;

// These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
// These characterization values MUST be determined either experimentally or
// theoretically for *your* robot's drive. The RobotPy Characterization
// Toolsuite provides a convenient tool for obtaining these values for your
// robot.
const auto ks = 1_V;
const auto kv = .8 * 1_V * 1_s / 1_m;
const auto ka = .15 * 1_V * 1_s * 1_s / 1_m;

// Example value only - as above, this must be tuned for your drive!
const double kPFrontLeftVel = .5;
const double kPRearLeftVel = .5;
const double kPFrontRightVel = .5;
const double kPRearRightVel = .5;
}  // namespace DriveConstants

namespace AutoConstants {
using radians_per_second_squared_t =
      units::compound_unit<units::radians, units::inverse<units::squared<units::second>>>;

const auto kMaxSpeed = units::meters_per_second_t(3);
const auto kMaxAcceleration = units::meters_per_second_squared_t(3);
const auto kMaxAngularSpeed = units::radians_per_second_t(3);
const auto kMaxAngularAcceleration = units::unit_t<radians_per_second_squared_t>(3);

const double kPXController = .5;
const double kPYController = .5;
const double kPThetaController = .5;

// 

const frc::TrapezoidProfile::Constraints kThetaControllerConstraints{units::meters_per_second_t(kMaxAngularSpeed.to<double>()), units::meters_per_second_squared_t(kMaxAngularAcceleration.to<double>())};

}  // namespace AutoConstants

namespace OIConstants {
const int kDriverControllerPort = 1;
}  // namespace OIConstants
