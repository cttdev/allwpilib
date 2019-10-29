/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "frc/trajectory/constraint/SwerveDriveKinematicsConstraint.h"

using namespace frc;

template <int NumModules>
SwerveDriveKinematicsConstraint<NumModules>::SwerveDriveKinematicsConstraint(
    frc::SwerveDriveKinematics<NumModules> kinematics,
    units::meters_per_second_t maxSpeed)
    : m_kinematics(kinematics), m_maxSpeed(maxSpeed) {}

template <int NumModules>
units::meters_per_second_t
SwerveDriveKinematicsConstraint<NumModules>::MaxVelocity(
    const Pose2d& pose, curvature_t curvature,
    units::meters_per_second_t velocity) {
  auto xVelocity = velocity * sin(pose.Rotation().Radians().to<double>());
  auto yVelocity = velocity * cos(pose.Rotation().Radians().to<double>());
  auto wheelSpeeds = m_kinematics.ToSwerveModuleStates(
      {xVelocity, yVelocity, velocity * curvature});
  m_kinematics.NormalizeWheelSpeeds(wheelSpeeds);

  return m_kinematics.ToChassisSpeeds(wheelSpeeds).vx;
}

template <int NumModules>
TrajectoryConstraint::MinMax
SwerveDriveKinematicsConstraint<NumModules>::MinMaxAcceleration(
    const Pose2d& pose, curvature_t curvature,
    units::meters_per_second_t speed) {
  return {};
}
