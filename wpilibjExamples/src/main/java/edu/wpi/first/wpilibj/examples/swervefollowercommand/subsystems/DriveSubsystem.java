/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.examples.swervefollowercommand.subsystems;

import static edu.wpi.first.wpilibj.examples.swervefollowercommand.Constants.DriveConstants.*;
import static edu.wpi.first.wpilibj.examples.swervefollowercommand.Constants.AutoConstants.kMaxSpeedMetersPerSecond;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.examples.frisbeebot.Constants.AutoConstants;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {

  //Robot swerve modules
  private final SwerveModule m_frontLeft = new SwerveModule(kFrontLeftDriveMotorPort,
    kFrontLeftTurningMotorPort,
    kFrontLeftDriveEncoderPorts,
    kFrontLeftTurningEncoderPorts,
    kFrontLeftDriveEncoderReversed,
    kFrontLeftTurningEncoderReversed);

  private final SwerveModule m_rearLeft = new SwerveModule(kRearLeftDriveMotorPort,
    kRearLeftTurningMotorPort,
    kRearLeftDriveEncoderPorts,
    kRearLeftTurningEncoderPorts,
    kRearLeftDriveEncoderReversed,
    kRearLeftTurningEncoderReversed);


  private final SwerveModule m_frontRight = new SwerveModule(kFrontRightDriveMotorPort,
    kFrontRightTurningMotorPort,
    kFrontRightDriveEncoderPorts,
    kFrontRightTurningEncoderPorts,
    kFrontRightDriveEncoderReversed,
    kFrontRightTurningEncoderReversed);

  private final SwerveModule m_rearRight = new SwerveModule(kRearRightDriveMotorPort,
    kRearRightTurningMotorPort,
    kRearRightDriveEncoderPorts,
    kRearRightTurningEncoderPorts,
    kRearRightDriveEncoderReversed,
    kRearRightTurningEncoderReversed);

  // The gyro sensor
  private final Gyro m_gyro = new ADXRS450_Gyro();

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(kDriveKinematics);

  /**
   * Creates a new DriveSubsystem.
   */
  public DriveSubsystem() {}

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
      new Rotation2d(getHeading()),
      m_frontLeft.getState(),
      m_rearLeft.getState(),
      m_frontRight.getState(),
      m_rearRight.getState()

      );
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    var swerveModuleStates = kDriveKinematics.toSwerveModuleStates(
        fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
            xSpeed, ySpeed, rot, new Rotation2d(-m_gyro.getAngle()))
            : new ChassisSpeeds(xSpeed, ySpeed, rot)
    );
    SwerveDriveKinematics.normalizeWheelSpeeds(swerveModuleStates, kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
  * Sets the swerve ModuleStates.
  *
  * @param desiredStates  The desired SwerveModule states.
  */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.normalizeWheelSpeeds(desiredStates, kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /**
   * Resets the drive encoders to currently read a position of 0.
   */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /**
   * Zeroes the heading of the robot.
   */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from 180 to 180
   */
  public double getHeading() {
    return Math.IEEEremainder(m_gyro.getAngle(), 360) * (kGyroReversed ? -1. : 1.);
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (kGyroReversed ? -1. : 1.);
  }
}
