/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.examples.mecanumfollowercommand.subsystems;

import static edu.wpi.first.wpilibj.examples.mecanumfollowercommand.Constants.DriveConstants.*;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.MecanumDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  private final PWMVictorSPX m_frontLeft = new PWMVictorSPX(kFrontLeftMotorPort);
  private final PWMVictorSPX m_rearLeft = new PWMVictorSPX(kRearLeftMotorPort);
  private final PWMVictorSPX m_frontRight = new PWMVictorSPX(kFrontRightMotorPort);
  private final PWMVictorSPX m_rearRight = new PWMVictorSPX(kRearRightMotorPort);


  private final MecanumDrive m_drive = new MecanumDrive(m_frontLeft, m_rearLeft, m_frontRight, m_rearRight);

  // The front-left-side drive encoder
  private final Encoder m_frontLeftEncoder =
      new Encoder(kFrontLeftEncoderPorts[0], kFrontLeftEncoderPorts[1], kFrontLeftEncoderReversed);

  // The rear-left-side drive encoder
  private final Encoder m_rearLeftEncoder =
      new Encoder(kRearLeftEncoderPorts[0], kRearLeftEncoderPorts[1], kRearLeftEncoderReversed);

  // The front-right--side drive encoder
  private final Encoder m_frontRightEncoder =
      new Encoder(kFrontRightEncoderPorts[0], kFrontRightEncoderPorts[1], kFrontRightEncoderReversed);

  // The rear-right-side drive encoder
  private final Encoder m_rearRightEncoder =
  new Encoder(kRearRightEncoderPorts[0], kRearRightEncoderPorts[1], kRearRightEncoderReversed);

  // The gyro sensor
  private final Gyro m_gyro = new ADXRS450_Gyro();

  // Odometry class for tracking robot pose
  MecanumDriveOdometry m_odometry = new MecanumDriveOdometry(kDriveKinematics);

  /**
   * Creates a new DriveSubsystem.
   */
  public DriveSubsystem() {
    // Sets the distance per pulse for the encoders
    m_frontLeftEncoder.setDistancePerPulse(kEncoderDistancePerPulse);
    m_rearLeftEncoder.setDistancePerPulse(kEncoderDistancePerPulse);
    m_frontRightEncoder.setDistancePerPulse(kEncoderDistancePerPulse);
    m_rearRightEncoder.setDistancePerPulse(kEncoderDistancePerPulse);
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(new Rotation2d(getHeading()),
                                  new MecanumDriveWheelSpeeds(
                                      m_frontLeftEncoder.getRate(),
                                      m_rearLeftEncoder.getRate(),
                                      m_frontRightEncoder.getRate(),
                                      m_rearRightEncoder.getRate()
                                  ));
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
   * Method to drive the robot.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    if ( fieldRelative ){
      m_drive.driveCartesian(ySpeed, xSpeed, rot, m_gyro.getAngle());
    } else {
      m_drive.driveCartesian(ySpeed, xSpeed, rot);
    }

  }


  /**
  * Sets the front left drive SpeedController to a power from -1 to 1.
  */
  public void setFrontLeftSpeedController(double power) {
    m_frontLeft.set(power);
  }

  /**
  * Sets the rear left drive SpeedController to a power from -1 to 1.
  */
  public void setRearLeftSpeedController(double power) {
    m_rearLeft.set(power);
  }

  /**
  * Sets the front right drive SpeedController to a power from -1 to 1.
  */
  public void setFrontRightSpeedController(double power) {
    m_frontRight.set(power);
  }

  /**
  * Sets the rear right drive SpeedController to a power from -1 to 1.
  */
  public void setRearRightSpeedController(double power) {
    m_rearRight.set(power);
  }


  /**
   * Resets the drive encoders to currently read a position of 0.
   */
  public void resetEncoders() {
    m_frontLeftEncoder.reset();
    m_rearLeftEncoder.reset();
    m_frontRightEncoder.reset();
    m_rearRightEncoder.reset();
  }

  /**
   * Gets the front left drive encoder.
   *
   * @return the front left drive encoder
   */
  public Encoder getFrontLeftEncoder() {
    return m_frontLeftEncoder;
  }

  /**
   * Gets the rear left drive encoder.
   *
   * @return the rear left drive encoder
   */
  public Encoder getRearLeftEncoder() {
    return m_rearLeftEncoder;
  }
  /**
   * Gets the front right drive encoder.
   *
   * @return the front right drive encoder
   */
  public Encoder getFrontRightEncoder() {
    return m_frontRightEncoder;
  }
  /**
   * Gets the rear right drive encoder.
   *
   * @return the rear right encoder
   */
  public Encoder getRearRightEncoder() {
    return m_rearRightEncoder;
  }


  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
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
