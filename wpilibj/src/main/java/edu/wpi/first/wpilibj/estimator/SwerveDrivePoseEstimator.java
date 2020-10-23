/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.estimator;

import java.util.function.BiConsumer;
import java.util.function.BiFunction;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.math.Discretization;
import edu.wpi.first.wpilibj.math.StateSpaceUtil;
import edu.wpi.first.wpiutil.math.Matrix;
import edu.wpi.first.wpiutil.math.Nat;
import edu.wpi.first.wpiutil.math.VecBuilder;
import edu.wpi.first.wpiutil.math.numbers.N1;
import edu.wpi.first.wpiutil.math.numbers.N2;
import edu.wpi.first.wpiutil.math.numbers.N3;
import edu.wpi.first.wpiutil.math.numbers.N4;

/**
 * This class wraps an {@link UnscentedKalmanFilter UnscentedKalmanFilter} to fuse
 * latency-compensated vision measurements with swerve drive encoder velocity measurements.
 * It will correct for noisy measurements and encoder drift. It is intended to be an easy
 * but more accurate drop-in for {@link edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry}.
 *
 * <p>{@link SwerveDrivePoseEstimator#update} should be called every robot loop. If
 * your loops are faster or slower than the default of 0.02s, then you should change
 * the nominal delta time using the secondary constructor:
 * {@link SwerveDrivePoseEstimator#SwerveDrivePoseEstimator(Rotation2d, Pose2d,
 * SwerveDriveKinematics, Matrix, Matrix, Matrix, double)}.
 *
 * <p>{@link SwerveDrivePoseEstimator#addVisionMeasurement} can be called as
 * infrequently as you want; if you never call it, then this class will behave mostly like regular
 * encoder odometry.
 *
 * <p>Our state-space system is:
 *
 * <p><strong> x = [[x, y, cos(theta), sin(theta)]]^T </strong> in the field-coordinate system.
 *
 * <p><strong> u = [[vx, vy, omega]]^T </strong> in the field-coordinate system.
 *
 * <p><strong> y = [[x, y, cos(theta), sin(theta)]]^T </strong> in field coords from vision,
 * or <strong> y = [[cos(theta), sin(theta)]]^T </strong> from the gyro.
 */
public class SwerveDrivePoseEstimator {
  private final UnscentedKalmanFilter<N4, N3, N2> m_observer;
  private final SwerveDriveKinematics m_kinematics;
  private final BiConsumer<Matrix<N3, N1>, Matrix<N4, N1>> m_visionCorrect;
  private final KalmanFilterLatencyCompensator<N4, N3, N2> m_latencyCompensator;

  private final double m_nominalDt; // Seconds
  private double m_prevTimeSeconds = -1.0;

  private Rotation2d m_gyroOffset;
  private Rotation2d m_previousAngle;

  Matrix<N3, N1> m_stateStdDevs;
  Matrix<N1, N1> m_localMeasurementStdDevs;
  Matrix<N3, N1> m_visionMeasurementStdDevs;

  /**
   * Constructs a SwerveDrivePoseEstimator.
   *
   * @param gyroAngle                The current gyro angle.
   * @param initialPoseMeters        The starting pose estimate.
   * @param kinematics               A correctly-configured kinematics object for your drivetrain.
   * @param stateStdDevs             Standard deviations of model states. Increase these numbers to
   *                                 trust your wheel and gyro velocities less. This vector should
   *                                 be in the form [x, y, theta (in rads)].
   * @param localMeasurementStdDevs  Standard deviations of the gyro measurement. Increase this
   *                                 number to trust gyro angle measurements less. This should
   *                                 be in the form [heading (in rads)].
   * @param visionMeasurementStdDevs Standard deviations of the encoder measurements. Increase
   *                                 these numbers to trust vision less. This should be in the
   *                                 form [x, y, theta (in rads)].
   */
  public SwerveDrivePoseEstimator(
          Rotation2d gyroAngle, Pose2d initialPoseMeters, SwerveDriveKinematics kinematics,
          Matrix<N3, N1> stateStdDevs, Matrix<N1, N1> localMeasurementStdDevs,
          Matrix<N3, N1> visionMeasurementStdDevs
  ) {
    this(gyroAngle, initialPoseMeters, kinematics, stateStdDevs, localMeasurementStdDevs,
            visionMeasurementStdDevs, 0.02);
  }

  /**
   * Constructs a SwerveDrivePoseEstimator.
   *
   * @param gyroAngle                The current gyro angle.
   * @param initialPoseMeters        The starting pose estimate.
   * @param kinematics               A correctly-configured kinematics object for your drivetrain.
   * @param stateStdDevs             Standard deviations of model states. Increase these numbers to
   *                                 trust your wheel and gyro velocities less.
   * @param localMeasurementStdDevs  Standard deviations of the gyro measurement. Increase this
   *                                 number to trust gyro angle measurements less.
   * @param visionMeasurementStdDevs Standard deviations of the encoder measurements. Increase
   *                                 these numbers to trust vision less.
   * @param nominalDtSeconds         The time in seconds between each robot loop.
   */
  @SuppressWarnings("ParameterName")
  public SwerveDrivePoseEstimator(
          Rotation2d gyroAngle, Pose2d initialPoseMeters, SwerveDriveKinematics kinematics,
          Matrix<N3, N1> stateStdDevs, Matrix<N1, N1> localMeasurementStdDevs,
          Matrix<N3, N1> visionMeasurementStdDevs, double nominalDtSeconds
  ) {
    m_nominalDt = nominalDtSeconds;

    m_observer = new UnscentedKalmanFilter<>(
        Nat.N4(), Nat.N2(),
        this::f,
        (x, u) -> x.block(Nat.N2(), Nat.N1(), 2, 0),
        makeQDiagonals(stateStdDevs, VecBuilder.fill(0.0, 0.0,
            initialPoseMeters.getRotation().getCos(), initialPoseMeters.getRotation().getSin())),
        makeRDiagonals(localMeasurementStdDevs, VecBuilder.fill(0.0, 0.0,
            initialPoseMeters.getRotation().getCos(), initialPoseMeters.getRotation().getSin())),
        m_nominalDt
    );
    m_kinematics = kinematics;
    m_latencyCompensator = new KalmanFilterLatencyCompensator<>();
    m_stateStdDevs = stateStdDevs;
    m_localMeasurementStdDevs = localMeasurementStdDevs;
    m_visionMeasurementStdDevs = visionMeasurementStdDevs;

    // Create correction mechanism for vision measurements.
    m_visionCorrect = (u, y) -> m_observer.correct(
        Nat.N4(), u, y,
        (x, u_) -> x,
        Discretization.discretizeR(StateSpaceUtil.makeCovarianceMatrix(Nat.N4(),
            makeVisionRDiagonals(visionMeasurementStdDevs, y)), nominalDtSeconds)
    );

    m_gyroOffset = initialPoseMeters.getRotation().minus(gyroAngle);
    m_previousAngle = initialPoseMeters.getRotation();
    m_observer.setXhat(StateSpaceUtil.poseTo4dVector(initialPoseMeters));
  }

  /**
   * Get x-dot given the current state and input. Recall that the state is [x, y, cos(theta),
   * sin(theta)]^T.
   * In our case, x-dot will be [dx/dt, dy/dt, d/dt cos(theta), d/dt sin(theta)].
   * 
   * @param x The current state.
   * @param u The current input. In our case, u = [vx, vy, d/dt theta]^T
   */
  @SuppressWarnings({"ParameterName", "MethodName"})
  private Matrix<N4, N1> f(Matrix<N4, N1> x, Matrix<N3, N1> u) {
    // Need to return [dx/dt, dy/dt, d/dt cos(theta), d/dt sin(theta)]
    // dx/dt and dy/dt are from u.
    // d/dt cos(theta) = -sin(theta) * d/dt(theta) by the chain rule. 
    return VecBuilder.fill(
            u.get(0, 0), u.get(1, 0), -x.get(3, 0) * u.get(2, 0), x.get(2, 0) * u.get(2, 0)
    );
  }

  /**
   * Resets the robot's position on the field.
   *
   * <p>You NEED to reset your encoders (to zero) when calling this method.
   *
   * <p>The gyroscope angle does not need to be reset in the user's robot code.
   * The library automatically takes care of offsetting the gyro angle.
   *
   * @param poseMeters The position on the field that your robot is at.
   * @param gyroAngle  The angle reported by the gyroscope.
   */
  public void resetPosition(Pose2d poseMeters, Rotation2d gyroAngle) {
    m_previousAngle = poseMeters.getRotation();
    m_gyroOffset = getEstimatedPosition().getRotation().minus(gyroAngle);
    m_observer.setXhat(StateSpaceUtil.poseTo4dVector(poseMeters));
  }

  /**
   * Gets the pose of the robot at the current time as estimated by the Unscented Kalman Filter.
   *
   * @return The estimated robot pose in meters.
   */
  public Pose2d getEstimatedPosition() {
    return new Pose2d(
            m_observer.getXhat(0),
            m_observer.getXhat(1),
            new Rotation2d(m_observer.getXhat(2), m_observer.getXhat(3))
    );
  }

  /**
   * Add a vision measurement to the Unscented Kalman Filter. This will correct the
   * odometry pose estimate while still accounting for measurement noise.
   *
   * <p>This method can be called as infrequently as you want, as long as you are
   * calling {@link SwerveDrivePoseEstimator#update} every loop.
   *
   * @param visionRobotPoseMeters The pose of the robot as measured by the vision
   *                              camera.
   * @param timestampSeconds      The timestamp of the vision measurement in seconds. Note that if
   *                              you don't use your own time source by calling
   *                              {@link SwerveDrivePoseEstimator#updateWithTime} then you
   *                              must use a timestamp with an epoch since FPGA startup
   *                              (i.e. the epoch of this timestamp is the same epoch as
   *                              {@link edu.wpi.first.wpilibj.Timer#getFPGATimestamp
   *                              Timer.getFPGATimestamp}.) This means that you should
   *                              use Timer.getFPGATimestamp as your time source or
   *                              sync the epochs.
   */
  public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
    m_latencyCompensator.applyPastGlobalMeasurement(
        m_observer, m_nominalDt,
            StateSpaceUtil.poseTo4dVector(visionRobotPoseMeters),
            m_visionCorrect,
            timestampSeconds
    );
  }

  /**
   * Updates the the Unscented Kalman Filter using only wheel encoder information.
   * This should be called every loop, and the correct loop period must be passed
   * into the constructor of this class.
   *
   * @param gyroAngle    The current gyro angle.
   * @param moduleStates The current velocities and rotations of the swerve modules.
   * @return The estimated pose of the robot in meters.
   */
  public Pose2d update(
          Rotation2d gyroAngle,
          SwerveModuleState... moduleStates
  ) {
    return updateWithTime(Timer.getFPGATimestamp(), gyroAngle, moduleStates);
  }

  /**
   * Updates the the Unscented Kalman Filter using only wheel encoder information.
   * This should be called every loop, and the correct loop period must be passed
   * into the constructor of this class.
   *
   * @param currentTimeSeconds Time at which this method was called, in seconds.
   * @param gyroAngle          The current gyroscope angle.
   * @param moduleStates       The current velocities and rotations of the swerve modules.
   * @return The estimated pose of the robot in meters.
   */
  @SuppressWarnings("LocalVariableName")
  public Pose2d updateWithTime(
          double currentTimeSeconds,
          Rotation2d gyroAngle, SwerveModuleState... moduleStates
  ) {
    double dt = m_prevTimeSeconds >= 0 ? currentTimeSeconds - m_prevTimeSeconds : m_nominalDt;
    m_prevTimeSeconds = currentTimeSeconds;

    var angle = gyroAngle.plus(m_gyroOffset);
    var omega = angle.minus(m_previousAngle).getRadians() / dt;

    var chassisSpeeds = m_kinematics.toChassisSpeeds(moduleStates);
    var fieldRelativeVelocities = new Translation2d(
            chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond
    ).rotateBy(angle);

    var u = VecBuilder.fill(
            fieldRelativeVelocities.getX(),
            fieldRelativeVelocities.getY(),
            omega
    );
    m_previousAngle = angle;

    var localY = VecBuilder.fill(angle.getCos(), angle.getSin());
    var q = StateSpaceUtil.makeCovarianceMatrix(Nat.N4(),
        makeQDiagonals(m_stateStdDevs, m_observer.getXhat()));
    var r = StateSpaceUtil.makeCovarianceMatrix(Nat.N2(),
        makeRDiagonals(m_localMeasurementStdDevs, m_observer.getXhat()));
    BiFunction<Matrix<N4, N1>, Matrix<N3, N1>, Matrix<N2, N1>> model =
        (x, u_) -> x.block(Nat.N2(), Nat.N1(), 2, 0);
    m_latencyCompensator.addObserverState(null, m_observer, u, localY, q, r, model, currentTimeSeconds);
    m_observer.predict(u, q, dt);
    m_observer.correct(Nat.N2(), u, localY, model, r);
    return getEstimatedPosition();
  }

  @SuppressWarnings("ParameterName")
  private static Matrix<N4, N1> makeQDiagonals(Matrix<N3, N1> stdDevs, Matrix<N4, N1> x) {
    return VecBuilder.fill(stdDevs.get(0, 0), stdDevs.get(1, 0),
        stdDevs.get(2, 0) * x.get(2, 0), stdDevs.get(2, 0) * x.get(3, 0));
  }

  @SuppressWarnings("ParameterName")
  private static Matrix<N2, N1> makeRDiagonals(Matrix<N1, N1> stdDevs, Matrix<N4, N1> x) {
    return VecBuilder.fill(stdDevs.get(0, 0) * x.get(2, 0), stdDevs.get(0, 0) * x.get(3, 0));
  }

  @SuppressWarnings("ParameterName")
  private static Matrix<N4, N1> makeVisionRDiagonals(Matrix<N3, N1> stdDevs, Matrix<N4, N1> y) {
    return VecBuilder.fill(stdDevs.get(0, 0), stdDevs.get(1, 0),
        stdDevs.get(2, 0) * y.get(2, 0), stdDevs.get(2, 0) * y.get(3, 0));
  }
}
