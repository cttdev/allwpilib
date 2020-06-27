/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "frc/controller/LTVDiffDriveController.h"

#include <cmath>

#include <wpi/MathExtras.h>
#include <wpi/math>

#include "frc/controller/LinearQuadraticRegulator.h"
#include "frc/system/NumericalJacobian.h"

using namespace frc;

LTVDiffDriveController::LTVDiffDriveController(
    const LinearSystem<2, 2, 2>& plant,
    const std::array<double, 5>& controllerQ,
    const std::array<double, 2>& controllerR,
    const DifferentialDriveKinematics& kinematics, units::second_t dt)
    : LTVDiffDriveController(plant, controllerQ, 1.0, controllerR, kinematics,
                             dt) {}

LTVDiffDriveController::LTVDiffDriveController(
    const LinearSystem<2, 2, 2>& plant,
    const std::array<double, 5>& controllerQ, const double rho,
    const std::array<double, 2>& controllerR,
    const DifferentialDriveKinematics& kinematics, units::second_t dt)
    : m_plant(plant),
      m_rb(kinematics.trackWidth / 2.0),
      m_controllerQ(controllerQ),
      m_controllerR(controllerR),
      m_dt(dt),
      m_kinematics(kinematics) {
  Reset();

  m_B = frc::NumericalJacobianU<5, 5, 2>(
      [this](auto& x, auto& u) { return Dynamics(x, u); },
      Eigen::Matrix<double, 5, 1>::Zero(), Eigen::Matrix<double, 2, 1>::Zero());
}

bool LTVDiffDriveController::AtReference() const {
  const auto& tolTranslate = m_poseTolerance.Translation();
  const auto& tolRotate = m_poseTolerance.Rotation();
  return std::abs(m_stateError(0)) < tolTranslate.X().to<double>() &&
         std::abs(m_stateError(1)) < tolTranslate.Y().to<double>() &&
         std::abs(m_stateError(2)) < tolRotate.Radians().to<double>() &&
         std::abs(m_stateError(3)) < m_velocityTolerance.to<double>() &&
         std::abs(m_stateError(4)) < m_velocityTolerance.to<double>();
}

const Vector<5>& LTVDiffDriveController::StateError() const {
  return m_stateError;
}

void LTVDiffDriveController::SetTolerance(
    const Pose2d& poseTolerance, units::meters_per_second_t velocityTolerance) {
  m_poseTolerance = poseTolerance;
  m_velocityTolerance = velocityTolerance;
}

const Vector<5>& LTVDiffDriveController::GetReferences() const {
  return m_nextR;
}

const Vector<2>& LTVDiffDriveController::GetInputs() const {
  return m_uncappedU;
}

const Vector<2>& LTVDiffDriveController::Calculate(
    const Vector<5>& currentState, const Vector<5>& stateRef) {
  m_nextR = stateRef;
  m_stateError = m_nextR - currentState;

  m_uncappedU = Controller(currentState, m_nextR);

  return m_uncappedU;
}

const Vector<2>& LTVDiffDriveController::Calculate(
    const Vector<5>& currentState, const Trajectory::State& desiredState) {
  DifferentialDriveWheelSpeeds wheelVelocities = m_kinematics.ToWheelSpeeds(
      ChassisSpeeds{desiredState.velocity, 0_mps,
                    desiredState.velocity * desiredState.curvature});

  Vector<5> stateRef;

  stateRef << desiredState.pose.Translation().X().to<double>(),
      desiredState.pose.Translation().Y().to<double>(),
      desiredState.pose.Rotation().Radians().to<double>(),
      wheelVelocities.left.to<double>(), wheelVelocities.right.to<double>();

  return Calculate(currentState, stateRef);
}

void LTVDiffDriveController::Reset() {
  m_nextR.setZero();
  m_uncappedU.setZero();
}

Vector<5> LTVDiffDriveController::Dynamics(const Vector<5>& x,
                                           const Vector<2>& u) {
  double v = (x(State::kLeftVelocity) + x(State::kRightVelocity)) / 2.0;

  Eigen::Matrix<double, 5, 1> result;
  result(0, 0) = v * std::cos(x(State::kHeading));
  result(1, 0) = v * std::sin(x(State::kHeading));
  result(2, 0) =
      ((x(State::kRightVelocity) - x(State::kLeftVelocity)) / (2.0 * m_rb))
          .to<double>();
  result.block<2, 1>(3, 0) =
      m_plant.A() * x.block<2, 1>(3, 0) + m_plant.B() * u;
  return result;
}

Vector<2> LTVDiffDriveController::Controller(
    // This implements the linear time-varying differential drive controller in
    // theorem 8.6.4 of https://tavsys.net/controls-in-frc.
    const Eigen::Matrix<double, 5, 1>& x,
    const Eigen::Matrix<double, 5, 1>& r) {
  Eigen::Matrix<double, 2, 5> K = ControllerGainForState(x);

  Eigen::Matrix<double, 5, 5> inRobotFrame =
      Eigen::Matrix<double, 5, 5>::Identity();
  inRobotFrame(0, 0) = std::cos(x(2));
  inRobotFrame(0, 1) = std::sin(x(2));
  inRobotFrame(1, 0) = -std::sin(x(2));
  inRobotFrame(1, 1) = std::cos(x(2));

  Eigen::Matrix<double, 5, 1> error = r - x;
  error(State::kHeading) =
      NormalizeAngle(units::radian_t{error(State::kHeading)}).to<double>();
  return K * inRobotFrame * error;
}

Eigen::Matrix<double, 2, 5> LTVDiffDriveController::ControllerGainForState(
    const Eigen::Matrix<double, 5, 1>& x) {
  // Make the heading zero because the LTV controller controls forward error
  // and cross-track error
  Eigen::Matrix<double, 5, 1> x0 = x;
  x0(State::kHeading) = 0.0;

  // The DARE is ill-conditioned if the velocity is close to zero, so don't
  // let the system stop.
  double velocity =
      (x0(State::kLeftVelocity) + x0(State::kRightVelocity)) / 2.0;
  if (std::abs(velocity) < 1e-9) {
    x0(State::kLeftVelocity) += 2e-9;
    x0(State::kRightVelocity) += 2e-9;
  }

  Eigen::Matrix<double, 5, 5> A = frc::NumericalJacobianX<5, 5, 2>(
      [this](auto& x, auto& u) { return Dynamics(x, u); }, x0,
      Vector<2>::Zero());

  return frc::LinearQuadraticRegulator<5, 2>(A, m_B, m_controllerQ,
                                             m_controllerR, m_dt)
      .K();
}

units::radian_t LTVDiffDriveController::NormalizeAngle(units::radian_t angle) {
  // Constrain theta to within (-3pi, pi)
  const int n_pi_pos =
      (angle.to<double>() + wpi::math::pi) / 2.0 / wpi::math::pi;
  angle -= units::radian_t(n_pi_pos * 2.0 * wpi::math::pi);

  // Cut off the bottom half of the above range to constrain within
  // (-pi, pi]
  const int n_pi_neg =
      (angle.to<double>() - wpi::math::pi) / 2.0 / wpi::math::pi;
  angle -= units::radian_t(n_pi_neg * 2.0 * wpi::math::pi);

  return angle;
}
