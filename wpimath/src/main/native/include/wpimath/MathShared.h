// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <memory>

#include <fmt/format.h>

namespace wpi::math {

enum class MathUsageId {
  kKinematics_DifferentialDrive,
  kKinematics_MecanumDrive,
  kKinematics_SwerveDrive,
  kTrajectory_TrapezoidProfile,
  kFilter_Linear,
  kOdometry_DifferentialDrive,
  kOdometry_SwerveDrive,
  kOdometry_MecanumDrive
};

class MathShared {
 public:
  virtual ~MathShared() = default;
  virtual void ReportErrorV(fmt::string_view format, fmt::format_args args) = 0;
  virtual void ReportUsage(MathUsageId id, int count) = 0;

  template <typename S, typename... Args>
  inline void ReportError(const S& format, Args&&... args) {
    ReportErrorV(format, fmt::make_args_checked<Args...>(format, args...));
  }
};

class MathSharedStore {
 public:
  static MathShared& GetMathShared();

  static void SetMathShared(std::unique_ptr<MathShared> shared);

  static void ReportErrorV(fmt::string_view format, fmt::format_args args) {
    GetMathShared().ReportErrorV(format, args);
  }

  template <typename S, typename... Args>
  static inline void ReportError(const S& format, Args&&... args) {
    ReportErrorV(format, fmt::make_args_checked<Args...>(format, args...));
  }

  static void ReportUsage(MathUsageId id, int count) {
    GetMathShared().ReportUsage(id, count);
  }
};

}  // namespace wpi::math
