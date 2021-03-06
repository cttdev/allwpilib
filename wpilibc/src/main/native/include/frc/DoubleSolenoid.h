// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <memory>

#include <hal/Types.h>

#include "frc/PneumaticsBase.h"
#include "frc/smartdashboard/Sendable.h"
#include "frc/smartdashboard/SendableHelper.h"

namespace frc {

class SendableBuilder;

/**
 * DoubleSolenoid class for running 2 channels of high voltage Digital Output
 * (PCM).
 *
 * The DoubleSolenoid class is typically used for pneumatics solenoids that
 * have two positions controlled by two separate channels.
 */
class DoubleSolenoid : public Sendable, public SendableHelper<DoubleSolenoid> {
 public:
  enum Value { kOff, kForward, kReverse };

  DoubleSolenoid(PneumaticsBase& module, int forwardChannel,
                 int reverseChannel);
  DoubleSolenoid(PneumaticsBase* module, int forwardChannel,
                 int reverseChannel);
  DoubleSolenoid(std::shared_ptr<PneumaticsBase> module, int forwardChannel,
                 int reverseChannel);

  ~DoubleSolenoid() override;

  DoubleSolenoid(DoubleSolenoid&&) = default;
  DoubleSolenoid& operator=(DoubleSolenoid&&) = default;

  /**
   * Set the value of a solenoid.
   *
   * @param value The value to set (Off, Forward or Reverse)
   */
  virtual void Set(Value value);

  /**
   * Read the current value of the solenoid.
   *
   * @return The current value of the solenoid.
   */
  virtual Value Get() const;

  /**
   * Toggle the value of the solenoid.
   *
   * If the solenoid is set to forward, it'll be set to reverse. If the solenoid
   * is set to reverse, it'll be set to forward. If the solenoid is set to off,
   * nothing happens.
   */
  void Toggle();

  /**
   * Get the forward channel.
   *
   * @return the forward channel.
   */
  int GetFwdChannel() const;

  /**
   * Get the reverse channel.
   *
   * @return the reverse channel.
   */
  int GetRevChannel() const;

  /**
   * Check if the forward solenoid is Disabled.
   *
   * If a solenoid is shorted, it is added to the DisabledList and disabled
   * until power cycle, or until faults are cleared.
   *
   * @see ClearAllPCMStickyFaults()
   * @return If solenoid is disabled due to short.
   */
  bool IsFwdSolenoidDisabled() const;

  /**
   * Check if the reverse solenoid is Disabled.
   *
   * If a solenoid is shorted, it is added to the DisabledList and disabled
   * until power cycle, or until faults are cleared.
   *
   * @see ClearAllPCMStickyFaults()
   * @return If solenoid is disabled due to short.
   */
  bool IsRevSolenoidDisabled() const;

  void InitSendable(SendableBuilder& builder) override;

 private:
  int m_forwardChannel;  // The forward channel on the module to control.
  int m_reverseChannel;  // The reverse channel on the module to control.
  int m_forwardMask;     // The mask for the forward channel.
  int m_reverseMask;     // The mask for the reverse channel.
  int m_mask;
  std::shared_ptr<PneumaticsBase> m_module;
};

}  // namespace frc
