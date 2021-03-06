// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <memory>

#include <hal/Types.h>
#include <units/time.h>

#include "frc/PneumaticsBase.h"
#include "frc/smartdashboard/Sendable.h"
#include "frc/smartdashboard/SendableHelper.h"

namespace frc {

class SendableBuilder;

/**
 * Solenoid class for running high voltage Digital Output (PCM).
 *
 * The Solenoid class is typically used for pneumatics solenoids, but could be
 * used for any device within the current spec of the PCM.
 */
class Solenoid : public Sendable, public SendableHelper<Solenoid> {
 public:
  Solenoid(PneumaticsBase& module, int channel);
  Solenoid(PneumaticsBase* module, int channel);
  Solenoid(std::shared_ptr<PneumaticsBase> module, int channel);

  ~Solenoid() override;

  Solenoid(Solenoid&&) = default;
  Solenoid& operator=(Solenoid&&) = default;

  /**
   * Set the value of a solenoid.
   *
   * @param on Turn the solenoid output off or on.
   */
  virtual void Set(bool on);

  /**
   * Read the current value of the solenoid.
   *
   * @return The current value of the solenoid.
   */
  virtual bool Get() const;

  /**
   * Toggle the value of the solenoid.
   *
   * If the solenoid is set to on, it'll be turned off. If the solenoid is set
   * to off, it'll be turned on.
   */
  void Toggle();

  /**
   * Get the channel this solenoid is connected to.
   */
  int GetChannel() const;

  /**
   * Check if solenoid is Disabled.
   *
   * If a solenoid is shorted, it is added to the DisabledList and
   * disabled until power cycle, or until faults are cleared.
   *
   * @see ClearAllPCMStickyFaults()
   *
   * @return If solenoid is disabled due to short.
   */
  bool IsDisabled() const;

  /**
   * Set the pulse duration in the PCM. This is used in conjunction with
   * the startPulse method to allow the PCM to control the timing of a pulse.
   * The timing can be controlled in 0.01 second increments.
   *
   * @param durationSeconds The duration of the pulse, from 0.01 to 2.55
   *                        seconds.
   *
   * @see startPulse()
   */
  void SetPulseDuration(units::second_t duration);

  /**
   * Trigger the PCM to generate a pulse of the duration set in
   * setPulseDuration.
   *
   * @see setPulseDuration()
   */
  void StartPulse();

  void InitSendable(SendableBuilder& builder) override;

 private:
  std::shared_ptr<PneumaticsBase> m_module;
  int m_mask;
  int m_channel;
};

}  // namespace frc
