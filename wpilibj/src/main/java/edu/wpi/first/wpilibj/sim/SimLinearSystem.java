/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.sim;

import edu.wpi.first.wpilibj.math.StateSpaceUtil;
import edu.wpi.first.wpilibj.system.LinearSystem;
import edu.wpi.first.wpiutil.math.Matrix;
import edu.wpi.first.wpiutil.math.Num;
import edu.wpi.first.wpiutil.math.numbers.N1;
import org.ejml.MatrixDimensionException;
import org.ejml.simple.SimpleMatrix;

/**
 * This class helps simulate linear systems. To use this class, do the following in the
 * {@link edu.wpi.first.wpilibj.IterativeRobotBase#simulationPeriodic} method.
 *
 * <p>Call {@link #setInput(double...)} with the inputs to the system (usually voltage).
 *
 * <p>Call {@link #update} to update the simulation.
 *
 * <p>Set simulated sensor readings with the simulated positions in {@link #getOutput(int)}
 *
 * @param <S> The number of states of the system.
 * @param <I> The number of inputs to the system.
 * @param <O> The number of outputs of the system.
 */
public class SimLinearSystem<S extends Num, I extends Num,
      O extends Num> {

  protected final LinearSystem<S, I, O> m_plant;

  protected boolean m_shouldAddNoise;

  protected Matrix<S, N1> m_trueXhat;
  @SuppressWarnings("MemberName")
  protected Matrix<O, N1> m_y;
  @SuppressWarnings("MemberName")
  protected Matrix<I, N1> m_u;
  protected final Matrix<O, N1> m_measurementStdDevs;

  /**
   * Create a SimLinearSystem. This simulator uses an {@link LinearSystem} to simulate
   * the state of the system. In simulationPeriodic, users should first set inputs from motors, update
   * the simulation and write simulated outputs to sensors.
   *
   * @param system             The system being controlled.
   * @param addNoise           If we should add noise to the measurement vector
   * @param measurementStdDevs Standard deviations of measurements. Can be null if addNoise is false.
   */
  public SimLinearSystem(LinearSystem<S, I, O> system, boolean addNoise, Matrix<O, N1> measurementStdDevs) {
    this.m_plant = system;
    this.m_shouldAddNoise = addNoise;
    this.m_measurementStdDevs = measurementStdDevs;

    m_trueXhat = new Matrix<>(new SimpleMatrix(system.getA().getNumRows(), 1));
    m_u = new Matrix<>(new SimpleMatrix(system.getB().getNumCols(), 1));
    m_y = new Matrix<>(new SimpleMatrix(system.getC().getNumRows(), 1));

  }

  public boolean getShouldAddNoise() {
    return m_shouldAddNoise;
  }

  public void setShouldAddNoise(boolean m_shouldAddNoise) {
    this.m_shouldAddNoise = m_shouldAddNoise;
  }

  protected Matrix<S, N1> updateXhat(Matrix<S, N1> currentXhat, Matrix<I, N1> u, double dtSeconds) {
    return m_plant.calculateX(currentXhat, u, dtSeconds);
  }

  @SuppressWarnings("LocalVariableName")
  public void update(double dtSeconds) {

    // x = ax + bu
    m_trueXhat = updateXhat(m_trueXhat, m_u, dtSeconds);

    // y = cx + du
    m_y = m_plant.calculateY(m_trueXhat, m_u);
    if (m_shouldAddNoise && m_measurementStdDevs != null) {
      m_y = m_y.plus(StateSpaceUtil.makeWhiteNoiseVector(m_measurementStdDevs));
    }
  }

  public Matrix<O, N1> getY() {
    return m_y;
  }

  public double getY(int row) {
    return m_y.get(row, 0);
  }

  public void setInput(Matrix<I, N1> u) {
    this.m_u = u;
  }

  public void setInput(int row, double value) {
    m_u.set(row, 0, value);
  }

  public void setInput(double... u) {
    if (u.length != m_u.getNumRows()) {
      throw new MatrixDimensionException("Malformed input! Got " + u.length
            + " elements instead of " + m_u.getNumRows());
    }
    m_u = new Matrix<>(new SimpleMatrix(m_u.getNumRows(), 1, true, u));
  }

  public Matrix<O, N1> getOutput() {
    return m_y;
  }

  public double getOutput(int row) {
    return m_y.get(row, 0);
  }

  /**
   * Get the current drawn by this simulated system. Override this method to add current
   * calculation.
   *
   * @return The currently drawn current.
   */
  public double getCurrentDrawAmps() {
    return 0.0;
  }
}
