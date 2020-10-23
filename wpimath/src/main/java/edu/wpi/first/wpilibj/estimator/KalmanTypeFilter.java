/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.estimator;

import edu.wpi.first.wpiutil.math.Matrix;
import edu.wpi.first.wpiutil.math.Nat;
import edu.wpi.first.wpiutil.math.Num;
import edu.wpi.first.wpiutil.math.numbers.N1;

import java.util.function.BiFunction;

@SuppressWarnings({"ParameterName", "InterfaceTypeParameterName"})
interface KalmanTypeFilter<S extends Num, I extends Num, O extends Num> {
  Matrix<S, S> getP();

  double getP(int i, int j);

  void setP(Matrix<S, S> newP);

  Matrix<S, N1> getXhat();

  double getXhat(int i);

  void setXhat(Matrix<S, N1> xHat);

  void setXhat(int i, double value);

  void reset();

  void predict(Matrix<I, N1> u, Matrix<S, S> q, double dtSeconds);

  void correct(Matrix<I, N1> u, Matrix<O, N1> y);

  <R extends Num> void correct(Nat<R> rows, Matrix<I, N1> u, Matrix<R, N1> y, BiFunction<Matrix<S, N1>,
      Matrix<I, N1>, Matrix<R, N1>> localMeasurementModel, Matrix<R, R> r);
}
