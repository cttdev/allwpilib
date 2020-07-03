/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpiutil.math;

import org.ejml.data.SingularMatrixException;
import org.ejml.dense.row.MatrixFeatures_DDRM;
import org.ejml.simple.SimpleMatrix;
import org.junit.jupiter.api.Test;

import edu.wpi.first.wpiutil.math.numbers.N1;
import edu.wpi.first.wpiutil.math.numbers.N2;
import edu.wpi.first.wpiutil.math.numbers.N3;
import edu.wpi.first.wpiutil.math.numbers.N4;

import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

public class MatrixTest {
  @Test
  void testMatrixMultiplication() {
    var mat1 = Matrix.mat(Nat.N2(), Nat.N2())
        .fill(2.0, 1.0,
            0.0, 1.0);
    var mat2 = Matrix.mat(Nat.N2(), Nat.N2())
        .fill(3.0, 0.0,
            0.0, 2.5);

    Matrix<N2, N2> result = mat1.times(mat2);

    assertTrue(MatrixFeatures_DDRM.isEquals(
        Matrix.mat(Nat.N2(), Nat.N2())
        .fill(6.0, 2.5,
            0.0, 2.5).m_storage.getDDRM(),
        result.m_storage.getDDRM()
    ));

    var mat3 = Matrix.mat(Nat.N2(), Nat.N3())
        .fill(1.0, 3.0, 0.5,
            2.0, 4.3, 1.2);
    var mat4 = Matrix.mat(Nat.N3(), Nat.N4())
        .fill(3.0, 1.5, 2.0, 4.5,
            2.3, 1.0, 1.6, 3.1,
            5.2, 2.1, 2.0, 1.0);

    Matrix<N2, N4> result2 = mat3.times(mat4);

    assertTrue(MatrixFeatures_DDRM.isIdentical(
        Matrix.mat(Nat.N2(), Nat.N4())
        .fill(12.5, 5.55, 7.8, 14.3,
            22.13, 9.82, 13.28, 23.53).m_storage.getDDRM(),
        result2.m_storage.getDDRM(),
        1E-9
    ));
  }

  @Test
  void testMatrixVectorMultiplication() {
    var mat = Matrix.mat(Nat.N2(), Nat.N2())
        .fill(1.0, 1.0,
            0.0, 1.0);

    var vec = VecBuilder.fill(3.0, 2.0);

    Matrix<N2, N1> result = mat.times(vec);
    assertTrue(MatrixFeatures_DDRM.isEquals(
        VecBuilder.fill(5.0,
            2.0).m_storage.getDDRM(),
        result.m_storage.getDDRM()
    ));
  }

  @Test
  void testTranspose() {
    Matrix<N3, N1> vec = VecBuilder
        .fill(1.0,
            2.0,
            3.0);

    Matrix<N1, N3> transpose = vec.transpose();

    assertTrue(MatrixFeatures_DDRM.isEquals(
        Matrix.mat(Nat.N1(), Nat.N3()).fill(1.0, 2.0, 3.0).m_storage
        .getDDRM(),
        transpose.m_storage.getDDRM()
    ));
  }

  @Test
  void testInverse() {
    var mat = Matrix.mat(Nat.N3(), Nat.N3())
        .fill(1.0, 3.0, 2.0,
            5.0, 2.0, 1.5,
            0.0, 1.3, 2.5);

    var inv = mat.inv();

    assertTrue(MatrixFeatures_DDRM.isIdentical(
        Matrix.eye(Nat.N3()).m_storage.getDDRM(),
        mat.times(inv).m_storage.getDDRM(),
        1E-9
    ));

    assertTrue(MatrixFeatures_DDRM.isIdentical(
        Matrix.eye(Nat.N3()).m_storage.getDDRM(),
        inv.times(mat).m_storage.getDDRM(),
        1E-9
    ));
  }

  @Test
  void testUninvertableMatrix() {
    var singularMatrix = Matrix.mat(Nat.N2(), Nat.N2())
        .fill(2.0, 1.0,
            2.0, 1.0);

    assertThrows(SingularMatrixException.class, singularMatrix::inv);
  }

  @Test
  void testMatrixScalarArithmetic() {
    var mat = Matrix.mat(Nat.N2(), Nat.N2())
        .fill(1.0, 2.0,
            3.0, 4.0);


    assertTrue(MatrixFeatures_DDRM.isEquals(
        Matrix.mat(Nat.N2(), Nat.N2())
        .fill(3.0, 4.0,
            5.0, 6.0).m_storage.getDDRM(),
        mat.plus(2.0).m_storage.getDDRM()
    ));

    assertTrue(MatrixFeatures_DDRM.isEquals(
        Matrix.mat(Nat.N2(), Nat.N2())
        .fill(0.0, 1.0,
            2.0, 3.0).m_storage.getDDRM(),
        mat.minus(1.0).m_storage.getDDRM()
    ));

    assertTrue(MatrixFeatures_DDRM.isEquals(
        Matrix.mat(Nat.N2(), Nat.N2())
        .fill(2.0, 4.0,
            6.0, 8.0).m_storage.getDDRM(),
        mat.times(2.0).m_storage.getDDRM()
    ));

    assertTrue(MatrixFeatures_DDRM.isIdentical(
        Matrix.mat(Nat.N2(), Nat.N2())
        .fill(0.5, 1.0,
            1.5, 2.0).m_storage.getDDRM(),
        mat.div(2.0).m_storage.getDDRM(),
        1E-3
    ));
  }

  @Test
  void testMatrixMatrixArithmetic() {
    var mat1 = Matrix.mat(Nat.N2(), Nat.N2())
        .fill(1.0, 2.0,
            3.0, 4.0);

    var mat2 = Matrix.mat(Nat.N2(), Nat.N2())
        .fill(5.0, 6.0,
            7.0, 8.0);

    assertTrue(MatrixFeatures_DDRM.isEquals(
        Matrix.mat(Nat.N2(), Nat.N2())
        .fill(-4.0, -4.0,
            -4.0, -4.0).m_storage.getDDRM(),
        mat1.minus(mat2).m_storage.getDDRM()
    ));

    assertTrue(MatrixFeatures_DDRM.isEquals(
        Matrix.mat(Nat.N2(), Nat.N2())
        .fill(6.0, 8.0,
            10.0, 12.0).m_storage.getDDRM(),
        mat1.plus(mat2).m_storage.getDDRM()
    ));
  }

  @Test
  void testMatrixExponential() {
    SimpleMatrix matrix = Matrix.eye(Nat.N2()).m_storage;
    var result = SimpleMatrixUtils.expm(matrix);

    assertTrue(MatrixFeatures_DDRM.isIdentical(
        result.getDDRM(),
        new SimpleMatrix(2, 2, true, new double[]{Math.E, 0, 0, Math.E}).getDDRM(),
        1E-9
    ));

    matrix = new SimpleMatrix(2, 2, true, new double[]{1, 2, 3, 4});
    result = SimpleMatrixUtils.expm(matrix.scale(0.01));

    assertTrue(MatrixFeatures_DDRM.isIdentical(
        result.getDDRM(),
        new SimpleMatrix(2, 2, true, new double[]{1.01035625, 0.02050912,
            0.03076368, 1.04111993}).getDDRM(),
        1E-8
    ));
  }
}
