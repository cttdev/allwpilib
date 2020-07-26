/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpiutil.math;

import java.util.Objects;

import org.ejml.MatrixDimensionException;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.MatrixFeatures_DDRM;
import org.ejml.dense.row.NormOps_DDRM;
import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.wpiutil.WPIUtilJNI;
import edu.wpi.first.wpiutil.math.numbers.N1;

/**
 * A shape-safe wrapper over Efficient Java Matrix Library (EJML) matrices.
 *
 * <p>This class is intended to be used alongside the state space library.
 *
 * @param <R> The number of rows in this matrix.
 * @param <C> The number of columns in this matrix.
 */
@SuppressWarnings("PMD.TooManyMethods")
public class Matrix<R extends Num, C extends Num> {
  final SimpleMatrix m_storage;

  /**
   * Constructs an empty zero matrix of the given dimensions.
   *
   * @param rows    The number of rows of the matrix.
   * @param columns The number of columns of the matrix.
   */
  public Matrix(Nat<R> rows, Nat<C> columns) {
    this.m_storage = new SimpleMatrix(
      Objects.requireNonNull(rows).getNum(),
      Objects.requireNonNull(columns).getNum()
    );
  }

  /**
   * Constructs a new matrix with the given storage.
   * Caller should make sure that the provided generic bounds match the shape of the provided matrix
   *
   * @param storage The {@link SimpleMatrix} to back this value.
   */
  Matrix(SimpleMatrix storage) {
    this.m_storage = Objects.requireNonNull(storage);
  }

  /**
   * Constructs a new matrix with the storage of the supplied matrix.
   *
   * @param other The {@link Matrix} to copy the storage of
   */
  public Matrix(Matrix<R, C> other) {
    this.m_storage = Objects.requireNonNull(other).m_storage;
  }

  /**
   * Gets the number of columns in this matrix.
   *
   * @return The number of columns, according to the internal storage.
   */
  public final int getNumCols() {
    return this.m_storage.numCols();
  }

  /**
   * Gets the number of rows in this matrix.
   *
   * @return The number of rows, according to the internal storage.
   */
  public final int getNumRows() {
    return this.m_storage.numRows();
  }

  /**
   * Get an element of this matrix.
   *
   * @param row The row of the element.
   * @param col The column of the element.
   * @return The element in this matrix at row,col.
   */
  public final double get(int row, int col) {
    return this.m_storage.get(row, col);
  }

  /**
   * Sets the value at the given indices.
   *
   * @param row   The row of the element.
   * @param col   The column of the element.
   * @param value The value to insert at the given location.
   */
  public final void set(int row, int col, double value) {
    this.m_storage.set(row, col, value);
  }

  /**
   * Sets a row to a given row vector.
   *
   * @param row The row to set.
   * @param val The row vector to set the given row to.
   */
  public final void setRow(int row, Matrix<N1, C> val) {
    this.m_storage.setRow(row, 0,
        Objects.requireNonNull(val).m_storage.getDDRM().getData());
  }

  /**
   * Sets a column to a given column vector.
   *
   * @param column The column to set.
   * @param val    The column vector to set the given row to.
   */
  public final void setColumn(int column, Matrix<R, N1> val) {
    this.m_storage.setColumn(column, 0,
        Objects.requireNonNull(val).m_storage.getDDRM().getData());
  }


  /**
   * Sets all the elements in this matrix equal to the specified value.
   *
   * @param value The value each element is set to.
   */
  public void fill(double value) {
    this.m_storage.fill(value);
  }

  /**
   * If a vector then a square matrix is returned
   * if a matrix then a vector of diagonal elements is returned.
   *
   * @return Diagonal elements inside a vector or a square matrix with the same diagonal elements.
   */
  public final Matrix<R, C> diag() {
    return new Matrix<>(this.m_storage.diag());
  }

  /**
   * Returns the largest element of this matrix.
   *
   * @return The largest element of this matrix.
   */
  public final double maxInternal() {
    return CommonOps_DDRM.elementMax(this.m_storage.getDDRM());
  }

  /**
   * Returns the smallest element of this matrix.
   *
   * @return The smallest element of this matrix.
   */
  public final double minInternal() {
    return CommonOps_DDRM.elementMin(this.m_storage.getDDRM());
  }

  /**
   * Calculates the mean of the elements in this matrix.
   *
   * @return The mean value of this matrix.
   */
  public final double mean() {
    return this.elementSum() / (double) this.m_storage.getNumElements();
  }

  /**
   * Multiplies this matrix with another that has C rows.
   *
   * <p>As matrix multiplication is only defined if the number of columns
   * in the first matrix matches the number of rows in the second,
   * this operation will fail to compile under any other circumstances.
   *
   * @param other The other matrix to multiply by.
   * @param <C2>  The number of columns in the second matrix.
   * @return The result of the matrix multiplication between this and the given matrix.
   */
  public final <C2 extends Num> Matrix<R, C2> times(Matrix<C, C2> other) {
    return new Matrix<>(this.m_storage.mult(Objects.requireNonNull(other).m_storage));
  }

  /**
   * Multiplies all the elements of this matrix by the given scalar.
   *
   * @param value The scalar value to multiply by.
   * @return A new matrix with all the elements multiplied by the given value.
   */
  public final Matrix<R, C> times(double value) {
    return new Matrix<>(this.m_storage.scale(value));
  }

  /**
   * <p>
   * Returns a matrix which is the result of an element by element multiplication of 'this' and 'b'.
   * c<sub>i,j</sub> = a<sub>i,j</sub>*b<sub>i,j</sub>
   * </p>
   *
   * @param other A matrix.
   * @return The element by element multiplication of 'this' and 'b'.
   */
  public final Matrix<R, C> elementTimes(Matrix<R, C> other) {
    return new Matrix<>(this.m_storage.elementMult(Objects.requireNonNull(other).m_storage));
  }

  /**
   * Subtracts the given value from all the elements of this matrix.
   *
   * @param value The value to subtract.
   * @return The resultant matrix.
   */
  public final Matrix<R, C> minus(double value) {
    return new Matrix<>(this.m_storage.minus(value));
  }


  /**
   * Subtracts the given matrix from this matrix.
   *
   * @param value The matrix to subtract.
   * @return The resultant matrix.
   */
  public final Matrix<R, C> minus(Matrix<R, C> value) {
    return new Matrix<>(this.m_storage.minus(Objects.requireNonNull(value).m_storage));
  }


  /**
   * Adds the given value to all the elements of this matrix.
   *
   * @param value The value to add.
   * @return The resultant matrix.
   */
  public final Matrix<R, C> plus(double value) {
    return new Matrix<>(this.m_storage.plus(value));
  }

  /**
   * Adds the given matrix to this matrix.
   *
   * @param value The matrix to add.
   * @return The resultant matrix.
   */
  public final Matrix<R, C> plus(Matrix<R, C> value) {
    return new Matrix<>(this.m_storage.plus(Objects.requireNonNull(value).m_storage));
  }

  /**
   * Divides all elements of this matrix by the given value.
   *
   * @param value The value to divide by.
   * @return The resultant matrix.
   */
  public final Matrix<R, C> div(int value) {
    return new Matrix<>(this.m_storage.divide((double) value));
  }

  /**
   * Divides all elements of this matrix by the given value.
   *
   * @param value The value to divide by.
   * @return The resultant matrix.
   */
  public final Matrix<R, C> div(double value) {
    return new Matrix<>(this.m_storage.divide(value));
  }

  /**
   * Calculates the transpose, M^T of this matrix.
   *
   * @return The transpose matrix.
   */
  public final Matrix<C, R> transpose() {
    return new Matrix<>(this.m_storage.transpose());
  }


  /**
   * Returns a copy of this matrix.
   *
   * @return A copy of this matrix.
   */
  public final Matrix<R, C> copy() {
    return new Matrix<>(this.m_storage.copy());
  }


  /**
   * Returns the inverse matrix of this matrix.
   *
   * @return The inverse of this matrix.
   * @throws org.ejml.data.SingularMatrixException If this matrix is non-invertable.
   */
  public final Matrix<R, C> inv() {
    return new Matrix<>(this.m_storage.invert());
  }

  /**
   * Returns the solution x to the equation Ax = b, where A is the matrix
   * "this".
   *
   * @param b The right-hand side of the equation to solve.
   * @return The solution to the linear system.
   */
  public final Matrix<R, N1> solve(Matrix<R, N1> b) {
    return new Matrix<>(this.m_storage.solve(Objects.requireNonNull(b).m_storage));
  }

  /**
   * Computes the matrix exponential using Eigen's solver.
   * This method only works for square matrices, and will
   * otherwise throw an {@link MatrixDimensionException}.
   *
   * @return the exponential of A.
   */
  public final Matrix<R, C> exp() {
    if (this.getNumRows() != this.getNumCols()) {
      throw new MatrixDimensionException("Non-square matrices cannot be exponentiated! "
            + "This matrix is " + this.getNumRows() + " x " + this.getNumCols());
    }
    Matrix<R, C> toReturn = new Matrix<>(new SimpleMatrix(this.getNumRows(), this.getNumCols()));
    WPIUtilJNI.exp(this.m_storage.getDDRM().getData(), this.getNumRows(),
          toReturn.m_storage.getDDRM().getData());
    return toReturn;
  }

  /**
   * Returns the determinant of this matrix.
   *
   * @return The determinant of this matrix.
   */
  public final double det() {
    return this.m_storage.determinant();
  }

  /**
   * Computes the Frobenius normal of the matrix.<br>
   * <br>
   * normF = Sqrt{  &sum;<sub>i=1:m</sub> &sum;<sub>j=1:n</sub> { a<sub>ij</sub><sup>2</sup>}   }
   *
   * @return The matrix's Frobenius normal.
   */
  public final double normF() {
    return this.m_storage.normF();
  }

  /**
   * Computes the induced p = 1 matrix norm.<br>
   * <br>
   * ||A||<sub>1</sub>= max(j=1 to n; sum(i=1 to m; |a<sub>ij</sub>|))
   *
   * @return The norm.
   */
  public final double normIndP1() {
    return NormOps_DDRM.inducedP1(this.m_storage.getDDRM());
  }

  /**
   * Computes the sum of all the elements in the matrix.
   *
   * @return Sum of all the elements.
   */
  public final double elementSum() {
    return this.m_storage.elementSum();
  }

  /**
   * Computes the trace of the matrix.
   *
   * @return The trace of the matrix.
   */
  public final double trace() {
    return this.m_storage.trace();
  }

  /**
   * Returns a matrix which is the result of an element by element power of 'this' and 'b':
   * c<sub>i,j</sub> = a<sub>i,j</sub> ^ b.
   *
   * @param b Scalar
   * @return The element by element power of 'this' and 'b'.
   */
  @SuppressWarnings("ParameterName")
  public final Matrix<R, C> epow(double b) {
    return new Matrix<>(this.m_storage.elementPower(b));
  }

  /**
   * Returns a matrix which is the result of an element by element power of 'this' and 'b':
   * c<sub>i,j</sub> = a<sub>i,j</sub> ^ b.
   *
   * @param b Scalar.
   * @return The element by element power of 'this' and 'b'.
   */
  @SuppressWarnings("ParameterName")
  public final Matrix<R, C> epow(int b) {
    return new Matrix<>(this.m_storage.elementPower((double) b));
  }

  /**
   * Extracts a given row into a row vector with new underlying storage.
   *
   * @param row The row to extract a vector from.
   * @return A row vector from the given row.
   */
  public final Matrix<N1, C> extractRowVector(int row) {
    return new Matrix<>(this.m_storage.extractVector(true, row));
  }

  /**
   * Extracts a given column into a column vector with new underlying storage.
   *
   * @param column The column to extract a vector from.
   * @return A column vector from the given column.
   */
  public final Matrix<R, N1> extractColumnVector(int column) {
    return new Matrix<>(this.m_storage.extractVector(false, column));
  }

  /**
   * Extracts a matrix of a given size and start position with new underlying
   * storage.
   *
   * @param height The number of rows of the extracted matrix.
   * @param width  The number of columns of the extracted matrix.
   * @param startingRow The starting row of the extracted matrix.
   * @param startingCol The starting column of the extracted matrix.
   * @return The extracted matrix.
   */
  public final <R2 extends Num, C2 extends Num> Matrix<R2, C2> block(
      Nat<R2> height, Nat<C2> width, int startingRow, int startingCol) {
    return new Matrix<>(this.m_storage.extractMatrix(
      startingRow,
      Objects.requireNonNull(height).getNum() + startingRow,
      startingCol,
      Objects.requireNonNull(width).getNum() + startingCol));
  }

  /**
   * Assign a matrix of a given size and start position.
   *
   * @param height The number of rows of the extracted matrix.
   * @param width  The number of columns of the extracted matrix.
   * @param other  The matrix to assign the block to.
   */
  public <R2 extends Num, C2 extends Num> void assignBlock(int startingRow, int startingCol, Matrix<R2, C2> other) {
    this.m_storage.insertIntoThis(startingRow, startingCol, Objects.requireNonNull(other).m_storage);
  }

  @Override
  public String toString() {
    return m_storage.toString();
  }

  /**
   * Creates a new vector of zeros.
   *
   * @param nums The size of the desired vector.
   * @param <N> The size of the desired vector as a generic.
   * @return A vector of size N filled with zeros.
   */
  public static <N extends Num> Matrix<N, N1> zeros(Nat<N> nums) {
    return new Matrix<>(new SimpleMatrix(Objects.requireNonNull(nums).getNum(), 1));
  }

  /**
   * Creates the identity matrix of the given dimension.
   *
   * @param dim The dimension of the desired matrix.
   * @param <D> The dimension of the desired matrix as a generic.
   * @return The DxD identity matrix.
   */
  public static <D extends Num> Matrix<D, D> eye(Nat<D> dim) {
    return new Matrix<>(SimpleMatrix.identity(Objects.requireNonNull(dim).getNum()));
  }

  /**
   * Creates the identity matrix of the given dimension.
   *
   * @param dim The dimension of the desired matrix.
   * @param <D> The dimension of the desired matrix.
   * @return The DxD identity matrix.
   */
  public static <D extends Num> Matrix<D, D> eye(D dim) {
    return new Matrix<>(SimpleMatrix.identity(Objects.requireNonNull(dim).getNum()));
  }

  /**
   * Entrypoint to the MatBuilder class for creation
   * of custom matrices with the given dimensions and contents.
   *
   * @param rows The number of rows of the desired matrix.
   * @param cols The number of columns of the desired matrix.
   * @param <R> The number of rows of the desired matrix as a generic.
   * @param <C> The number of columns of the desired matrix as a generic.
   * @return A builder to construct the matrix.
   */
  public static <R extends Num, C extends Num> MatBuilder<R, C> mat(Nat<R> rows, Nat<C> cols) {
    return new MatBuilder<>(Objects.requireNonNull(rows), Objects.requireNonNull(cols));
  }

  
  /** 
   * Checks if another Matrix is identical to this one within a specified tolerance.
   *
   * <p>This will check if each element is in tolerance of the corresponding element
   * from the other Matrix or if the elements have the same symbolic meaning. For two
   * elements to have the same symbolic meaning they both must be either Double.NaN,
   * Double.POSITIVE_INFINITY, or Double.NEGATIVE_INFINITY.
   * 
   * <p>NOTE:It is recommend to use {@link Matrix#equals(Matrix, double)} over this
   * when checking if two matrices are equal as {@link Matrix#equals(Matrix, double)}
   * will return false if an element is uncountable. This should only be used when
   * uncountable elements need to compared.
   * 
   * @param other     The Matrix to check against this one.
   * @param tolerance The tolerance to check equality with.
   * @return true if this Matrix is identical to the one supplied.
   */
  public boolean isIdentical(Matrix<?, ?> other, double tolerance) {
    return MatrixFeatures_DDRM.isIdentical(this.m_storage.getDDRM(),
        other.m_storage.getDDRM(), tolerance);
  }

  /** 
   * Checks if another Matrix is equal to this one within a specified tolerance.
   * 
   * <p>This will check if each element is in tolerance of the corresponding element
   * from the other Matrix.
   * 
   * @param other     The Matrix to check against this one.
   * @param tolerance The tolerance to check equality with.
   * @return true if this Matrix is equal to the one supplied.
   */
  public boolean equals(Matrix<?, ?> other, double tolerance) {
    return MatrixFeatures_DDRM.isEquals(this.m_storage.getDDRM(),
        other.m_storage.getDDRM(), tolerance);
  }

  
  @Override
  public boolean equals(Object o) {
    if (this == o) return true;
    if (o == null || getClass() != o.getClass()) return false;

    Matrix<?, ?> matrix = (Matrix<?, ?>) o;
    if (MatrixFeatures_DDRM.hasUncountable(matrix.m_storage.getDDRM())) return false;
    return MatrixFeatures_DDRM.isEquals(this.m_storage.getDDRM(), matrix.m_storage.getDDRM());
  }
}
