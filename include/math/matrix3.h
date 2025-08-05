/******************************************************************************
 * ISA Flight Software
 * 
 * File: matrix3.h
 * Description: 3x3 matrix operations for the guidance system
 *****************************************************************************/

#ifndef MATRIX3_H
#define MATRIX3_H

#include "../common/types.h"
#include "vector3.h"

/**
 * @brief 3x3 matrix structure
 * 
 * Matrix is stored in row-major order to match MATLAB's notation:
 * [ m11 m12 m13 ]
 * [ m21 m22 m23 ]
 * [ m31 m32 m33 ]
 */
typedef struct {
    double m11, m12, m13;  /* First row */
    double m21, m22, m23;  /* Second row */
    double m31, m32, m33;  /* Third row */
} Matrix3;

/**
 * @brief Initialize a Matrix3 with given components
 * 
 * @param m11 Element at row 1, column 1
 * @param m12 Element at row 1, column 2
 * @param m13 Element at row 1, column 3
 * @param m21 Element at row 2, column 1
 * @param m22 Element at row 2, column 2
 * @param m23 Element at row 2, column 3
 * @param m31 Element at row 3, column 1
 * @param m32 Element at row 3, column 2
 * @param m33 Element at row 3, column 3
 * @return Matrix3 The initialized matrix
 */
Matrix3 Matrix3_Create(
    double m11, double m12, double m13,
    double m21, double m22, double m23,
    double m31, double m32, double m33
);

/**
 * @brief Create an identity matrix
 * 
 * @return Matrix3 The 3x3 identity matrix
 */
Matrix3 Matrix3_Identity(void);

/**
 * @brief Create a rotation matrix from three column vectors
 * 
 * @param col1 First column vector
 * @param col2 Second column vector
 * @param col3 Third column vector
 * @return Matrix3 The rotation matrix
 */
Matrix3 Matrix3_FromColumns(Vector3 col1, Vector3 col2, Vector3 col3);

/**
 * @brief Add two matrices
 * 
 * @param m1 First matrix
 * @param m2 Second matrix
 * @return Matrix3 Result of m1 + m2
 */
Matrix3 Matrix3_Add(Matrix3 m1, Matrix3 m2);

/**
 * @brief Subtract two matrices
 * 
 * @param m1 First matrix
 * @param m2 Second matrix
 * @return Matrix3 Result of m1 - m2
 */
Matrix3 Matrix3_Subtract(Matrix3 m1, Matrix3 m2);

/**
 * @brief Multiply a matrix by a scalar
 * 
 * @param m The matrix
 * @param scalar The scalar value
 * @return Matrix3 Result of scalar * m
 */
Matrix3 Matrix3_ScalarMultiply(Matrix3 m, double scalar);

/**
 * @brief Multiply two matrices
 * 
 * @param m1 First matrix
 * @param m2 Second matrix
 * @return Matrix3 Result of m1 * m2
 */
Matrix3 Matrix3_Multiply(Matrix3 m1, Matrix3 m2);

/**
 * @brief Multiply a matrix by a vector
 * 
 * @param m The matrix
 * @param v The vector
 * @return Vector3 Result of m * v
 */
Vector3 Matrix3_MultiplyVector(Matrix3 m, Vector3 v);

/**
 * @brief Calculate the transpose of a matrix
 * 
 * @param m The matrix
 * @return Matrix3 The transpose of m
 */
Matrix3 Matrix3_Transpose(Matrix3 m);

/**
 * @brief Calculate the determinant of a matrix
 * 
 * @param m The matrix
 * @return double The determinant of m
 */
double Matrix3_Determinant(Matrix3 m);

/**
 * @brief Calculate the inverse of a matrix
 * 
 * @param m The matrix
 * @param status Pointer to status variable (can be NULL)
 * @return Matrix3 The inverse of m, or identity matrix if m is singular
 */
Matrix3 Matrix3_Inverse(Matrix3 m, Status* status);

/**
 * @brief Print a matrix to stdout (for debugging)
 * 
 * @param m The matrix to print
 * @param name Optional name to display
 */
void Matrix3_Print(Matrix3 m, const char* name);

#endif /* MATRIX3_H */
