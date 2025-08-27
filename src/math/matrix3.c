/******************************************************************************
 * ISA Flight Software
 * 
 * File: matrix3.c
 * Description: Implementation of 3x3 matrix operations
 *****************************************************************************/

#include <stdio.h>
#include <math.h>
#include "../../include/math/matrix3.h"

Matrix3 Matrix3_Create(
    double m11, double m12, double m13,
    double m21, double m22, double m23,
    double m31, double m32, double m33
) {
    Matrix3 result;
    
    result.m11 = m11; result.m12 = m12; result.m13 = m13;
    result.m21 = m21; result.m22 = m22; result.m23 = m23;
    result.m31 = m31; result.m32 = m32; result.m33 = m33;
    
    return result;
}

Matrix3 Matrix3_Identity(void) {
    return Matrix3_Create(
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0
    );
}

Matrix3 Matrix3_FromColumns(Vector3 col1, Vector3 col2, Vector3 col3) {
    return Matrix3_Create(
        col1.a, col2.a, col3.a,
        col1.b, col2.b, col3.b,
        col1.c, col2.c, col3.c
    );
}

Matrix3 Matrix3_Add(Matrix3 m1, Matrix3 m2) {
    return Matrix3_Create(
        m1.m11 + m2.m11, m1.m12 + m2.m12, m1.m13 + m2.m13,
        m1.m21 + m2.m21, m1.m22 + m2.m22, m1.m23 + m2.m23,
        m1.m31 + m2.m31, m1.m32 + m2.m32, m1.m33 + m2.m33
    );
}

Matrix3 Matrix3_Subtract(Matrix3 m1, Matrix3 m2) {
    return Matrix3_Create(
        m1.m11 - m2.m11, m1.m12 - m2.m12, m1.m13 - m2.m13,
        m1.m21 - m2.m21, m1.m22 - m2.m22, m1.m23 - m2.m23,
        m1.m31 - m2.m31, m1.m32 - m2.m32, m1.m33 - m2.m33
    );
}

Matrix3 Matrix3_ScalarMultiply(Matrix3 m, double scalar) {
    return Matrix3_Create(
        m.m11 * scalar, m.m12 * scalar, m.m13 * scalar,
        m.m21 * scalar, m.m22 * scalar, m.m23 * scalar,
        m.m31 * scalar, m.m32 * scalar, m.m33 * scalar
    );
}

Matrix3 Matrix3_Multiply(Matrix3 m1, Matrix3 m2) {
    return Matrix3_Create(
        /* First row */
        m1.m11 * m2.m11 + m1.m12 * m2.m21 + m1.m13 * m2.m31,
        m1.m11 * m2.m12 + m1.m12 * m2.m22 + m1.m13 * m2.m32,
        m1.m11 * m2.m13 + m1.m12 * m2.m23 + m1.m13 * m2.m33,
        
        /* Second row */
        m1.m21 * m2.m11 + m1.m22 * m2.m21 + m1.m23 * m2.m31,
        m1.m21 * m2.m12 + m1.m22 * m2.m22 + m1.m23 * m2.m32,
        m1.m21 * m2.m13 + m1.m22 * m2.m23 + m1.m23 * m2.m33,
        
        /* Third row */
        m1.m31 * m2.m11 + m1.m32 * m2.m21 + m1.m33 * m2.m31,
        m1.m31 * m2.m12 + m1.m32 * m2.m22 + m1.m33 * m2.m32,
        m1.m31 * m2.m13 + m1.m32 * m2.m23 + m1.m33 * m2.m33
    );
}

Vector3 Matrix3_MultiplyVector(Matrix3 m, Vector3 v) {
    return Vector3_Create(
        m.m11 * v.a + m.m12 * v.b + m.m13 * v.c,
        m.m21 * v.a + m.m22 * v.b + m.m23 * v.c,
        m.m31 * v.a + m.m32 * v.b + m.m33 * v.c
    );
}

Matrix3 Matrix3_Transpose(Matrix3 m) {
    return Matrix3_Create(
        m.m11, m.m21, m.m31,
        m.m12, m.m22, m.m32,
        m.m13, m.m23, m.m33
    );
}

double Matrix3_Determinant(Matrix3 m) {
    return m.m11 * (m.m22 * m.m33 - m.m23 * m.m32) -
           m.m12 * (m.m21 * m.m33 - m.m23 * m.m31) +
           m.m13 * (m.m21 * m.m32 - m.m22 * m.m31);
}

Matrix3 Matrix3_Inverse(Matrix3 m, Status* status) {
    double det = Matrix3_Determinant(m);
    
    /* Check if matrix is singular (determinant close to zero) */
    if (fabs(det) < EPSILON) {
        if (status) {
            *status = STATUS_MATH_ERROR;
        }
        /* Return identity matrix if inverse doesn't exist */
        return Matrix3_Identity();
    }
    
    double invDet = 1.0 / det;
    
    Matrix3 result;
    
    /* Calculate cofactors and adjugate */
    result.m11 = (m.m22 * m.m33 - m.m23 * m.m32) * invDet;
    result.m12 = (m.m13 * m.m32 - m.m12 * m.m33) * invDet;
    result.m13 = (m.m12 * m.m23 - m.m13 * m.m22) * invDet;
    
    result.m21 = (m.m23 * m.m31 - m.m21 * m.m33) * invDet;
    result.m22 = (m.m11 * m.m33 - m.m13 * m.m31) * invDet;
    result.m23 = (m.m13 * m.m21 - m.m11 * m.m23) * invDet;
    
    result.m31 = (m.m21 * m.m32 - m.m22 * m.m31) * invDet;
    result.m32 = (m.m12 * m.m31 - m.m11 * m.m32) * invDet;
    result.m33 = (m.m11 * m.m22 - m.m12 * m.m21) * invDet;
    
    if (status) {
        *status = STATUS_OK;
    }
    
    return result;
}

void Matrix3_Print(Matrix3 m, const char* name) {
    if (name) {
        printf("%s =\n", name);
    } else {
        printf("Matrix3 =\n");
    }
    
    printf("[%.6f, %.6f, %.6f\n", m.m11, m.m12, m.m13);
    printf(" %.6f, %.6f, %.6f\n", m.m21, m.m22, m.m23);
    printf(" %.6f, %.6f, %.6f]\n", m.m31, m.m32, m.m33);
}
