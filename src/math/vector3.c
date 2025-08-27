/******************************************************************************
 * ISA Flight Software
 * 
 * File: vector3.c
 * Description: Implementation of 3D vector operations
 *****************************************************************************/

#include <stdio.h>
#include <math.h>
#include "../../include/math/vector3.h"

Vector3 Vector3_Create(float a, float b, float c) {
    Vector3 result;
    result.a = a;
    result.b = b;
    result.c = c;
    return result;
}

Vector3 Vector3_Add(Vector3 v1, Vector3 v2) {
    Vector3 result;
    result.a = v1.a + v2.a;
    result.b = v1.b + v2.b;
    result.c = v1.c + v2.c;
    return result;
}

Vector3 Vector3_Subtract(Vector3 v1, Vector3 v2) {
    Vector3 result;
    result.a = v1.a - v2.a;
    result.b = v1.b - v2.b;
    result.c = v1.c - v2.c;
    return result;
}

Vector3 Vector3_ScalarMultiply(Vector3 v, double scalar) {
    Vector3 result;
    result.a = v.a * scalar;
    result.b = v.b * scalar;
    result.c = v.c * scalar;
    return result;
}

double Vector3_DotProduct(Vector3 v1, Vector3 v2) {
    return v1.a * v2.a + v1.b * v2.b + v1.c * v2.c;
}

Vector3 Vector3_CrossProduct(Vector3 v1, Vector3 v2) {
    Vector3 result;
    result.a = v1.b * v2.c - v1.c * v2.b;
    result.b = v1.c * v2.a - v1.a * v2.c;
    result.c = v1.a * v2.b - v1.b * v2.a;
    return result;
}

double Vector3_Norm(Vector3 v) {
    return sqrt(v.a * v.a + v.b * v.b + v.c * v.c);
}

Vector3 Vector3_Normalize(Vector3 v) {
    double norm = Vector3_Norm(v);
    
    /* Handle zero vector case to avoid division by zero */
    if (norm < EPSILON) {
        return Vector3_Create(0.0, 0.0, 0.0);
    }
    
    return Vector3_ScalarDivide(v, norm);
}

Vector3 Vector3_ScalarDivide(Vector3 v, double scalar) {
    /* Check for division by zero */
    if (fabs(scalar) < EPSILON) {
        /* Return zero vector if division by zero would occur */
        return Vector3_Create(0.0, 0.0, 0.0);
    }
    
    Vector3 result;
    result.a = v.a / scalar;
    result.b = v.b / scalar;
    result.c = v.c / scalar;
    return result;
}

void Vector3_Print(Vector3 v, const char* name) {
    if (name) {
        printf("%s = [%.6f, %.6f, %.6f]\n", name, v.a, v.b, v.c);
    } else {
        printf("Vector3 = [%.6f, %.6f, %.6f]\n", v.a, v.b, v.c);
    }
}
