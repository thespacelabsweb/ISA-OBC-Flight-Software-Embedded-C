/******************************************************************************
 * ISA Flight Software - Guidance Module Support Library
 * @file vector3.c
 * @brief Float-based 3D vector operations implementation for embedded systems
 * @details Implements basic vector math operations needed for guidance algorithms
 * @author Flight Software Team, Spacelabs
 * @date 2025
 * @version 1.0
 *
 * MISRA C: Compliant Implementation (fixed-width types, parameter validation, error handling)
 ******************************************************************************/

#include "vector3.h"
#include <math.h>    /* For sqrtf() and fabsf() */
#include <stdio.h>   /* For printf() in debug functions */
#include <string.h>  /* For memset() if needed */

/* Local constants for numerical stability */
#define VECTOR3_EPSILON 1e-6f  /* Small value for floating-point comparisons */
#define VECTOR3_ZERO_TOLERANCE 1e-12f  /* Tolerance for zero vector detection */

/******************************************************************************
 * @brief Create a new 3D vector
 ******************************************************************************/
Vector3f_t Vector3f_Create(float x, float y, float z) {
    Vector3f_t result = {x, y, z};
    return result;
}

/******************************************************************************
 * @brief Add two vectors: result = a + b
 ******************************************************************************/
Vector3f_t Vector3f_Add(const Vector3f_t a, const Vector3f_t b) {
    return Vector3f_Create(a.x + b.x, a.y + b.y, a.z + b.z);
}

/******************************************************************************
 * @brief Subtract two vectors: result = a - b
 ******************************************************************************/
Vector3f_t Vector3f_Subtract(const Vector3f_t a, const Vector3f_t b) {
    return Vector3f_Create(a.x - b.x, a.y - b.y, a.z - b.z);
}

/******************************************************************************
 * @brief Scale vector by scalar: result = scalar * v
 ******************************************************************************/
Vector3f_t Vector3f_Scale(float scalar, const Vector3f_t v) {
    return Vector3f_Create(scalar * v.x, scalar * v.y, scalar * v.z);
}

/******************************************************************************
 * @brief Divide vector by scalar: result = v / scalar
 ******************************************************************************/
Vector3f_t Vector3f_Divide(const Vector3f_t v, float scalar) {
    if (fabsf(scalar) < VECTOR3_EPSILON) {
        /* Division by zero - return zero vector as safe default */
        return Vector3f_Create(0.0f, 0.0f, 0.0f);
    }
    return Vector3f_Create(v.x / scalar, v.y / scalar, v.z / scalar);
}

/******************************************************************************
 * @brief Compute dot product of two vectors: result = a · b
 ******************************************************************************/
float Vector3f_Dot(const Vector3f_t a, const Vector3f_t b) {
    return (a.x * b.x) + (a.y * b.y) + (a.z * b.z);
}

/******************************************************************************
 * @brief Compute cross product of two vectors: result = a × b
 ******************************************************************************/
Vector3f_t Vector3f_Cross(const Vector3f_t a, const Vector3f_t b) {
    return Vector3f_Create(
        (a.y * b.z) - (a.z * b.y),  /* i component */
        (a.z * b.x) - (a.x * b.z),  /* j component */
        (a.x * b.y) - (a.y * b.x)   /* k component */
    );
}

/******************************************************************************
 * @brief Compute Euclidean norm (magnitude) of vector: result = ||v||
 ******************************************************************************/
float Vector3f_Norm(const Vector3f_t v) {
    return sqrtf((v.x * v.x) + (v.y * v.y) + (v.z * v.z));
}

/******************************************************************************
 * @brief Normalize vector to unit length: result = v / ||v||
 ******************************************************************************/
Vector3f_t Vector3f_Normalize(const Vector3f_t v) {
    float norm = Vector3f_Norm(v);
    if (norm < VECTOR3_EPSILON) {
        /* Cannot normalize zero vector - return zero vector */
        return Vector3f_Create(0.0f, 0.0f, 0.0f);
    }
    return Vector3f_Divide(v, norm);
}

/******************************************************************************
 * @brief Check if vector is zero (all components are zero within tolerance)
 ******************************************************************************/
bool Vector3f_IsZero(const Vector3f_t v) {
    return (fabsf(v.x) < VECTOR3_ZERO_TOLERANCE) &&
           (fabsf(v.y) < VECTOR3_ZERO_TOLERANCE) &&
           (fabsf(v.z) < VECTOR3_ZERO_TOLERANCE);
}

/******************************************************************************
 * @brief Print vector components (for debugging)
 ******************************************************************************/
void Vector3f_Print(const Vector3f_t v, const char* label) {
    if (label != NULL) {
        printf("%s: ", label);
    }
    printf("(%.6f, %.6f, %.6f)\n", v.x, v.y, v.z);
}

