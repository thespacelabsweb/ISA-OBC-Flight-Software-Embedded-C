/******************************************************************************
 * ISA Flight Software - Guidance Module Support Library
 * @file vector3.h
 * @brief Float-based 3D vector operations for embedded systems
 * @details Provides basic vector math operations needed for guidance algorithms
 * @author Flight Software Team, Spacelabs
 * @date 2025
 * @version 1.0
 *
 * MISRA C: Compliant Implementation (float instead of double for embedded efficiency)
 ******************************************************************************/

#ifndef VECTOR3_H
#define VECTOR3_H

#include <stdint.h>
#include <stdbool.h>

/* Flight software error codes for vector operations */
typedef enum {
    VECTOR3_SUCCESS = 0U,
    VECTOR3_ERROR_INVALID_PARAM = 1U,
    VECTOR3_ERROR_DIVISION_BY_ZERO = 2U
} Vector3Error_t;

/* 3D Vector structure using float for embedded efficiency */
typedef struct {
    float x;  /* X component */
    float y;  /* Y component */
    float z;  /* Z component */
} Vector3f_t;

/* Function prototypes - MISRA C compliant with parameter validation */

/******************************************************************************
 * @brief Create a new 3D vector
 * @param x X component
 * @param y Y component
 * @param z Z component
 * @return Vector3f_t New vector
 ******************************************************************************/
Vector3f_t Vector3f_Create(float x, float y, float z);

/******************************************************************************
 * @brief Add two vectors: result = a + b
 * @param a First vector
 * @param b Second vector
 * @return Vector3f_t Result vector
 ******************************************************************************/
Vector3f_t Vector3f_Add(const Vector3f_t a, const Vector3f_t b);

/******************************************************************************
 * @brief Subtract two vectors: result = a - b
 * @param a First vector (minuend)
 * @param b Second vector (subtrahend)
 * @return Vector3f_t Result vector
 ******************************************************************************/
Vector3f_t Vector3f_Subtract(const Vector3f_t a, const Vector3f_t b);

/******************************************************************************
 * @brief Scale vector by scalar: result = scalar * v
 * @param scalar Scale factor
 * @param v Input vector
 * @return Vector3f_t Scaled vector
 ******************************************************************************/
Vector3f_t Vector3f_Scale(float scalar, const Vector3f_t v);

/******************************************************************************
 * @brief Divide vector by scalar: result = v / scalar
 * @param v Input vector
 * @param scalar Divisor (must not be zero)
 * @return Vector3f_t Result vector, or zero vector on division by zero
 ******************************************************************************/
Vector3f_t Vector3f_Divide(const Vector3f_t v, float scalar);

/******************************************************************************
 * @brief Compute dot product of two vectors: result = a · b
 * @param a First vector
 * @param b Second vector
 * @return float Dot product result
 ******************************************************************************/
float Vector3f_Dot(const Vector3f_t a, const Vector3f_t b);

/******************************************************************************
 * @brief Compute cross product of two vectors: result = a × b
 * @param a First vector
 * @param b Second vector
 * @return Vector3f_t Cross product result
 ******************************************************************************/
Vector3f_t Vector3f_Cross(const Vector3f_t a, const Vector3f_t b);

/******************************************************************************
 * @brief Compute Euclidean norm (magnitude) of vector: result = ||v||
 * @param v Input vector
 * @return float Vector magnitude (always >= 0)
 ******************************************************************************/
float Vector3f_Norm(const Vector3f_t v);

/******************************************************************************
 * @brief Normalize vector to unit length: result = v / ||v||
 * @param v Input vector
 * @return Vector3f_t Unit vector, or zero vector if input is zero vector
 ******************************************************************************/
Vector3f_t Vector3f_Normalize(const Vector3f_t v);

/******************************************************************************
 * @brief Check if vector is zero (all components are zero within epsilon)
 * @param v Input vector
 * @return true if vector is zero, false otherwise
 ******************************************************************************/
bool Vector3f_IsZero(const Vector3f_t v);

/******************************************************************************
 * @brief Print vector components (for debugging)
 * @param v Vector to print
 * @param label Optional label for output
 ******************************************************************************/
void Vector3f_Print(const Vector3f_t v, const char* label);

#endif /* VECTOR3_H */

