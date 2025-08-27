/******************************************************************************
 * ISA Flight Software
 * 
 * File: quaternion.h
 * Description: Quaternion operations for attitude representation
 *****************************************************************************/

#ifndef QUATERNION_H
#define QUATERNION_H

#include "../common/types.h"
#include "vector3.h"
#include "matrix3.h"

/**
 * @brief Quaternion structure
 * 
 * Quaternion is represented as q = w + x*i + y*j + z*k
 * where w is the scalar part and (x,y,z) is the vector part
 */
typedef struct {
    double w;  /* Scalar part */
    double x;  /* Vector part, i component */
    double y;  /* Vector part, j component */
    double z;  /* Vector part, k component */
} Quaternion;

/**
 * @brief Initialize a Quaternion with given components
 * 
 * @param w The scalar part
 * @param x The x component of the vector part
 * @param y The y component of the vector part
 * @param z The z component of the vector part
 * @return Quaternion The initialized quaternion
 */
Quaternion Quaternion_Create(double w, double x, double y, double z);

/**
 * @brief Create an identity quaternion (no rotation)
 * 
 * @return Quaternion The identity quaternion (1,0,0,0)
 */
Quaternion Quaternion_Identity(void);

/**
 * @brief Create a quaternion from an axis-angle representation
 * 
 * @param axis The rotation axis (should be normalized)
 * @param angle The rotation angle in radians
 * @return Quaternion The resulting quaternion
 */
Quaternion Quaternion_FromAxisAngle(Vector3 axis, double angle);

/**
 * @brief Create a quaternion from Euler angles (roll, pitch, yaw)
 * 
 * @param roll Rotation around x-axis in radians
 * @param pitch Rotation around y-axis in radians
 * @param yaw Rotation around z-axis in radians
 * @return Quaternion The resulting quaternion
 */
Quaternion Quaternion_FromEulerAngles(double roll, double pitch, double yaw);

/**
 * @brief Add two quaternions
 * 
 * @param q1 First quaternion
 * @param q2 Second quaternion
 * @return Quaternion Result of q1 + q2
 */
Quaternion Quaternion_Add(Quaternion q1, Quaternion q2);

/**
 * @brief Multiply two quaternions (quaternion composition)
 * 
 * @param q1 First quaternion
 * @param q2 Second quaternion
 * @return Quaternion Result of q1 * q2
 */
Quaternion Quaternion_Multiply(Quaternion q1, Quaternion q2);

/**
 * @brief Calculate the conjugate of a quaternion
 * 
 * @param q The quaternion
 * @return Quaternion The conjugate of q
 */
Quaternion Quaternion_Conjugate(Quaternion q);

/**
 * @brief Calculate the magnitude (norm) of a quaternion
 * 
 * @param q The quaternion
 * @return double The magnitude ||q||
 */
double Quaternion_Norm(Quaternion q);

/**
 * @brief Normalize a quaternion (make it unit length)
 * 
 * @param q The quaternion to normalize
 * @return Quaternion The normalized quaternion q/||q||
 */
Quaternion Quaternion_Normalize(Quaternion q);

/**
 * @brief Calculate the inverse of a quaternion
 * 
 * @param q The quaternion
 * @return Quaternion The inverse of q
 */
Quaternion Quaternion_Inverse(Quaternion q);

/**
 * @brief Convert a quaternion to a rotation matrix
 * 
 * @param q The quaternion (should be normalized)
 * @return Matrix3 The corresponding rotation matrix
 */
Matrix3 Quaternion_ToRotationMatrix(Quaternion q);

/**
 * @brief Rotate a vector using a quaternion
 * 
 * @param q The quaternion (should be normalized)
 * @param v The vector to rotate
 * @return Vector3 The rotated vector
 */
Vector3 Quaternion_RotateVector(Quaternion q, Vector3 v);

/**
 * @brief Print a quaternion to stdout (for debugging)
 * 
 * @param q The quaternion to print
 * @param name Optional name to display
 */
void Quaternion_Print(Quaternion q, const char* name);

#endif /* QUATERNION_H */
