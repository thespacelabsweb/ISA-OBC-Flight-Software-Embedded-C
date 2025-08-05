/******************************************************************************
 * ISA Flight Software
 * 
 * File: quaternion.c
 * Description: Implementation of quaternion operations
 *****************************************************************************/

#include <stdio.h>
#include <math.h>
#include "../../include/math/quaternion.h"

Quaternion Quaternion_Create(double w, double x, double y, double z) {
    Quaternion result;
    result.w = w;
    result.x = x;
    result.y = y;
    result.z = z;
    return result;
}

Quaternion Quaternion_Identity(void) {
    return Quaternion_Create(1.0, 0.0, 0.0, 0.0);
}

Quaternion Quaternion_FromAxisAngle(Vector3 axis, double angle) {
    Quaternion result;
    double halfAngle = angle * 0.5;
    double sinHalfAngle = sin(halfAngle);
    
    /* Ensure axis is normalized */
    double axisMagnitude = Vector3_Norm(axis);
    if (axisMagnitude < EPSILON) {
        /* If axis is zero vector, return identity quaternion */
        return Quaternion_Identity();
    }
    
    if (fabs(axisMagnitude - 1.0) > EPSILON) {
        /* Normalize axis if it's not already normalized */
        axis = Vector3_ScalarDivide(axis, axisMagnitude);
    }
    
    result.w = cos(halfAngle);
    result.x = axis.a * sinHalfAngle;
    result.y = axis.b * sinHalfAngle;
    result.z = axis.c * sinHalfAngle;
    
    return result;
}

Quaternion Quaternion_FromEulerAngles(double roll, double pitch, double yaw) {
    /* Calculate half angles */
    double halfRoll = roll * 0.5;
    double halfPitch = pitch * 0.5;
    double halfYaw = yaw * 0.5;
    
    /* Calculate sines and cosines */
    double cr = cos(halfRoll);
    double sr = sin(halfRoll);
    double cp = cos(halfPitch);
    double sp = sin(halfPitch);
    double cy = cos(halfYaw);
    double sy = sin(halfYaw);
    
    /* Calculate quaternion components using the ZYX convention */
    Quaternion result;
    result.w = cr * cp * cy + sr * sp * sy;
    result.x = sr * cp * cy - cr * sp * sy;
    result.y = cr * sp * cy + sr * cp * sy;
    result.z = cr * cp * sy - sr * sp * cy;
    
    return result;
}

Quaternion Quaternion_Add(Quaternion q1, Quaternion q2) {
    Quaternion result;
    result.w = q1.w + q2.w;
    result.x = q1.x + q2.x;
    result.y = q1.y + q2.y;
    result.z = q1.z + q2.z;
    return result;
}

Quaternion Quaternion_Multiply(Quaternion q1, Quaternion q2) {
    Quaternion result;
    
    /* Hamilton product formula */
    result.w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z;
    result.x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y;
    result.y = q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x;
    result.z = q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w;
    
    return result;
}

Quaternion Quaternion_Conjugate(Quaternion q) {
    Quaternion result;
    result.w = q.w;
    result.x = -q.x;
    result.y = -q.y;
    result.z = -q.z;
    return result;
}

double Quaternion_Norm(Quaternion q) {
    return sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
}

Quaternion Quaternion_Normalize(Quaternion q) {
    double norm = Quaternion_Norm(q);
    
    /* Handle zero quaternion case to avoid division by zero */
    if (norm < EPSILON) {
        return Quaternion_Identity();
    }
    
    Quaternion result;
    result.w = q.w / norm;
    result.x = q.x / norm;
    result.y = q.y / norm;
    result.z = q.z / norm;
    
    return result;
}

Quaternion Quaternion_Inverse(Quaternion q) {
    double normSquared = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;
    
    /* Handle zero quaternion case to avoid division by zero */
    if (normSquared < EPSILON) {
        return Quaternion_Identity();
    }
    
    double invNormSquared = 1.0 / normSquared;
    
    Quaternion result;
    result.w = q.w * invNormSquared;
    result.x = -q.x * invNormSquared;
    result.y = -q.y * invNormSquared;
    result.z = -q.z * invNormSquared;
    
    return result;
}

Matrix3 Quaternion_ToRotationMatrix(Quaternion q) {
    /* Ensure quaternion is normalized */
    double norm = Quaternion_Norm(q);
    if (fabs(norm - 1.0) > EPSILON) {
        q = Quaternion_Normalize(q);
    }
    
    double w2 = q.w * q.w;
    double x2 = q.x * q.x;
    double y2 = q.y * q.y;
    double z2 = q.z * q.z;
    
    double xy = q.x * q.y;
    double xz = q.x * q.z;
    double yz = q.y * q.z;
    double wx = q.w * q.x;
    double wy = q.w * q.y;
    double wz = q.w * q.z;
    
    /* Create rotation matrix - this matches the MATLAB implementation */
    return Matrix3_Create(
        w2 + x2 - y2 - z2, 2.0 * (xy - wz), 2.0 * (xz + wy),
        2.0 * (xy + wz), w2 - x2 + y2 - z2, 2.0 * (yz - wx),
        2.0 * (xz - wy), 2.0 * (yz + wx), w2 - x2 - y2 + z2
    );
}

Vector3 Quaternion_RotateVector(Quaternion q, Vector3 v) {
    /* Create a quaternion with the vector as the imaginary part */
    Quaternion vq = Quaternion_Create(0.0, v.a, v.b, v.c);
    
    /* Ensure q is normalized */
    q = Quaternion_Normalize(q);
    
    /* Calculate q * vq * q^-1 (since q is normalized, q^-1 = q*) */
    Quaternion qInv = Quaternion_Conjugate(q);
    Quaternion temp = Quaternion_Multiply(q, vq);
    Quaternion result = Quaternion_Multiply(temp, qInv);
    
    /* Extract the vector part */
    return Vector3_Create(result.x, result.y, result.z);
}

void Quaternion_Print(Quaternion q, const char* name) {
    if (name) {
        printf("%s = [%.6f, %.6f, %.6f, %.6f]\n", name, q.w, q.x, q.y, q.z);
    } else {
        printf("Quaternion = [%.6f, %.6f, %.6f, %.6f]\n", q.w, q.x, q.y, q.z);
    }
}
