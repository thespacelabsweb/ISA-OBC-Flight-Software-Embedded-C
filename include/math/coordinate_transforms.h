/******************************************************************************
 * ISA Flight Software
 * 
 * File: coordinate_transforms.h
 * Description: Coordinate transformation functions for the guidance system
 *****************************************************************************/

#ifndef COORDINATE_TRANSFORMS_H
#define COORDINATE_TRANSFORMS_H

#include "../common/types.h"
#include "vector3.h"
#include "matrix3.h"

/**
 * @brief Convert geodetic coordinates to Earth-Centered Inertial (ECI) coordinates
 * 
 * @param position Geodetic position (latitude, longitude, altitude)
 * @param wgs84 WGS84 parameters (if NULL, default parameters are used)
 * @return Vector3 Position vector in ECI frame
 */
Vector3 GeodeticToECI(GeodeticPosition position, const WGS84Parameters* wgs84);

/**
 * @brief Convert Earth-Centered Inertial (ECI) coordinates to geodetic coordinates
 * 
 * @param positionECI Position vector in ECI frame
 * @param wgs84 WGS84 parameters (if NULL, default parameters are used)
 * @return GeodeticPosition Geodetic position (latitude, longitude, altitude)
 */
GeodeticPosition ECIToGeodetic(Vector3 positionECI, const WGS84Parameters* wgs84);

/**
 * @brief Convert Earth-Centered Inertial (ECI) coordinates to local navigation frame
 * 
 * @param positionECI Position vector in ECI frame
 * @param originPosition Geodetic position of the local frame origin
 * @param targetPosition Geodetic position of the target
 * @return Vector3 Position vector in local frame
 */
Vector3 ECIToLocal(Vector3 positionECI, GeodeticPosition originPosition, GeodeticPosition targetPosition);

/**
 * @brief Convert velocity from Earth-Centered Inertial (ECI) frame to local navigation frame
 * 
 * @param velocityECI Velocity vector in ECI frame
 * @param originPosition Geodetic position of the local frame origin
 * @param targetPosition Geodetic position of the target
 * @return Vector3 Velocity vector in local frame
 */
Vector3 ECIToLocalVelocity(Vector3 velocityECI, GeodeticPosition originPosition, GeodeticPosition targetPosition);

/**
 * @brief Convert local navigation frame coordinates to Earth-Centered Inertial (ECI) coordinates
 * 
 * @param positionLocal Position vector in local frame
 * @param originPosition Geodetic position of the local frame origin
 * @param targetPosition Geodetic position of the target
 * @return Vector3 Position vector in ECI frame
 */
Vector3 LocalToECI(Vector3 positionLocal, GeodeticPosition originPosition, GeodeticPosition targetPosition);

/**
 * @brief Convert velocity from local navigation frame to Earth-Centered Inertial (ECI) frame
 * 
 * @param velocityLocal Velocity vector in local frame
 * @param originPosition Geodetic position of the local frame origin
 * @param targetPosition Geodetic position of the target
 * @return Vector3 Velocity vector in ECI frame
 */
Vector3 LocalToECIVelocity(Vector3 velocityLocal, GeodeticPosition originPosition, GeodeticPosition targetPosition);

/**
 * @brief Convert body frame coordinates to local navigation frame coordinates
 * 
 * @param vectorBody Vector in body frame
 * @param theta Pitch angle in radians
 * @param psi Yaw angle in radians
 * @param phi Roll angle in radians
 * @return Vector3 Vector in local frame
 */
Vector3 BodyToLocal(Vector3 vectorBody, double theta, double psi, double phi);

/**
 * @brief Convert local navigation frame coordinates to body frame coordinates
 * 
 * @param vectorLocal Vector in local frame
 * @param theta Pitch angle in radians
 * @param psi Yaw angle in radians
 * @param phi Roll angle in radians
 * @return Vector3 Vector in body frame
 */
Vector3 LocalToBody(Vector3 vectorLocal, double theta, double psi, double phi);

/**
 * @brief Calculate the rotation matrix from local to ECI frame
 * 
 * @param originPosition Geodetic position of the local frame origin
 * @param targetPosition Geodetic position of the target
 * @return Matrix3 Rotation matrix from local to ECI frame
 */
Matrix3 CalculateLocalToECIMatrix(GeodeticPosition originPosition, GeodeticPosition targetPosition);

/**
 * @brief Calculate the rotation matrix from ECI to local frame
 * 
 * @param originPosition Geodetic position of the local frame origin
 * @param targetPosition Geodetic position of the target
 * @return Matrix3 Rotation matrix from ECI to local frame
 */
Matrix3 CalculateECIToLocalMatrix(GeodeticPosition originPosition, GeodeticPosition targetPosition);

/**
 * @brief Calculate the rotation matrix from body to local frame
 * 
 * @param theta Pitch angle in radians
 * @param psi Yaw angle in radians
 * @param phi Roll angle in radians
 * @return Matrix3 Rotation matrix from body to local frame
 */
Matrix3 CalculateBodyToLocalMatrix(double theta, double psi, double phi);

/**
 * @brief Calculate the rotation matrix from local to body frame
 * 
 * @param theta Pitch angle in radians
 * @param psi Yaw angle in radians
 * @param phi Roll angle in radians
 * @return Matrix3 Rotation matrix from local to body frame
 */
Matrix3 CalculateLocalToBodyMatrix(double theta, double psi, double phi);

/**
 * @brief Signed power function used in guidance calculations
 * 
 * @param m Power exponent
 * @param x Base value
 * @return double sign(x) * |x|^m
 */
double SignedPower(double m, double x);

#endif /* COORDINATE_TRANSFORMS_H */
