/******************************************************************************
 * ISA Flight Software
 * 
 * File: coordinate_transforms.c
 * Description: Implementation of coordinate transformation functions
 *****************************************************************************/
#include <stddef.h> 
#include <math.h>
#include "../../include/math/coordinate_transforms.h"
#include "../../include/common/config.h"

/* Helper function to convert degrees to radians */
static double DegToRad(double degrees) {
    return degrees * DEG_TO_RAD;
}

/* Helper function to convert radians to degrees */
static double RadToDeg(double radians) {
    return radians * RAD_TO_DEG;
}

Vector3 GeodeticToECI(GeodeticPosition position, const WGS84Parameters* wgs84) {
    /* Use default WGS84 parameters if none provided */
    WGS84Parameters params = (wgs84 != NULL) ? *wgs84 : WGS84_DEFAULT;
    
    /* Convert lat, lon to radians */
    double lat = DegToRad(position.latitude);
    double lon = DegToRad(position.longitude);
    double alt = position.altitude;
    
    /* Radius of curvature in the prime vertical */
    double N = params.semiMajorAxis / sqrt(1.0 - params.eccentricity2 * sin(lat) * sin(lat));
    
    /* Compute ECI coordinates (X, Y, Z) */
    double X = (N + alt) * cos(lat) * cos(lon);
    double Y = (N + alt) * cos(lat) * sin(lon);
    double Z = ((1.0 - params.eccentricity2) * N + alt) * sin(lat);
    
    return Vector3_Create(X, Y, Z);
}

GeodeticPosition ECIToGeodetic(Vector3 positionECI, const WGS84Parameters* wgs84) {
    /* Use default WGS84 parameters if none provided */
    WGS84Parameters params = (wgs84 != NULL) ? *wgs84 : WGS84_DEFAULT;
    
    /* ECI coordinates */
    double X = positionECI.a;
    double Y = positionECI.b;
    double Z = positionECI.c;
    
    /* Calculate longitude */
    double lon = atan2(Y, X);
    
    /* Initial guess for latitude */
    double r = sqrt(X*X + Y*Y);
    double lat = atan2(Z, r * (1.0 - params.eccentricity2));
    
    /* Iteratively solve for latitude */
    double N = params.semiMajorAxis / sqrt(1.0 - params.eccentricity2 * sin(lat)*sin(lat));
    double alt = r / cos(lat) - N;
    
    /* Iterate to refine latitude and altitude (same as MATLAB implementation) */
    int i;
    for (i = 0; i < 5; i++) {
        N = params.semiMajorAxis / sqrt(1.0 - params.eccentricity2 * sin(lat)*sin(lat));
        alt = r / cos(lat) - N;
        lat = atan2(Z + params.eccentricity2 * N * sin(lat), r);
    }
    
    /* Convert to degrees */
    GeodeticPosition result;
    result.latitude = RadToDeg(lat);
    result.longitude = RadToDeg(lon);
    result.altitude = alt;
    
    return result;
}

Matrix3 CalculateLocalToECIMatrix(GeodeticPosition originPosition, GeodeticPosition targetPosition) {
    /* Convert geodetic coordinates to ECEF */
    Vector3 originECEF = GeodeticToECI(originPosition, NULL);
    Vector3 targetECEF = GeodeticToECI(targetPosition, NULL);
    
    /* Create perturbed point for Z-axis (normal to ellipsoid) */
    GeodeticPosition perturbedPosition = originPosition;
    perturbedPosition.altitude += 1.0;  /* 1 meter higher */
    Vector3 perturbedECEF = GeodeticToECI(perturbedPosition, NULL);
    
    /* Z-axis: Points radially outward from Earth's surface */
    Vector3 zAxisECEF = Vector3_Subtract(perturbedECEF, originECEF);
    zAxisECEF = Vector3_Normalize(zAxisECEF);
    
    /* X-axis: Points from origin to target */
    Vector3 xAxisECEF = Vector3_Subtract(targetECEF, originECEF);
    xAxisECEF = Vector3_Normalize(xAxisECEF);
    
    /* Y-axis: Completes the right-handed system */
    Vector3 yAxisECEF = Vector3_CrossProduct(zAxisECEF, xAxisECEF);
    yAxisECEF = Vector3_Normalize(yAxisECEF);
    
    /* Construct rotation matrix from local to ECI */
    return Matrix3_FromColumns(xAxisECEF, yAxisECEF, zAxisECEF);
}

Matrix3 CalculateECIToLocalMatrix(GeodeticPosition originPosition, GeodeticPosition targetPosition) {
    /* Get local to ECI matrix and transpose it */
    Matrix3 localToECI = CalculateLocalToECIMatrix(originPosition, targetPosition);
    return Matrix3_Transpose(localToECI);
}

Vector3 ECIToLocal(Vector3 positionECI, GeodeticPosition originPosition, GeodeticPosition targetPosition) {
    /* Convert origin position to ECI */
    Vector3 originECI = GeodeticToECI(originPosition, NULL);
    
    /* Get rotation matrix from ECI to local */
    Matrix3 eciToLocal = CalculateECIToLocalMatrix(originPosition, targetPosition);
    
    /* Transform position: first subtract origin, then rotate */
    Vector3 relativeECI = Vector3_Subtract(positionECI, originECI);
    return Matrix3_MultiplyVector(eciToLocal, relativeECI);
}

Vector3 ECIToLocalVelocity(Vector3 velocityECI, GeodeticPosition originPosition, GeodeticPosition targetPosition) {
    /* For velocity, we only need to rotate (no translation) */
    Matrix3 eciToLocal = CalculateECIToLocalMatrix(originPosition, targetPosition);
    return Matrix3_MultiplyVector(eciToLocal, velocityECI);
}

Vector3 LocalToECI(Vector3 positionLocal, GeodeticPosition originPosition, GeodeticPosition targetPosition) {
    /* Convert origin position to ECI */
    Vector3 originECI = GeodeticToECI(originPosition, NULL);
    
    /* Get rotation matrix from local to ECI */
    Matrix3 localToECI = CalculateLocalToECIMatrix(originPosition, targetPosition);
    
    /* Transform position: first rotate, then add origin */
    Vector3 relativeECI = Matrix3_MultiplyVector(localToECI, positionLocal);
    return Vector3_Add(relativeECI, originECI);
}

Vector3 LocalToECIVelocity(Vector3 velocityLocal, GeodeticPosition originPosition, GeodeticPosition targetPosition) {
    /* For velocity, we only need to rotate (no translation) */
    Matrix3 localToECI = CalculateLocalToECIMatrix(originPosition, targetPosition);
    return Matrix3_MultiplyVector(localToECI, velocityLocal);
}

Matrix3 CalculateBodyToLocalMatrix(double theta, double psi, double phi) {
    /* Create rotation matrix from body to local frame using Euler angles */
    /* This matches the MATLAB implementation in b2l.m */
    
    return Matrix3_Create(
        cos(psi) * cos(theta), 
        (cos(phi) * sin(psi) * cos(theta)) + (sin(phi) * sin(theta)), 
        (sin(phi) * sin(psi) * cos(theta)) - (cos(phi) * sin(theta)),
        
        -sin(theta), 
        cos(phi) * cos(psi), 
        sin(phi) * cos(psi),
        
        cos(psi) * sin(theta), 
        (cos(phi) * sin(psi) * sin(theta)) - (sin(phi) * cos(theta)), 
        (sin(phi) * sin(psi) * sin(theta)) + (cos(phi) * cos(theta))
    );
}

Matrix3 CalculateLocalToBodyMatrix(double theta, double psi, double phi) {
    /* Get body to local matrix and transpose it */
    Matrix3 bodyToLocal = CalculateBodyToLocalMatrix(theta, psi, phi);
    return Matrix3_Transpose(bodyToLocal);
}

Vector3 BodyToLocal(Vector3 vectorBody, double theta, double psi, double phi) {
    Matrix3 bodyToLocal = CalculateBodyToLocalMatrix(theta, psi, phi);
    return Matrix3_MultiplyVector(bodyToLocal, vectorBody);
}

Vector3 LocalToBody(Vector3 vectorLocal, double theta, double psi, double phi) {
    Matrix3 localToBody = CalculateLocalToBodyMatrix(theta, psi, phi);
    return Matrix3_MultiplyVector(localToBody, vectorLocal);
}

double SignedPower(double m, double x) {
    /* Implements the sig() function from MATLAB */
    /* sign(x) * |x|^m */
    return (x >= 0.0 ? 1.0 : -1.0) * pow(fabs(x), m);
}
