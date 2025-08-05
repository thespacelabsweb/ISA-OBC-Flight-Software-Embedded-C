/******************************************************************************
 * ISA Flight Software
 * 
 * File: types.h
 * Description: Common data type definitions for the guidance system
 *****************************************************************************/

#ifndef TYPES_H
#define TYPES_H

#include <stdint.h>
#include <float.h>
#include <math.h>

/**
 * @brief Constant values used throughout the system
 */
#define PI                  3.14159265358979323846
#define DEG_TO_RAD          (PI / 180.0)
#define RAD_TO_DEG          (180.0 / PI)
#define EPSILON             2.2204e-16  /* Same as MATLAB's eps constant */

/**
 * @brief Boolean type definition
 * Note: Using BOOL_FALSE/BOOL_TRUE to avoid conflicts with Windows.h
 */
typedef enum {
    BOOL_FALSE = 0,
    BOOL_TRUE = 1
} Bool;

/**
 * @brief Status codes for function returns
 */
typedef enum {
    STATUS_OK = 0,
    STATUS_ERROR,
    STATUS_INVALID_PARAM,
    STATUS_MATH_ERROR,
    STATUS_OUT_OF_RANGE
} Status;

/**
 * @brief Reference frames used in the system
 */
typedef enum {
    FRAME_ECI = 0,  /* Earth-Centered Inertial */
    FRAME_ECEF,     /* Earth-Centered Earth-Fixed */
    FRAME_LOCAL,    /* Local navigation frame */
    FRAME_BODY      /* Body-fixed frame */
} ReferenceFrame;

/**
 * @brief WGS84 ellipsoid parameters
 */
typedef struct {
    double semiMajorAxis;   /* Semi-major axis (meters) */
    double flattening;      /* Flattening */
    double eccentricity2;   /* Square of eccentricity */
} WGS84Parameters;

/**
 * @brief Geodetic position (latitude, longitude, altitude)
 */
typedef struct {
    double latitude;    /* Latitude in degrees */
    double longitude;   /* Longitude in degrees */
    double altitude;    /* Altitude in meters */
} GeodeticPosition;

/**
 * @brief Guidance parameters
 */
typedef struct {
    double a;           /* Impact angle control parameter a > 0 */
    double b;           /* Impact angle control parameter b > 0 */
    double m;           /* Impact angle control parameter 0 < m < 1 */
    double n;           /* Impact angle control parameter n > 1 */
    double d;           /* Impact angle control parameter d > 0 */
    double K;           /* Impact angle control parameter K > 0 */
    double sigmaMax;    /* Maximum look angle in radians */
    double N;           /* Guidance gain */
} GuidanceParameters;

/**
 * @brief Physical parameters of the projectile
 */
typedef struct {
    double mass;        /* Mass in kg */
    double diameter;    /* Diameter in meters */
    double dragCoeff;   /* Drag coefficient */
    double refArea;     /* Reference area in m^2 */
} ProjectileParameters;

/**
 * @brief Environmental parameters
 */
typedef struct {
    double gravity;     /* Gravity acceleration in m/s^2 */
    double airDensity;  /* Air density in kg/m^3 */
} EnvironmentParameters;

/**
 * @brief Simulation parameters
 */
typedef struct {
    double timeStep;                /* Time step in seconds */
    double maxSimulationTime;       /* Maximum simulation time in seconds */
    double minProjectileVelocity;   /* Minimum velocity threshold in m/s */
    double terminalPhaseDistance;   /* Distance to switch to terminal phase in meters */
} SimulationParameters;

#endif /* TYPES_H */
