/******************************************************************************
 * ISA Flight Software - Guidance Module Support Library
 * @file coordinate_transforms.c
 * @brief Coordinate transformations implementation for guidance algorithms
 * @details Implements geodetic/ECI conversions and local frame transformations needed for guidance
 * @author Flight Software Team, Spacelabs
 * @date 2025
 * @version 1.0
 *
 * MISRA C: Compliant Implementation (fixed-width types, parameter validation, error handling)
 *
 * Coordinate Frame Definitions:
 * - ECI (Earth-Centered Inertial): Fixed inertial reference frame
 * - Local Frame: Origin at launch point, X-axis to target, Z-axis local vertical
 ******************************************************************************/

#include "coordinate_transforms.h"
#include <math.h>    /* For trigonometric functions, sqrtf() */
#include <string.h>  /* For memset() if needed */
#include <stdio.h>   /* For printf() in debug functions */

/* WGS84 Ellipsoid Constants */
#define WGS84_SEMI_MAJOR_AXIS 6378137.0f        /* Earth's semi-major axis (meters) */
#define WGS84_FLATTENING 0.0033528106647474805f /* Flattening factor */
#define WGS84_ECCENTRICITY_SQUARED 0.0066943799901413165f /* e² = 2f - f² */

/* Degree to radian conversion */
#define DEG_TO_RAD (3.141592653589793f / 180.0f)
#define RAD_TO_DEG (180.0f / 3.141592653589793f)

/* Numerical constants for stability */
#define COORD_EPSILON 1e-6f

/******************************************************************************
 * @brief Convert geodetic coordinates (lat, lon, alt) to ECI coordinates (X, Y, Z)
 * WGS84 ellipsoid model: Converts latitude/longitude/altitude to Earth-Centered Inertial coordinates
 ******************************************************************************/
CoordinateError_t GeodeticToECI(const GeodeticPosition_t geodetic, Vector3f_t* eci) {
    /* Parameter validation */
    if (eci == NULL) {
        return COORD_ERROR_INVALID_PARAM;
    }

    /* Convert degrees to radians */
    float lat_rad = geodetic.latitude * DEG_TO_RAD;
    float lon_rad = geodetic.longitude * DEG_TO_RAD;

    /* Pre-compute trigonometric functions */
    float sin_lat = sinf(lat_rad);
    float cos_lat = cosf(lat_rad);
    float sin_lon = sinf(lon_rad);
    float cos_lon = cosf(lon_rad);

    /* Radius of curvature in the prime vertical */
    float denom = sqrtf(1.0f - WGS84_ECCENTRICITY_SQUARED * sin_lat * sin_lat);
    if (denom < COORD_EPSILON) {
        return COORD_ERROR_CONVERSION_FAILED; /* Division by zero protection */
    }
    float N = WGS84_SEMI_MAJOR_AXIS / denom;

    /* Compute ECI coordinates using WGS84 formulas */
    float N_plus_alt = N + geodetic.altitude;
    float one_minus_e2 = 1.0f - WGS84_ECCENTRICITY_SQUARED;

    eci->x = N_plus_alt * cos_lat * cos_lon;
    eci->y = N_plus_alt * cos_lat * sin_lon;
    eci->z = (one_minus_e2 * N + geodetic.altitude) * sin_lat;

    return COORD_SUCCESS;
}

/******************************************************************************
 * @brief Convert ECI coordinates to geodetic coordinates (lat, lon, alt)
 * Iterative solution for inverse WGS84 conversion
 ******************************************************************************/
CoordinateError_t ECIToGeodetic(const Vector3f_t eci, GeodeticPosition_t* geodetic) {
    /* Parameter validation */
    if (geodetic == NULL) {
        return COORD_ERROR_INVALID_PARAM;
    }

    /* Iterative solution for latitude (Bowring's method) */
    float X = eci.x;
    float Y = eci.y;
    float Z = eci.z;

    float p = sqrtf(X*X + Y*Y);  /* Distance from Z-axis */

    /* Handle equatorial case */
    if (p < COORD_EPSILON) {
        geodetic->latitude = (Z >= 0.0f) ? 90.0f : -90.0f;
        geodetic->longitude = 0.0f;
        geodetic->altitude = fabsf(Z) - WGS84_SEMI_MAJOR_AXIS * (1.0f - WGS84_FLATTENING);
        return COORD_SUCCESS;
    }

    /* Initial approximation for latitude */
    float lat_rad = atan2f(Z, p * (1.0f - WGS84_ECCENTRICITY_SQUARED));

    /* Iterative refinement (usually converges in 2-3 iterations) */
    for (int i = 0; i < 5; i++) {
        float sin_lat = sinf(lat_rad);
        float N = WGS84_SEMI_MAJOR_AXIS / sqrtf(1.0f - WGS84_ECCENTRICITY_SQUARED * sin_lat * sin_lat);
        float alt_approx = p / cosf(lat_rad) - N;

        float lat_new = atan2f(Z + WGS84_ECCENTRICITY_SQUARED * N * sin_lat, p);

        if (fabsf(lat_new - lat_rad) < COORD_EPSILON) {
            lat_rad = lat_new;
            break;
        }
        lat_rad = lat_new;
    }

    /* Compute final altitude */
    float sin_lat = sinf(lat_rad);
    float N = WGS84_SEMI_MAJOR_AXIS / sqrtf(1.0f - WGS84_ECCENTRICITY_SQUARED * sin_lat * sin_lat);
    float altitude = p / cosf(lat_rad) - N;

    /* Convert back to degrees */
    geodetic->latitude = lat_rad * RAD_TO_DEG;
    geodetic->longitude = atan2f(Y, X) * RAD_TO_DEG;
    geodetic->altitude = altitude;

    return COORD_SUCCESS;
}

/******************************************************************************
 * @brief Convert ECI position vector to Local frame coordinates
 * Local frame: X-axis points from origin to target, Z-axis is local vertical
 ******************************************************************************/
CoordinateError_t ECIToLocal(const Vector3f_t eci,
                            const GeodeticPosition_t origin,
                            const GeodeticPosition_t target,
                            Vector3f_t* local) {
    /* Parameter validation */
    if (local == NULL) {
        return COORD_ERROR_INVALID_PARAM;
    }

    /* Convert origin and target to ECI coordinates */
    Vector3f_t origin_eci, target_eci;
    CoordinateError_t result;

    result = GeodeticToECI(origin, &origin_eci);
    if (result != COORD_SUCCESS) return result;

    result = GeodeticToECI(target, &target_eci);
    if (result != COORD_SUCCESS) return result;

    /* Step 1: Compute local frame basis vectors */

    /* Z-axis: Local vertical (normal to Earth's surface at origin) */
    /* Perturb origin altitude by 1 meter to compute normal vector */
    GeodeticPosition_t origin_pert = origin;
    origin_pert.altitude += 1.0f;

    Vector3f_t origin_pert_eci;
    result = GeodeticToECI(origin_pert, &origin_pert_eci);
    if (result != COORD_SUCCESS) return result;

    Vector3f_t z_local = Vector3f_Subtract(origin_pert_eci, origin_eci);
    z_local = Vector3f_Normalize(z_local);

    /* X-axis: Points from origin to target */
    Vector3f_t x_local = Vector3f_Subtract(target_eci, origin_eci);
    x_local = Vector3f_Normalize(x_local);

    /* Y-axis: Completes right-handed coordinate system */
    Vector3f_t y_local = Vector3f_Cross(z_local, x_local);
    y_local = Vector3f_Normalize(y_local);

    /* Step 2: Construct rotation matrix (ECI to Local) */
    /* MATLAB: R_ECI_to_Local = [x_local, y_local, z_local]' */
    /* This means: R[row][col] where each row is a basis vector */
    float R[3][3] = {
        {x_local.x, x_local.y, x_local.z},  /* Row 0: x_local vector */
        {y_local.x, y_local.y, y_local.z},  /* Row 1: y_local vector */
        {z_local.x, z_local.y, z_local.z}   /* Row 2: z_local vector */
    };

    /* Rotation matrix constructed successfully */

    /* Step 3: Apply transformation: local = R * (eci - origin_eci) */
    Vector3f_t relative_eci = Vector3f_Subtract(eci, origin_eci);

    local->x = R[0][0] * relative_eci.x + R[0][1] * relative_eci.y + R[0][2] * relative_eci.z;
    local->y = R[1][0] * relative_eci.x + R[1][1] * relative_eci.y + R[1][2] * relative_eci.z;
    local->z = R[2][0] * relative_eci.x + R[2][1] * relative_eci.y + R[2][2] * relative_eci.z;

    return COORD_SUCCESS;
}

/******************************************************************************
 * @brief Convert Local frame position vector to ECI coordinates
 ******************************************************************************/
CoordinateError_t LocalToECI(const Vector3f_t local,
                            const GeodeticPosition_t origin,
                            const GeodeticPosition_t target,
                            Vector3f_t* eci) {
    /* Parameter validation */
    if (eci == NULL) {
        return COORD_ERROR_INVALID_PARAM;
    }

    /* Convert origin and target to ECI coordinates */
    Vector3f_t origin_eci, target_eci;
    CoordinateError_t result;

    result = GeodeticToECI(origin, &origin_eci);
    if (result != COORD_SUCCESS) return result;

    result = GeodeticToECI(target, &target_eci);
    if (result != COORD_SUCCESS) return result;

    /* Step 1: Compute local frame basis vectors (same as ECIToLocal) */

    /* Z-axis: Local vertical */
    GeodeticPosition_t origin_pert = origin;
    origin_pert.altitude += 1.0f;

    Vector3f_t origin_pert_eci;
    result = GeodeticToECI(origin_pert, &origin_pert_eci);
    if (result != COORD_SUCCESS) return result;

    Vector3f_t z_local = Vector3f_Subtract(origin_pert_eci, origin_eci);
    z_local = Vector3f_Normalize(z_local);

    /* X-axis: Points from origin to target */
    Vector3f_t x_local = Vector3f_Subtract(target_eci, origin_eci);
    x_local = Vector3f_Normalize(x_local);

    /* Y-axis: Completes right-handed coordinate system */
    Vector3f_t y_local = Vector3f_Cross(z_local, x_local);
    y_local = Vector3f_Normalize(y_local);

    /* Step 2: Construct rotation matrix (Local to ECI) - transpose of ECI to Local */
    float R[3][3] = {
        {x_local.x, x_local.y, x_local.z},
        {y_local.x, y_local.y, y_local.z},
        {z_local.x, z_local.y, z_local.z}
    };

    /* Step 3: Apply transformation: eci = origin_eci + R * local */
    eci->x = origin_eci.x + R[0][0] * local.x + R[0][1] * local.y + R[0][2] * local.z;
    eci->y = origin_eci.y + R[1][0] * local.x + R[1][1] * local.y + R[1][2] * local.z;
    eci->z = origin_eci.z + R[2][0] * local.x + R[2][1] * local.y + R[2][2] * local.z;

    return COORD_SUCCESS;
}

/******************************************************************************
 * @brief Convert ECI velocity vector to Local frame coordinates
 * Note: Velocity transforms with rotation matrix ONLY (no translation)
 ******************************************************************************/
CoordinateError_t ECIToLocalVelocity(const Vector3f_t eci,
                                    const GeodeticPosition_t origin,
                                    const GeodeticPosition_t target,
                                    Vector3f_t* local) {
    /* Parameter validation */
    if (local == NULL) {
        return COORD_ERROR_INVALID_PARAM;
    }

    /* Convert origin and target to ECI coordinates */
    Vector3f_t origin_eci, target_eci;
    CoordinateError_t result;

    result = GeodeticToECI(origin, &origin_eci);
    if (result != COORD_SUCCESS) return result;

    result = GeodeticToECI(target, &target_eci);
    if (result != COORD_SUCCESS) return result;

    /* Step 1: Compute local frame basis vectors (same as position) */

    /* Z-axis: Local vertical */
    GeodeticPosition_t origin_pert = origin;
    origin_pert.altitude += 1.0f;

    Vector3f_t origin_pert_eci;
    result = GeodeticToECI(origin_pert, &origin_pert_eci);
    if (result != COORD_SUCCESS) return result;

    Vector3f_t z_local = Vector3f_Subtract(origin_pert_eci, origin_eci);
    z_local = Vector3f_Normalize(z_local);

    /* X-axis: Points from origin to target */
    Vector3f_t x_local = Vector3f_Subtract(target_eci, origin_eci);
    x_local = Vector3f_Normalize(x_local);

    /* Y-axis: Completes right-handed coordinate system */
    Vector3f_t y_local = Vector3f_Cross(z_local, x_local);
    y_local = Vector3f_Normalize(y_local);

    /* Step 2: Construct rotation matrix (ECI to Local) */
    float R[3][3] = {
        {x_local.x, y_local.x, z_local.x},
        {x_local.y, y_local.y, z_local.y},
        {x_local.z, y_local.z, z_local.z}
    };

    /* Step 3: Apply ONLY rotation (no translation for velocity) */
    local->x = R[0][0] * eci.x + R[0][1] * eci.y + R[0][2] * eci.z;
    local->y = R[1][0] * eci.x + R[1][1] * eci.y + R[1][2] * eci.z;
    local->z = R[2][0] * eci.x + R[2][1] * eci.y + R[2][2] * eci.z;

    return COORD_SUCCESS;
}

/******************************************************************************
 * @brief Convert Local frame velocity vector to ECI coordinates
 * Note: Velocity transforms with rotation matrix ONLY (no translation)
 ******************************************************************************/
CoordinateError_t LocalToECIVelocity(const Vector3f_t local,
                                    const GeodeticPosition_t origin,
                                    const GeodeticPosition_t target,
                                    Vector3f_t* eci) {
    /* Parameter validation */
    if (eci == NULL) {
        return COORD_ERROR_INVALID_PARAM;
    }

    /* Convert origin and target to ECI coordinates */
    Vector3f_t origin_eci, target_eci;
    CoordinateError_t result;

    result = GeodeticToECI(origin, &origin_eci);
    if (result != COORD_SUCCESS) return result;

    result = GeodeticToECI(target, &target_eci);
    if (result != COORD_SUCCESS) return result;

    /* Step 1: Compute local frame basis vectors (same as position) */

    /* Z-axis: Local vertical */
    GeodeticPosition_t origin_pert = origin;
    origin_pert.altitude += 1.0f;

    Vector3f_t origin_pert_eci;
    result = GeodeticToECI(origin_pert, &origin_pert_eci);
    if (result != COORD_SUCCESS) return result;

    Vector3f_t z_local = Vector3f_Subtract(origin_pert_eci, origin_eci);
    z_local = Vector3f_Normalize(z_local);

    /* X-axis: Points from origin to target */
    Vector3f_t x_local = Vector3f_Subtract(target_eci, origin_eci);
    x_local = Vector3f_Normalize(x_local);

    /* Y-axis: Completes right-handed coordinate system */
    Vector3f_t y_local = Vector3f_Cross(z_local, x_local);
    y_local = Vector3f_Normalize(y_local);

    /* Step 2: Construct rotation matrix (Local to ECI) - transpose of ECI to Local */
    float R[3][3] = {
        {x_local.x, x_local.y, x_local.z},
        {y_local.x, y_local.y, y_local.z},
        {z_local.x, z_local.y, z_local.z}
    };

    /* Step 3: Apply ONLY rotation (no translation for velocity) */
    eci->x = R[0][0] * local.x + R[0][1] * local.y + R[0][2] * local.z;
    eci->y = R[1][0] * local.x + R[1][1] * local.y + R[1][2] * local.z;
    eci->z = R[2][0] * local.x + R[2][1] * local.y + R[2][2] * local.z;

    return COORD_SUCCESS;
}

