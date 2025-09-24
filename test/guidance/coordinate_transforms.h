/******************************************************************************
 * ISA Flight Software - Guidance Module Support Library
 * @file coordinate_transforms.h
 * @brief Coordinate transformations between ECI and Local frames for guidance algorithms
 * @details Implements geodetic to ECI conversions and local frame transformations needed for guidance
 * @author Flight Software Team, Spacelabs
 * @date 2025
 * @version 1.0
 *
 * MISRA C: Compliant Implementation (fixed-width types, parameter validation, error handling)
 ******************************************************************************/

#ifndef COORDINATE_TRANSFORMS_H
#define COORDINATE_TRANSFORMS_H

#include <stdint.h>
#include <stdbool.h>
#include "vector3.h"  /* Include our vector3 types */

/* Flight software error codes for coordinate transformations */
typedef enum {
    COORD_SUCCESS = 0U,
    COORD_ERROR_INVALID_PARAM = 1U,
    COORD_ERROR_CONVERSION_FAILED = 2U
} CoordinateError_t;

/* Geodetic position structure (latitude, longitude, altitude) */
typedef struct {
    float latitude;   /* Latitude in degrees (-90 to +90) */
    float longitude;  /* Longitude in degrees (-180 to +180) */
    float altitude;   /* Altitude in meters above WGS84 ellipsoid */
} GeodeticPosition_t;

/* Function prototypes - MISRA C compliant with parameter validation */

/******************************************************************************
 * @brief Convert geodetic coordinates (lat, lon, alt) to ECI coordinates (X, Y, Z)
 * @details Implements WGS84 ellipsoid conversion to Earth-Centered Inertial frame
 * @param geodetic Input geodetic position (lat/lon in degrees, alt in meters)
 * @param eci Output ECI position vector (meters) - must not be NULL
 * @return CoordinateError_t Success or error code
 ******************************************************************************/
CoordinateError_t GeodeticToECI(const GeodeticPosition_t geodetic, Vector3f_t* eci);

/******************************************************************************
 * @brief Convert ECI coordinates to geodetic coordinates (lat, lon, alt)
 * @details Implements inverse WGS84 conversion from ECI to geodetic coordinates
 * @param eci Input ECI position vector (meters)
 * @param geodetic Output geodetic position - must not be NULL
 * @return CoordinateError_t Success or error code
 ******************************************************************************/
CoordinateError_t ECIToGeodetic(const Vector3f_t eci, GeodeticPosition_t* geodetic);

/******************************************************************************
 * @brief Convert ECI position vector to Local frame coordinates
 * @details Local frame: X-axis points from origin to target, Z-axis is local vertical
 * @param eci Input ECI position vector (meters)
 * @param origin Origin position (launch point) in geodetic coordinates
 * @param target Target position in geodetic coordinates
 * @param local Output local frame position vector - must not be NULL
 * @return CoordinateError_t Success or error code
 ******************************************************************************/
CoordinateError_t ECIToLocal(const Vector3f_t eci,
                            const GeodeticPosition_t origin,
                            const GeodeticPosition_t target,
                            Vector3f_t* local);

/******************************************************************************
 * @brief Convert Local frame position vector to ECI coordinates
 * @details Inverse of ECIToLocal transformation
 * @param local Input local frame position vector
 * @param origin Origin position (launch point) in geodetic coordinates
 * @param target Target position in geodetic coordinates
 * @param eci Output ECI position vector (meters) - must not be NULL
 * @return CoordinateError_t Success or error code
 ******************************************************************************/
CoordinateError_t LocalToECI(const Vector3f_t local,
                            const GeodeticPosition_t origin,
                            const GeodeticPosition_t target,
                            Vector3f_t* eci);

/******************************************************************************
 * @brief Convert ECI velocity vector to Local frame coordinates
 * @details Same rotation matrix as position, since velocity transforms identically
 * @param eci Input ECI velocity vector (m/s)
 * @param origin Origin position (launch point) in geodetic coordinates
 * @param target Target position in geodetic coordinates
 * @param local Output local frame velocity vector - must not be NULL
 * @return CoordinateError_t Success or error code
 ******************************************************************************/
CoordinateError_t ECIToLocalVelocity(const Vector3f_t eci,
                                    const GeodeticPosition_t origin,
                                    const GeodeticPosition_t target,
                                    Vector3f_t* local);

/******************************************************************************
 * @brief Convert Local frame velocity vector to ECI coordinates
 * @details Inverse of ECIToLocalVelocity transformation
 * @param local Input local frame velocity vector
 * @param origin Origin position (launch point) in geodetic coordinates
 * @param target Target position in geodetic coordinates
 * @param eci Output ECI velocity vector (m/s) - must not be NULL
 * @return CoordinateError_t Success or error code
 ******************************************************************************/
CoordinateError_t LocalToECIVelocity(const Vector3f_t local,
                                    const GeodeticPosition_t origin,
                                    const GeodeticPosition_t target,
                                    Vector3f_t* eci);

#endif /* COORDINATE_TRANSFORMS_H */

