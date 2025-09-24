/******************************************************************************
 * ISA Flight Software - Guidance Module
 * @file guidance.h
 * @brief Projectile Flight Computer Guidance Module Interface
 * @details Implements proportional navigation guidance with impact angle control for precision-guided projectiles
 * @author Flight Software Team, Spacelabs (MATLAB algorithm by Aerospace Team, Spacelabs)
 * @date 2025
 * @version 1.0
 *
 * Algorithm: Based on onboard_guidance_algorithm_3.m MATLAB implementation
 * MISRA C: Compliant Implementation (fixed-width types, parameter validation, error handling)
 *
 * Guidance Phases:
 * 1. Guided Phase (0-100m): Proportional Navigation + Impact Angle Control
 * 2. Terminal Phase (<100m): Ballistic free-fall
 ******************************************************************************/

#ifndef GUIDANCE_H
#define GUIDANCE_H

#include <stdint.h>
#include <stdbool.h>
#include "vector3.h"
#include "coordinate_transforms.h"

/* Flight software error codes for guidance module */
typedef enum {
    GUIDANCE_SUCCESS = 0U,
    GUIDANCE_ERROR_INVALID_PARAM = 1U,
    GUIDANCE_ERROR_INVALID_STATE = 2U,
    GUIDANCE_ERROR_MATH_ERROR = 3U,
    GUIDANCE_ERROR_COORD_TRANSFORM_FAILED = 4U
} GuidanceError_t;

/* Guidance algorithm constants (from MATLAB onboard_guidance_algorithm_3.m) */
#define GUIDANCE_GRAVITY_ACCELERATION 9.8f                   /* g = 9.8 m/s² */
#define GUIDANCE_NAVIGATION_GAIN 3.0f                        /* N = 3 (dimensionless) */
#define GUIDANCE_AIR_DENSITY 1.225f                          /* rho = 1.225 kg/m³ */

#define GUIDANCE_IMPACT_ANGLE_CONTROL_A 0.100001f           /* a = 0.100001 */
#define GUIDANCE_IMPACT_ANGLE_CONTROL_B 0.100001f           /* b = 0.100001 */
#define GUIDANCE_IMPACT_ANGLE_CONTROL_M 0.90001f            /* m = 0.90001 */
#define GUIDANCE_IMPACT_ANGLE_CONTROL_N 1.30001f            /* n = 1.30001 */
#define GUIDANCE_IMPACT_ANGLE_CONTROL_D 7.5f                /* d = 7.5 */
#define GUIDANCE_IMPACT_ANGLE_CONTROL_K 21.00000001f        /* K = 21.00000001 */

#define GUIDANCE_MAXIMUM_LOOK_ANGLE_DEG 73.0f               /* sigma_max = 73° */

#define GUIDANCE_PROJECTILE_MASS 47.0f                      /* m_c = 47 kg */
#define GUIDANCE_PROJECTILE_DIAMETER 0.155f                 /* dia = 0.155 m */
#define GUIDANCE_PROJECTILE_DRAG_COEFFICIENT 0.2808f        /* C_d = 0.2808 */

#define GUIDANCE_TERMINAL_PHASE_DISTANCE 100.0f             /* r_loop3_start = 100 m */

#define GUIDANCE_SIMULATION_TIME_STEP 0.01f                 /* dt = 0.01 s (10ms) */
#define GUIDANCE_SIMULATION_MAX_TIME 200.0f                 /* max_simulation_time = 200 s */
#define GUIDANCE_MINIMUM_VELOCITY_THRESHOLD 1.0f            /* min_missile_velocity = 1 m/s */

/* Guidance state structure */
typedef struct {
    /* Target information */
    GeodeticPosition_t targetPosition;      /* Target lat/lon/alt (degrees/meters) */
    Vector3f_t targetPositionECI;           /* Target position in ECI frame (meters) */
    Vector3f_t targetVelocityECI;           /* Target velocity in ECI frame (m/s) - assumed zero */

    /* Desired impact angles (radians) */
    float desiredElevationAngle;            /* theta_f - desired elevation at impact */
    float desiredAzimuthAngle;              /* psi_f - desired azimuth at impact */

    /* Current projectile state */
    Vector3f_t positionECI;                  /* Current position in ECI frame (meters) */
    Vector3f_t velocityECI;                  /* Current velocity in ECI frame (m/s) */

    /* Local frame reference */
    GeodeticPosition_t originPosition;       /* Launch point (origin) lat/lon/alt */

    /* Guidance state variables */
    float time;                              /* Current simulation time (seconds) */
    float previousDistance;                  /* Distance to target in previous iteration (meters) */
    bool terminalPhaseActive;                /* True when distance < TERMINAL_PHASE_DISTANCE */

    /* Output acceleration commands */
    Vector3f_t accelerationCommandLocal;     /* Guidance command in local frame (m/s²) */
    Vector3f_t accelerationCommandECI;       /* Guidance command in ECI frame (m/s²) */

    /* Computed forces (for physics integration and debugging) */
    Vector3f_t dragAccelerationECI;          /* Drag acceleration in ECI frame (m/s²) */
    Vector3f_t gravityAccelerationECI;       /* Gravity acceleration in ECI frame (m/s²) */

    /* Internal guidance algorithm variables */
    Vector3f_t desiredImpactDirection;       /* e_f - desired impact unit vector */
    float maximumLookAngleRad;               /* sigma_max in radians */
} GuidanceState_t;

/* Guidance output structure for cleaner API */
typedef struct {
    Vector3f_t accelerationCommandLocal;     /* Guidance command in local frame */
    Vector3f_t accelerationCommandECI;       /* Guidance command in ECI frame */
    float distanceToTarget;                  /* Current distance to target (meters) */
    float timeToImpact;                     /* Estimated time to impact (seconds) */
    bool terminalPhaseActive;               /* Terminal phase status */
} GuidanceOutput_t;

/* Function prototypes - MISRA C compliant with parameter validation */

/******************************************************************************
 * @brief Initialize the guidance module
 * @details Sets up all constants, initializes state to safe defaults
 * @param state Pointer to guidance state structure - must not be NULL
 * @return GuidanceError_t Success or error code
 ******************************************************************************/
GuidanceError_t guidanceInit(GuidanceState_t* state);

/******************************************************************************
 * @brief Configure target position for guidance
 * @details Sets target coordinates and converts to ECI frame
 * @param state Pointer to guidance state structure - must not be NULL
 * @param targetLat Target latitude (degrees)
 * @param targetLon Target longitude (degrees)
 * @param targetAlt Target altitude (meters)
 * @return GuidanceError_t Success or error code
 ******************************************************************************/
GuidanceError_t guidanceSetTarget(GuidanceState_t* state,
                                 float targetLat, float targetLon, float targetAlt);

/******************************************************************************
 * @brief Set desired impact angles
 * @details Configures the desired elevation and azimuth at impact point
 * @param state Pointer to guidance state structure - must not be NULL
 * @param elevationAngle Desired elevation angle at impact (degrees)
 * @param azimuthAngle Desired azimuth angle at impact (degrees)
 * @return GuidanceError_t Success or error code
 ******************************************************************************/
GuidanceError_t guidanceSetImpactAngles(GuidanceState_t* state,
                                       float elevationAngle, float azimuthAngle);

/******************************************************************************
 * @brief Set origin (launch) position
 * @details Configures the launch point for coordinate transformations
 * @param state Pointer to guidance state structure - must not be NULL
 * @param originLat Launch latitude (degrees)
 * @param originLon Launch longitude (degrees)
 * @param originAlt Launch altitude (meters)
 * @return GuidanceError_t Success or error code
 ******************************************************************************/
GuidanceError_t guidanceSetOrigin(GuidanceState_t* state,
                                 float originLat, float originLon, float originAlt);

/******************************************************************************
 * @brief Update projectile state from navigation system
 * @details Updates current position and velocity in ECI frame
 * @param state Pointer to guidance state structure - must not be NULL
 * @param positionECI Current position in ECI frame (meters)
 * @param velocityECI Current velocity in ECI frame (m/s)
 * @return GuidanceError_t Success or error code
 ******************************************************************************/
GuidanceError_t guidanceUpdateState(GuidanceState_t* state,
                                   const Vector3f_t positionECI,
                                   const Vector3f_t velocityECI);

/******************************************************************************
 * @brief Execute guidance algorithm (called every minor cycle)
 * @details Main guidance algorithm: proportional navigation + impact angle control
 * @param state Pointer to guidance state structure - must not be NULL
 * @param timeStep Time step (seconds) - typically 0.01s
 * @param output Pointer to output structure for results - must not be NULL
 * @return GuidanceError_t Success or error code
 ******************************************************************************/
GuidanceError_t guidanceExecute(GuidanceState_t* state,
                               float timeStep,
                               GuidanceOutput_t* output);

/******************************************************************************
 * @brief Get current acceleration command in ECI frame
 * @param state Pointer to guidance state structure - must not be NULL
 * @param accelerationECI Output acceleration command - must not be NULL
 * @return GuidanceError_t Success or error code
 ******************************************************************************/
GuidanceError_t guidanceGetAccelerationCommandECI(const GuidanceState_t* state,
                                                 Vector3f_t* accelerationECI);

/******************************************************************************
 * @brief Get current acceleration command in local frame
 * @param state Pointer to guidance state structure - must not be NULL
 * @param accelerationLocal Output acceleration command - must not be NULL
 * @return GuidanceError_t Success or error code
 ******************************************************************************/
GuidanceError_t guidanceGetAccelerationCommandLocal(const GuidanceState_t* state,
                                                   Vector3f_t* accelerationLocal);

/******************************************************************************
 * @brief Check if terminal phase is active
 * @param state Pointer to guidance state structure - must not be NULL
 * @return true if terminal phase active (distance < 100m), false otherwise
 ******************************************************************************/
bool guidanceIsTerminalPhaseActive(const GuidanceState_t* state);

/******************************************************************************
 * @brief Calculate distance to target in meters
 * @param state Pointer to guidance state structure - must not be NULL
 * @return float Distance to target (meters), or 0.0f on error
 ******************************************************************************/
float guidanceGetDistanceToTarget(const GuidanceState_t* state);

/******************************************************************************
 * @brief Calculate estimated time to impact
 * @param state Pointer to guidance state structure - must not be NULL
 * @return float Estimated time to impact (seconds), or 0.0f on error
 ******************************************************************************/
float guidanceGetTimeToImpact(const GuidanceState_t* state);

/******************************************************************************
 * @brief Print guidance state for debugging
 * @param state Pointer to guidance state structure - must not be NULL
 ******************************************************************************/
void guidancePrintState(const GuidanceState_t* state);

#endif /* GUIDANCE_H */

