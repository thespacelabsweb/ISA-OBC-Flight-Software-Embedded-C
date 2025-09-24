/******************************************************************************
 * ISA Flight Software
 * 
 * File: guidance.h
 * Description: Guidance module interface
 *****************************************************************************/

#ifndef GUIDANCE_H
#define GUIDANCE_H

#include "../../include/common/types.h"
#include "../../include/math/vector3.h"

/**
 * @brief Guidance module state
 */
typedef struct {
    /* Target information */
    GeodeticPosition targetPosition;
    Vector3 targetPositionECI;
    Vector3 targetVelocityECI;
    
    /* Desired impact angles */
    double thetaF;  /* Desired elevation at impact (radians) */
    double psiF;    /* Desired azimuth at impact (radians) */
    
    /* Current projectile state */
    Vector3 positionECI;
    Vector3 velocityECI;
    
    /* Local frame reference */
    GeodeticPosition originPosition;
    
    /* Guidance parameters */
    GuidanceParameters params;
    ProjectileParameters projectileParams;
    EnvironmentParameters envParams;
    SimulationParameters simParams;
    
    /* Guidance state */
    double time;
    double previousDistance;
    Bool terminalPhaseActive;
    
    /* Output acceleration commands */
    Vector3 accelerationCommandLocal;
    Vector3 accelerationCommandECI;
    
    /* Debug information */
    Vector3 dragAccelerationECI;
    Vector3 gravityAccelerationECI;
} GuidanceState;

/**
 * @brief Initialize the guidance module
 * 
 * @param state Pointer to guidance state structure
 * @param originPosition Origin position (launch point)
 * @param targetPosition Target position
 * @param initialPositionECI Initial projectile position in ECI frame
 * @param initialVelocityECI Initial projectile velocity in ECI frame
 * @param thetaF Desired elevation angle at impact (radians)
 * @param psiF Desired azimuth angle at impact (radians)
 * @return Status Status code
 */
Status Guidance_Initialize(
    GuidanceState* state,
    GeodeticPosition originPosition,
    GeodeticPosition targetPosition,
    Vector3 initialPositionECI,
    Vector3 initialVelocityECI,
    double thetaF,
    double psiF
);

/**
 * @brief Update the guidance state with new navigation data
 * 
 * @param state Pointer to guidance state structure
 * @param positionECI Current projectile position in ECI frame
 * @param velocityECI Current projectile velocity in ECI frame
 * @return Status Status code
 */
Status Guidance_UpdateState(
    GuidanceState* state,
    Vector3 positionECI,
    Vector3 velocityECI
);

/**
 * @brief Execute one step of the guidance algorithm
 * 
 * @param state Pointer to guidance state structure
 * @param timeStep Time step in seconds
 * @return Status Status code
 */
Status Guidance_Execute(
    GuidanceState* state,
    double timeStep
);

/**
 * @brief Get the current acceleration command in ECI frame
 * 
 * @param state Pointer to guidance state structure
 * @return Vector3 Acceleration command in ECI frame
 */
Vector3 Guidance_GetAccelerationCommandECI(const GuidanceState* state);

/**
 * @brief Get the current acceleration command in local frame
 * 
 * @param state Pointer to guidance state structure
 * @return Vector3 Acceleration command in local frame
 */
Vector3 Guidance_GetAccelerationCommandLocal(const GuidanceState* state);

/**
 * @brief Check if the guidance algorithm has reached the terminal phase
 * 
 * @param state Pointer to guidance state structure
 * @return Bool TRUE if terminal phase is active, FALSE otherwise
 */
Bool Guidance_IsTerminalPhaseActive(const GuidanceState* state);

/**
 * @brief Calculate the estimated time to impact
 * 
 * @param state Pointer to guidance state structure
 * @return double Estimated time to impact in seconds
 */
double Guidance_GetTimeToImpact(const GuidanceState* state);

/**
 * @brief Calculate the current distance to target
 * 
 * @param state Pointer to guidance state structure
 * @return double Distance to target in meters
 */
double Guidance_GetDistanceToTarget(const GuidanceState* state);

/**
 * @brief Print current guidance state (for debugging)
 * 
 * @param state Pointer to guidance state structure
 */
void Guidance_PrintState(const GuidanceState* state);

#endif /* GUIDANCE_H */
