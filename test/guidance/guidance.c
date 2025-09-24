/******************************************************************************
 * ISA Flight Software - Guidance Module Implementation
 * @file guidance.c
 * @brief Projectile Flight Computer Guidance Module Implementation
 * @details Implements proportional navigation guidance with impact angle control for precision-guided projectiles
 * @author Flight Software Team, Spacelabs (MATLAB algorithm by Aerospace Team, Spacelabs)
 * @date 2025
 * @version 1.0
 *
 * Algorithm: Based on onboard_guidance_algorithm_3.m MATLAB implementation
 * MISRA C: Compliant Implementation (fixed-width types, parameter validation, error handling)
 *
 * Guidance Algorithm Overview:
 * 1. Convert states to Local coordinate frame
 * 2. Calculate proportional navigation command
 * 3. Apply impact angle control using quaternion rotations
 * 4. Switch to terminal phase (ballistic) when < 100m from target
 * 5. Transform commands back to ECI frame for physics integration
 ******************************************************************************/

#include "guidance.h"
#include "math_utils.h"
#include <math.h>        /* For trigonometric functions, sqrtf(), acosf(), etc. */
#include <string.h>      /* For memset() */
#include <stdio.h>       /* For printf() in debug functions */

/* Degree to radian conversion constants */
#define DEG_TO_RAD (3.141592653589793f / 180.0f)
#define RAD_TO_DEG (180.0f / 3.141592653589793f)

/* Numerical constants for stability */
#define GUIDANCE_EPSILON 1e-10f

/******************************************************************************
 * @brief Initialize the guidance module
 ******************************************************************************/
GuidanceError_t guidanceInit(GuidanceState_t* state) {
    /* Parameter validation */
    if (state == NULL) {
        return GUIDANCE_ERROR_INVALID_PARAM;
    }

    /* Initialize all state to zero (safest starting condition) */
    memset(state, 0, sizeof(GuidanceState_t));

    /* Set default values */
    state->terminalPhaseActive = false;
    state->time = 0.0f;
    state->previousDistance = 0.0f;

    /* Set maximum look angle */
    state->maximumLookAngleRad = GUIDANCE_MAXIMUM_LOOK_ANGLE_DEG * DEG_TO_RAD;

    return GUIDANCE_SUCCESS;
}

/******************************************************************************
 * @brief Configure target position for guidance
 ******************************************************************************/
GuidanceError_t guidanceSetTarget(GuidanceState_t* state,
                                 float targetLat, float targetLon, float targetAlt) {
    /* Parameter validation */
    if (state == NULL) {
        return GUIDANCE_ERROR_INVALID_PARAM;
    }

    /* Store target position */
    state->targetPosition.latitude = targetLat;
    state->targetPosition.longitude = targetLon;
    state->targetPosition.altitude = targetAlt;

    /* Convert target to ECI coordinates */
    CoordinateError_t coordResult = GeodeticToECI(state->targetPosition, &state->targetPositionECI);
    if (coordResult != COORD_SUCCESS) {
        return GUIDANCE_ERROR_COORD_TRANSFORM_FAILED;
    }

    /* Assume stationary target */
    state->targetVelocityECI = Vector3f_Create(0.0f, 0.0f, 0.0f);

    return GUIDANCE_SUCCESS;
}

/******************************************************************************
 * @brief Set desired impact angles
 ******************************************************************************/
GuidanceError_t guidanceSetImpactAngles(GuidanceState_t* state,
                                       float elevationAngle, float azimuthAngle) {
    /* Parameter validation */
    if (state == NULL) {
        return GUIDANCE_ERROR_INVALID_PARAM;
    }

    /* Convert degrees to radians and store */
    state->desiredElevationAngle = elevationAngle * DEG_TO_RAD;
    state->desiredAzimuthAngle = azimuthAngle * DEG_TO_RAD;

    /* Calculate desired impact direction unit vector: e_f = [cos(theta_f)*cos(psi_f), cos(theta_f)*sin(psi_f), sin(theta_f)] */
    float cos_theta_f = cosf(state->desiredElevationAngle);
    float sin_theta_f = sinf(state->desiredElevationAngle);
    float cos_psi_f = cosf(state->desiredAzimuthAngle);
    float sin_psi_f = sinf(state->desiredAzimuthAngle);

    state->desiredImpactDirection = Vector3f_Create(
        cos_theta_f * cos_psi_f,
        cos_theta_f * sin_psi_f,
        sin_theta_f
    );

    return GUIDANCE_SUCCESS;
}

/******************************************************************************
 * @brief Set origin (launch) position
 ******************************************************************************/
GuidanceError_t guidanceSetOrigin(GuidanceState_t* state,
                                 float originLat, float originLon, float originAlt) {
    /* Parameter validation */
    if (state == NULL) {
        return GUIDANCE_ERROR_INVALID_PARAM;
    }

    /* Store origin position */
    state->originPosition.latitude = originLat;
    state->originPosition.longitude = originLon;
    state->originPosition.altitude = originAlt;

    return GUIDANCE_SUCCESS;
}

/******************************************************************************
 * @brief Update projectile state from navigation system
 ******************************************************************************/
GuidanceError_t guidanceUpdateState(GuidanceState_t* state,
                                   const Vector3f_t positionECI,
                                   const Vector3f_t velocityECI) {
    /* Parameter validation */
    if (state == NULL) {
        return GUIDANCE_ERROR_INVALID_PARAM;
    }

    /* Update projectile state */
    state->positionECI = positionECI;
    state->velocityECI = velocityECI;

    return GUIDANCE_SUCCESS;
}

/******************************************************************************
 * @brief Execute guidance algorithm (main function)
 ******************************************************************************/
GuidanceError_t guidanceExecute(GuidanceState_t* state,
                               float timeStep,
                               GuidanceOutput_t* output) {
    /* Parameter validation */
    if (state == NULL || output == NULL) {
        return GUIDANCE_ERROR_INVALID_PARAM;
    }

    /* Convert current ECI states to local frame for guidance calculations */
    Vector3f_t positionLocal, velocityLocal;
    CoordinateError_t coordResult;

    coordResult = ECIToLocal(state->positionECI, state->originPosition, state->targetPosition, &positionLocal);
    if (coordResult != COORD_SUCCESS) {
        printf("COORD ERROR: %d\n", coordResult);
        return GUIDANCE_ERROR_COORD_TRANSFORM_FAILED;
    }

    coordResult = ECIToLocalVelocity(state->velocityECI, state->originPosition, state->targetPosition, &velocityLocal);
    if (coordResult != COORD_SUCCESS) {
        printf("COORD ERROR: %d\n", coordResult);
        return GUIDANCE_ERROR_COORD_TRANSFORM_FAILED;
    }

    /* Convert target position to local frame */
    Vector3f_t targetPositionLocal;
    coordResult = ECIToLocal(state->targetPositionECI, state->originPosition, state->targetPosition, &targetPositionLocal);
    if (coordResult != COORD_SUCCESS) {
        printf("COORD ERROR: %d\n", coordResult);
        return GUIDANCE_ERROR_COORD_TRANSFORM_FAILED;
    }

    /* Calculate distance to actual target (no offset for distance calculation) */
    Vector3f_t targetRelativeVector = Vector3f_Subtract(targetPositionLocal, positionLocal);
    /* Note: distanceToActualTarget not used in current implementation but kept for reference */
    (void)Vector3f_Norm(targetRelativeVector);
    
    /* For guidance calculations, use target with offset (MATLAB algorithm) */
    Vector3f_t targetWithOffset = Vector3f_Create(
        targetPositionLocal.x,
        targetPositionLocal.y,
        targetPositionLocal.z + GUIDANCE_TERMINAL_PHASE_DISTANCE
    );
    Vector3f_t guidanceTargetVector = Vector3f_Subtract(targetWithOffset, positionLocal);
    float distance = Vector3f_Norm(guidanceTargetVector);
    Vector3f_t lineOfSightUnit = Vector3f_Normalize(guidanceTargetVector);

    /* Velocity calculations */
    float velocityMagnitude = Vector3f_Norm(velocityLocal);
    Vector3f_t velocityUnit = Vector3f_Normalize(velocityLocal);

    /* Current Euler angles - calculated for reference but not used in current guidance logic */
    (void)asinf(fmaxf(-1.0f, fminf(1.0f, velocityLocal.z / velocityMagnitude))); /* theta (pitch) */
    (void)atan2f(velocityLocal.y, velocityLocal.x); /* psi (yaw) */

    /* Guidance look angle */
    float sigma = acosf(fmaxf(-1.0f, fminf(1.0f, Vector3f_Dot(velocityUnit, lineOfSightUnit))));

    /* Initialize acceleration command */
    Vector3f_t accelerationCommandLocal = Vector3f_Create(0.0f, 0.0f, 0.0f);

    /* Check if we should switch to terminal phase - MATLAB uses guidance distance r with offset */
    if (distance <= GUIDANCE_TERMINAL_PHASE_DISTANCE && !state->terminalPhaseActive) {
        state->terminalPhaseActive = true;
        printf("Entered terminal phase at t=%.2f s, distance=%.2f m\n", state->time, distance);
    }

    if (state->terminalPhaseActive) {
        /* Terminal phase - ballistic flight (zero acceleration command) */
        accelerationCommandLocal = Vector3f_Create(0.0f, 0.0f, 0.0f);
    } else {
        /* Guided phase - calculate proportional navigation + impact angle control */

        /* Proportional navigation: omega_L = (cross(V_m_local, R)) / (r²) */
        Vector3f_t omega_L = Vector3f_Scale(
            1.0f / (distance * distance),
            Vector3f_Cross(velocityLocal, guidanceTargetVector)
        );

        /* Basic proportional navigation: A_P = N × omega_L × V_m_local (calculated for reference) */
        (void)Vector3f_Cross(
            Vector3f_Scale(GUIDANCE_NAVIGATION_GAIN, omega_L),
            velocityLocal
        );

        /* PN calculation completed */

        /* Impact angle control calculations - MATLAB: k_L = (cross(e_R, e_m)) / norm(...) */
        Vector3f_t k_L = Vector3f_Normalize(
            Vector3f_Cross(lineOfSightUnit, velocityUnit)  /* e_R x e_m */
        );

        /* Angles for quaternion rotation */
        float eta = sigma / (GUIDANCE_NAVIGATION_GAIN - 1.0f);
        float mu = sigma + eta;

        /* Quaternion components for rotation */
        float w_q = cosf(-mu / 2.0f);
        float x_q = sinf(-mu / 2.0f) * k_L.x;
        float y_q = sinf(-mu / 2.0f) * k_L.y;
        float z_q = sinf(-mu / 2.0f) * k_L.z;

        /* Rotation matrix from quaternion */
        float R_q[3][3] = {
            {w_q*w_q + x_q*x_q - y_q*y_q - z_q*z_q, 2*(x_q*y_q - w_q*z_q), 2*(x_q*z_q + w_q*y_q)},
            {2*(x_q*y_q + w_q*z_q), w_q*w_q - x_q*x_q + y_q*y_q - z_q*z_q, 2*(y_q*z_q - w_q*x_q)},
            {2*(x_q*z_q - w_q*y_q), 2*(y_q*z_q + w_q*x_q), w_q*w_q - x_q*x_q - y_q*y_q + z_q*z_q}
        };

        /* Rotate velocity to predicted velocity: V_p = R_q * V_m_local */
        Vector3f_t V_p = Vector3f_Create(
            R_q[0][0]*velocityLocal.x + R_q[0][1]*velocityLocal.y + R_q[0][2]*velocityLocal.z,
            R_q[1][0]*velocityLocal.x + R_q[1][1]*velocityLocal.y + R_q[1][2]*velocityLocal.z,
            R_q[2][0]*velocityLocal.x + R_q[2][1]*velocityLocal.y + R_q[2][2]*velocityLocal.z
        );

        /* Impact angle error */
        float delta = acosf(fmaxf(-1.0f, fminf(1.0f, Vector3f_Dot(state->desiredImpactDirection, Vector3f_Normalize(V_p)))));

        /* Time to go estimate */
        float timeToGo = distance / fmaxf(velocityMagnitude * cosf(sigma), GUIDANCE_EPSILON);

        /* Look angle ratio - MATLAB: sigma_not = sigma / sigma_max */
        float sigma_not = sigma / state->maximumLookAngleRad;
        float f_sigma_not = 1.0f - powf(fabsf(sigma_not), GUIDANCE_IMPACT_ANGLE_CONTROL_D);  /* MATLAB: 1 - (abs(sigma_not)^d) */

        /* Impact angle rate command */
        float delta_dot = (-GUIDANCE_IMPACT_ANGLE_CONTROL_K * f_sigma_not / timeToGo) *
                         (GUIDANCE_IMPACT_ANGLE_CONTROL_A * sig(GUIDANCE_IMPACT_ANGLE_CONTROL_M, delta) +
                          GUIDANCE_IMPACT_ANGLE_CONTROL_B * sig(GUIDANCE_IMPACT_ANGLE_CONTROL_N, delta));

        /* Impact angle control acceleration */
        Vector3f_t l_f = Vector3f_Normalize(
            Vector3f_Cross(state->desiredImpactDirection, Vector3f_Normalize(V_p))
        );

        Vector3f_t omega_P = Vector3f_Scale(delta_dot, l_f);
        (void)Vector3f_Cross(omega_P, V_p);  /* A_F calculated for reference */

        /* Coordinate system vectors */
        Vector3f_t l_m = Vector3f_Cross(k_L, velocityUnit);
        Vector3f_t l_P = Vector3f_Cross(k_L, Vector3f_Normalize(V_p));

        /* Impact angle control gains */
        float h_delta = GUIDANCE_IMPACT_ANGLE_CONTROL_A * sig(GUIDANCE_IMPACT_ANGLE_CONTROL_M, delta) +
                       GUIDANCE_IMPACT_ANGLE_CONTROL_B * sig(GUIDANCE_IMPACT_ANGLE_CONTROL_N, delta);

        float a_p = (-GUIDANCE_NAVIGATION_GAIN * velocityMagnitude * velocityMagnitude * sinf(sigma)) / distance;

        float a_I_lm = ((GUIDANCE_NAVIGATION_GAIN - 1.0f) * GUIDANCE_IMPACT_ANGLE_CONTROL_K * f_sigma_not * velocityMagnitude * h_delta / timeToGo) *
                      Vector3f_Dot(Vector3f_Cross(l_f, Vector3f_Normalize(V_p)), l_P);  /* MATLAB: cross(l_f, e_p) */

        float a_I_kl = (GUIDANCE_IMPACT_ANGLE_CONTROL_K * f_sigma_not * velocityMagnitude * sinf(sigma) * h_delta / (timeToGo * sinf(eta))) *
                      Vector3f_Dot(Vector3f_Cross(l_f, Vector3f_Normalize(V_p)), k_L);  /* MATLAB: cross(l_f, e_p) */

        /* Full MATLAB algorithm: A_M_local = (a_p + a_I_lm) * l_m + a_I_kl * k_L */
        Vector3f_t accelerationTerm1 = Vector3f_Scale(a_p + a_I_lm, l_m);
        Vector3f_t accelerationTerm2 = Vector3f_Scale(a_I_kl, k_L);
        accelerationCommandLocal = Vector3f_Add(accelerationTerm1, accelerationTerm2);
    }

    /* Store local acceleration command */
    state->accelerationCommandLocal = accelerationCommandLocal;

    /* Convert guidance acceleration to ECI frame */
    coordResult = LocalToECIVelocity(accelerationCommandLocal, state->originPosition, state->targetPosition, &state->accelerationCommandECI);
    if (coordResult != COORD_SUCCESS) {
        printf("COORD ERROR: %d\n", coordResult);
        return GUIDANCE_ERROR_COORD_TRANSFORM_FAILED;
    }

    /* Calculate drag acceleration in ECI frame (MATLAB implementation: drag opposes velocity) */
    float velocityMagnitudeECI = Vector3f_Norm(state->velocityECI);
    if (velocityMagnitudeECI > GUIDANCE_EPSILON) {
        float dynamicPressure = 0.5f * GUIDANCE_AIR_DENSITY * velocityMagnitudeECI * velocityMagnitudeECI;
        float referenceArea = (3.141592653589793f * GUIDANCE_PROJECTILE_DIAMETER * GUIDANCE_PROJECTILE_DIAMETER) / 4.0f;
        float dragForce = dynamicPressure * GUIDANCE_PROJECTILE_DRAG_COEFFICIENT * referenceArea;

        /* Drag acceleration components in ECI (opposes velocity direction) */
        float dragAccelMagnitude = dragForce / GUIDANCE_PROJECTILE_MASS;
        state->dragAccelerationECI = Vector3f_Create(
            dragAccelMagnitude * state->velocityECI.x / velocityMagnitudeECI,
            dragAccelMagnitude * state->velocityECI.y / velocityMagnitudeECI,
            dragAccelMagnitude * state->velocityECI.z / velocityMagnitudeECI
        );
    } else {
        state->dragAccelerationECI = Vector3f_Create(0.0f, 0.0f, 0.0f);
    }

    /* Calculate gravity acceleration in ECI frame */
    Vector3f_t gravityLocal = Vector3f_Create(0.0f, 0.0f, -GUIDANCE_GRAVITY_ACCELERATION);
    coordResult = LocalToECIVelocity(gravityLocal, state->originPosition, state->targetPosition, &state->gravityAccelerationECI);
    if (coordResult != COORD_SUCCESS) {
        printf("COORD ERROR: %d\n", coordResult);
        return GUIDANCE_ERROR_COORD_TRANSFORM_FAILED;
    }

    /* Update time and previous distance */
    state->time += timeStep;
    state->previousDistance = distance;  /* Store guidance distance (with offset) for next iteration - MATLAB behavior */

    /* Fill output structure */
    output->accelerationCommandLocal = state->accelerationCommandLocal;
    output->accelerationCommandECI = state->accelerationCommandECI;
    output->distanceToTarget = distance;  /* Report guidance distance to match MATLAB output */
    output->timeToImpact = guidanceGetTimeToImpact(state);
    output->terminalPhaseActive = state->terminalPhaseActive;

    return GUIDANCE_SUCCESS;
}

/******************************************************************************
 * @brief Get current acceleration command in ECI frame
 ******************************************************************************/
GuidanceError_t guidanceGetAccelerationCommandECI(const GuidanceState_t* state,
                                                 Vector3f_t* accelerationECI) {
    if (state == NULL || accelerationECI == NULL) {
        return GUIDANCE_ERROR_INVALID_PARAM;
    }

    *accelerationECI = state->accelerationCommandECI;
    return GUIDANCE_SUCCESS;
}

/******************************************************************************
 * @brief Get current acceleration command in local frame
 ******************************************************************************/
GuidanceError_t guidanceGetAccelerationCommandLocal(const GuidanceState_t* state,
                                                   Vector3f_t* accelerationLocal) {
    if (state == NULL || accelerationLocal == NULL) {
        return GUIDANCE_ERROR_INVALID_PARAM;
    }

    *accelerationLocal = state->accelerationCommandLocal;
    return GUIDANCE_SUCCESS;
}

/******************************************************************************
 * @brief Check if terminal phase is active
 ******************************************************************************/
bool guidanceIsTerminalPhaseActive(const GuidanceState_t* state) {
    if (state == NULL) {
        return false;
    }
    return state->terminalPhaseActive;
}

/******************************************************************************
 * @brief Calculate distance to target
 ******************************************************************************/
float guidanceGetDistanceToTarget(const GuidanceState_t* state) {
    if (state == NULL) {
        return 0.0f;
    }

    /* Convert positions to local frame */
    Vector3f_t positionLocal, targetLocal;
    CoordinateError_t coordResult;

    coordResult = ECIToLocal(state->positionECI, state->originPosition, state->targetPosition, &positionLocal);
    if (coordResult != COORD_SUCCESS) return 0.0f;

    coordResult = ECIToLocal(state->targetPositionECI, state->originPosition, state->targetPosition, &targetLocal);
    if (coordResult != COORD_SUCCESS) return 0.0f;

    /* Return distance to actual target (no offset) */
    Vector3f_t relativeVector = Vector3f_Subtract(targetLocal, positionLocal);
    return Vector3f_Norm(relativeVector);
}

/******************************************************************************
 * @brief Calculate estimated time to impact
 ******************************************************************************/
float guidanceGetTimeToImpact(const GuidanceState_t* state) {
    if (state == NULL) {
        return 0.0f;
    }

    /* Convert velocity to local frame */
    Vector3f_t velocityLocal;
    CoordinateError_t coordResult = ECIToLocalVelocity(state->velocityECI, state->originPosition, state->targetPosition, &velocityLocal);
    if (coordResult != COORD_SUCCESS) return 0.0f;

    float velocityMagnitude = Vector3f_Norm(velocityLocal);
    float distance = guidanceGetDistanceToTarget(state);

    if (velocityMagnitude > GUIDANCE_EPSILON) {
        return distance / velocityMagnitude;
    }

    return 0.0f;
}

/******************************************************************************
 * @brief Print guidance state for debugging
 ******************************************************************************/
void guidancePrintState(const GuidanceState_t* state) {
    if (state == NULL) {
        printf("Error: NULL guidance state\n");
        return;
    }

    printf("Time: %.2f s\n", state->time);
    printf("GUIDANCE COMMAND LOCAL: (%.2f, %.2f, %.2f) m/s^2\n",
           state->accelerationCommandLocal.x,
           state->accelerationCommandLocal.y,
           state->accelerationCommandLocal.z);
    printf("GUIDANCE COMMAND ECI: (%.2f, %.2f, %.2f) m/s^2\n",
           state->accelerationCommandECI.x,
           state->accelerationCommandECI.y,
           state->accelerationCommandECI.z);
    printf("DRAG IN ECI: (%.2f, %.2f, %.2f) m/s^2\n",
           state->dragAccelerationECI.x,
           state->dragAccelerationECI.y,
           state->dragAccelerationECI.z);
    printf("GRAVITY IN ECI: (%.2f, %.2f, %.2f) m/s^2\n",
           state->gravityAccelerationECI.x,
           state->gravityAccelerationECI.y,
           state->gravityAccelerationECI.z);
    printf("Projectile ECI position: (%.2f, %.2f, %.2f) m\n",
           state->positionECI.x,
           state->positionECI.y,
           state->positionECI.z);
    printf("Projectile ECI velocity: (%.2f, %.2f, %.2f) m/s\n",
           state->velocityECI.x,
           state->velocityECI.y,
           state->velocityECI.z);

    float velocityMagnitude = Vector3f_Norm(state->velocityECI);
    float theta = RAD_TO_DEG * asinf(fmaxf(-1.0f, fminf(1.0f, state->velocityECI.z / velocityMagnitude)));
    float psi = RAD_TO_DEG * atan2f(state->velocityECI.y, state->velocityECI.x);
    printf("Theta: %.2f deg, Psi: %.2f deg\n", theta, psi);

    /* Calculate guidance distance (with offset) to match MATLAB output */
    Vector3f_t positionLocal, targetLocal;
    CoordinateError_t coordResult;
    coordResult = ECIToLocal(state->positionECI, state->originPosition, state->targetPosition, &positionLocal);
    if (coordResult == COORD_SUCCESS) {
        coordResult = ECIToLocal(state->targetPositionECI, state->originPosition, state->targetPosition, &targetLocal);
        if (coordResult == COORD_SUCCESS) {
            Vector3f_t targetWithOffset = Vector3f_Create(targetLocal.x, targetLocal.y, targetLocal.z + GUIDANCE_TERMINAL_PHASE_DISTANCE);
            Vector3f_t guidanceVector = Vector3f_Subtract(targetWithOffset, positionLocal);
            float guidanceDistance = Vector3f_Norm(guidanceVector);
            printf("Distance to target: %.2f m\n", guidanceDistance);
        }
    }

    if (state->terminalPhaseActive) {
        printf("Terminal phase active\n");
    }
    printf("\n");
}
