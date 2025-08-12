/******************************************************************************
 * ISA Flight Software
 * 
 * File: guidance.c
 * Description: Implementation of guidance algorithm based on MATLAB reference
 *****************************************************************************/

#include <stdio.h>
#include <math.h>
#include "../../include/guidance/guidance.h"
#include "../../include/math/coordinate_transforms.h"
#include "../../include/common/config.h"

/* Helper function to calculate desired impact direction unit vector */
static Vector3 CalculateDesiredImpactDirection(double thetaF, double psiF) {
    return Vector3_Create(
        cos(thetaF) * cos(psiF),
        cos(thetaF) * sin(psiF),
        sin(thetaF)
);
}

Status Guidance_Initialize(
    GuidanceState* state,
    GeodeticPosition originPosition,
    GeodeticPosition targetPosition,
    Vector3 initialPositionECI,
    Vector3 initialVelocityECI,
    double thetaF,
    double psiF
) {
    if (state == NULL) {
        return STATUS_INVALID_PARAM;
    }
    
    /* Store origin and target positions */
    state->originPosition = originPosition;
    state->targetPosition = targetPosition;
    
    /* Convert target position to ECI */
    state->targetPositionECI = GeodeticToECI(targetPosition, NULL);
    state->targetVelocityECI = Vector3_Create(0.0, 0.0, 0.0);  /* Stationary target */
    
    /* Store desired impact angles */
    state->thetaF = thetaF;
    state->psiF = psiF;
    
    /* Store initial projectile state */
    state->positionECI = initialPositionECI;
    state->velocityECI = initialVelocityECI;
    
    /* Initialize guidance parameters with defaults */
    state->params = GUIDANCE_DEFAULT;
    state->projectileParams = PROJECTILE_DEFAULT;
    state->envParams = ENVIRONMENT_DEFAULT;
    state->simParams = SIMULATION_DEFAULT;
    
    /* Calculate reference area */
    state->projectileParams.refArea = (PI * state->projectileParams.diameter * 
                                      state->projectileParams.diameter) / 4.0;
    
    /* Initialize guidance state */
    state->time = 0.0;
    state->terminalPhaseActive = BOOL_FALSE;
    
    /* Calculate initial distance to target */
    Vector3 positionLocal = ECIToLocal(state->positionECI, originPosition, targetPosition);
    Vector3 targetLocal = ECIToLocal(state->targetPositionECI, originPosition, targetPosition);
    Vector3 relativeVector = Vector3_Subtract(targetLocal, positionLocal);
    state->previousDistance = Vector3_Norm(relativeVector);
    
    /* Initialize acceleration commands to zero */
    state->accelerationCommandLocal = Vector3_Create(0.0, 0.0, 0.0);
    state->accelerationCommandECI = Vector3_Create(0.0, 0.0, 0.0);
    state->dragAccelerationECI = Vector3_Create(0.0, 0.0, 0.0);
    state->gravityAccelerationECI = Vector3_Create(0.0, 0.0, 0.0);
    
    return STATUS_OK;
}

Status Guidance_UpdateState(
    GuidanceState* state,
    Vector3 positionECI,
    Vector3 velocityECI
) {
    if (state == NULL) {
        return STATUS_INVALID_PARAM;
    }
    
    /* Update projectile state */
    state->positionECI = positionECI;
    state->velocityECI = velocityECI;
    
    return STATUS_OK;
}

Status Guidance_Execute(
    GuidanceState* state,
    double timeStep
) {
    if (state == NULL) {
        return STATUS_INVALID_PARAM;
    }
    
    /* Convert current ECI states to local frame for guidance calculations */
    Vector3 positionLocal = ECIToLocal(
        state->positionECI, 
        state->originPosition, 
        state->targetPosition
    );
    
    Vector3 velocityLocal = ECIToLocalVelocity(
        state->velocityECI, 
        state->originPosition, 
        state->targetPosition
    );
    
    /* Convert target position to local frame */
    Vector3 targetLocal = ECIToLocal(
        state->targetPositionECI, 
        state->originPosition, 
        state->targetPosition
    );
    
    /* Vector to target and unit vector - match MATLAB implementation exactly */
    /* In MATLAB: R = [R_t_local(1); R_t_local(2); R_t_local(3) + r_loop3_start] - R_m_local; */
    Vector3 targetLocalWithOffset = Vector3_Create(
        targetLocal.a,
        targetLocal.b,
        targetLocal.c + state->simParams.terminalPhaseDistance
    );
    Vector3 relativeVector = Vector3_Subtract(targetLocalWithOffset, positionLocal);
    double distance = Vector3_Norm(relativeVector);
    Vector3 lineOfSightUnit = Vector3_ScalarDivide(relativeVector, distance);
    
    /* Calculate velocity magnitude and unit vector */
    double velocityMagnitude = Vector3_Norm(velocityLocal);
    Vector3 velocityUnit = Vector3_ScalarDivide(velocityLocal, velocityMagnitude);
    
    /* Calculate current theta and psi angles */
    double theta = asin(velocityLocal.c / velocityMagnitude);
    double psi = atan2(velocityLocal.b, velocityLocal.a);
    /* These variables are used for debugging/display purposes */
    (void)theta;
    (void)psi;
    
    /* Calculate look angle (sigma) */
    double sigma = acos(Vector3_DotProduct(velocityUnit, lineOfSightUnit));
    
    /* Initialize acceleration command */
    Vector3 accelerationCommandLocal;
    
    /* Check if we should switch to terminal phase - match MATLAB implementation */
    /* In MATLAB: if r <= r_loop3_start */
    if (distance <= state->simParams.terminalPhaseDistance && !state->terminalPhaseActive) {
        state->terminalPhaseActive = BOOL_TRUE;
        printf("Entered terminal phase at t=%.2f s, distance=%.2f m\n", 
               state->time, distance);
    }
    
    if (state->terminalPhaseActive) {
        /* Terminal phase - ballistic flight */
        accelerationCommandLocal = Vector3_Create(0.0, 0.0, 0.0);
    } else {
        /* Guided phase - calculate acceleration command in local frame */
        
        /* Calculate guidance commands - following MATLAB implementation */
        Vector3 omegaL = Vector3_ScalarDivide(
            Vector3_CrossProduct(velocityLocal, relativeVector), 
            distance * distance
        );
        
        /* Proportional navigation acceleration command (used in original MATLAB) */
        Vector3 AP = Vector3_CrossProduct(
            Vector3_ScalarMultiply(omegaL, state->params.N), 
            velocityLocal
        );
        (void)AP; /* Mark as intentionally unused */
        
        Vector3 crossProduct = Vector3_CrossProduct(lineOfSightUnit, velocityUnit);
        double crossNorm = Vector3_Norm(crossProduct);
        Vector3 kL;
        
        if (crossNorm < EPSILON) {
            /* Handle parallel vectors case */
            kL = Vector3_Create(0.0, 0.0, 1.0);
        } else {
            kL = Vector3_ScalarDivide(crossProduct, crossNorm);
        }
        
        double eta = sigma / (state->params.N - 1.0);
        double mu = sigma + eta;
        
        /* Calculate quaternion components */
        double w = cos(-mu / 2.0);
        double x_q = sin(-mu / 2.0) * kL.a;
        double y_q = sin(-mu / 2.0) * kL.b;
        double z_q = sin(-mu / 2.0) * kL.c;
        
        /* Create rotation matrix from quaternion */
        Matrix3 L_q = Matrix3_Create(
            w*w + x_q*x_q - y_q*y_q - z_q*z_q,     2.0*(x_q*y_q - w*z_q),     2.0*(x_q*z_q + w*y_q),
            2.0*(x_q*y_q + w*z_q),     w*w - x_q*x_q + y_q*y_q - z_q*z_q,     2.0*(y_q*z_q - w*x_q),
            2.0*(x_q*z_q - w*y_q),     2.0*(y_q*z_q + w*x_q),     w*w - x_q*x_q - y_q*y_q + z_q*z_q
        );
        
        /* Calculate predicted velocity direction */
        Vector3 e_p = Matrix3_MultiplyVector(L_q, velocityUnit);
        Vector3 V_p = Vector3_ScalarMultiply(e_p, velocityMagnitude);
        
        /* Calculate desired impact direction */
        Vector3 e_f = CalculateDesiredImpactDirection(state->thetaF, state->psiF);
        
        /* Calculate delta angle */
        double delta = acos(Vector3_DotProduct(e_f, e_p));
        
        /* Calculate l_f vector */
        Vector3 crossProduct2 = Vector3_CrossProduct(e_f, e_p);
        double crossNorm2 = Vector3_Norm(crossProduct2);
        Vector3 l_f;
        
        if (crossNorm2 < EPSILON) {
            /* Handle parallel vectors case */
            l_f = Vector3_Create(0.0, 1.0, 0.0);
        } else {
            l_f = Vector3_ScalarDivide(crossProduct2, crossNorm2);
        }
        
        /* Calculate time-to-go */
        double t_go = distance / (velocityMagnitude * cos(sigma));
        
        /* Calculate normalized sigma */
        double sigma_not = sigma / state->params.sigmaMax;
        double f_sigma_not = 1.0 - pow(fabs(sigma_not), state->params.d);
        
        /* Calculate delta_dot */
        double h_delta = state->params.a * SignedPower(state->params.m, delta) + 
                        state->params.b * SignedPower(state->params.n, delta);
        double delta_dot = (-state->params.K * f_sigma_not / t_go) * h_delta;
        
        /* Calculate omega_P */
        Vector3 omega_P = Vector3_ScalarMultiply(l_f, delta_dot);
        
        /* Calculate A_F */
        /* Final acceleration command from MATLAB implementation */
        Vector3 A_F = Vector3_CrossProduct(omega_P, V_p);
        (void)A_F; /* Mark as intentionally unused */
        
        /* Calculate l_m and l_P */
        Vector3 l_m = Vector3_CrossProduct(kL, velocityUnit);
        Vector3 l_P = Vector3_CrossProduct(kL, e_p);
        
        /* Calculate acceleration components */
        double a_p = (-state->params.N * velocityMagnitude * velocityMagnitude * sin(sigma)) / distance;
        
        double crossDot1 = Vector3_DotProduct(
            Vector3_CrossProduct(l_f, e_p), 
            l_P
        );
        
        double a_I_lm = ((state->params.N - 1.0) * state->params.K * f_sigma_not * 
                        velocityMagnitude * h_delta / t_go) * crossDot1;
        
        double crossDot2 = Vector3_DotProduct(
            Vector3_CrossProduct(l_f, e_p), 
            kL
        );
        
        double a_I_kl = (state->params.K * f_sigma_not * velocityMagnitude * sin(sigma) * 
                        h_delta / (t_go * sin(eta))) * crossDot2;
        
        /* Calculate final acceleration command */
        Vector3 lmScaled = Vector3_ScalarMultiply(l_m, a_p + a_I_lm);
        Vector3 klScaled = Vector3_ScalarMultiply(kL, a_I_kl);
        accelerationCommandLocal = Vector3_Add(lmScaled, klScaled);
    }
    
    /* Store local acceleration command */
    state->accelerationCommandLocal = accelerationCommandLocal;
    
    /* Convert guidance acceleration to ECI frame */
    state->accelerationCommandECI = LocalToECIVelocity(
        accelerationCommandLocal, 
        state->originPosition, 
        state->targetPosition
    );
    
    /* Calculate drag in ECI frame - match MATLAB implementation */
    /* In MATLAB: 
       Dx = D*(V_m_ECI(1))/v_ECI;
       Dy = D*(V_m_ECI(2))/v_ECI; 
       Dz = D*(V_m_ECI(3))/v_ECI;
       A_D_ECI = [Dx/m_c; Dy/m_c; Dz/m_c];
    */
    double velocityMagnitudeECI = Vector3_Norm(state->velocityECI);
    double dynamicPressure = 0.5 * state->envParams.airDensity * 
                            velocityMagnitudeECI * velocityMagnitudeECI;
    double dragForce = dynamicPressure * state->projectileParams.dragCoeff * 
                      state->projectileParams.refArea;
    
    /* Calculate drag components in direction of velocity */
    double dragX = dragForce * state->velocityECI.a / velocityMagnitudeECI;
    double dragY = dragForce * state->velocityECI.b / velocityMagnitudeECI;
    double dragZ = dragForce * state->velocityECI.c / velocityMagnitudeECI;
    
    /* Drag acceleration is in the opposite direction of velocity */
    state->dragAccelerationECI = Vector3_Create(
        -dragX / state->projectileParams.mass,
        -dragY / state->projectileParams.mass,
        -dragZ / state->projectileParams.mass
    );
    
    /* Calculate gravity in ECI frame */
    Vector3 gravityLocal = Vector3_Create(0.0, 0.0, -state->envParams.gravity);
    state->gravityAccelerationECI = LocalToECIVelocity(
        gravityLocal, 
        state->originPosition, 
        state->targetPosition
    );
    
    /* Update time and previous distance */
    state->time += timeStep;
    state->previousDistance = distance;
    
    return STATUS_OK;
}

Vector3 Guidance_GetAccelerationCommandECI(const GuidanceState* state) {
    if (state == NULL) {
        return Vector3_Create(0.0, 0.0, 0.0);
    }
    return state->accelerationCommandECI;
}

Vector3 Guidance_GetAccelerationCommandLocal(const GuidanceState* state) {
    if (state == NULL) {
        return Vector3_Create(0.0, 0.0, 0.0);
    }
    return state->accelerationCommandLocal;
}

Bool Guidance_IsTerminalPhaseActive(const GuidanceState* state) {
    if (state == NULL) {
        return BOOL_FALSE;
    }
    return state->terminalPhaseActive;
}

double Guidance_GetTimeToImpact(const GuidanceState* state) {
    if (state == NULL) {
        return 0.0;
    }
    
    /* Convert to local frame */
    Vector3 positionLocal = ECIToLocal(
        state->positionECI, 
        state->originPosition, 
        state->targetPosition
    );
    
    Vector3 velocityLocal = ECIToLocalVelocity(
        state->velocityECI, 
        state->originPosition, 
        state->targetPosition
    );
    
    Vector3 targetLocal = ECIToLocal(
        state->targetPositionECI, 
        state->originPosition, 
        state->targetPosition
    );
    
    /* Calculate relative vector and LOS */
    Vector3 relativeVector = Vector3_Subtract(targetLocal, positionLocal);
    double distance = Vector3_Norm(relativeVector);
    Vector3 lineOfSightUnit = Vector3_ScalarDivide(relativeVector, distance);
    
    /* Calculate velocity magnitude and unit vector */
    double velocityMagnitude = Vector3_Norm(velocityLocal);
    Vector3 velocityUnit = Vector3_ScalarDivide(velocityLocal, velocityMagnitude);
    
    /* Calculate look angle (sigma) */
    double sigma = acos(Vector3_DotProduct(velocityUnit, lineOfSightUnit));
    
    /* Calculate time-to-go */
    return distance / (velocityMagnitude * cos(sigma));
}

double Guidance_GetDistanceToTarget(const GuidanceState* state) {
    if (state == NULL) {
        return 0.0;
    }
    
    /* Convert to local frame */
    Vector3 positionLocal = ECIToLocal(
        state->positionECI, 
        state->originPosition, 
        state->targetPosition
    );
    
    Vector3 targetLocal = ECIToLocal(
        state->targetPositionECI, 
        state->originPosition, 
        state->targetPosition
    );
    
    /* Calculate distance - match MATLAB implementation */
    /* In MATLAB, the distance to target is always calculated with the terminal phase offset */
    Vector3 targetLocalWithOffset = Vector3_Create(
        targetLocal.a,
        targetLocal.b,
        targetLocal.c + state->simParams.terminalPhaseDistance
    );
    
    Vector3 relativeVector = Vector3_Subtract(targetLocalWithOffset, positionLocal);
    return Vector3_Norm(relativeVector);
}

void Guidance_PrintState(const GuidanceState* state) {
    if (state == NULL) {
        printf("Error: NULL guidance state\n");
        return;
    }
    
    printf("Time: %.2f s\n", state->time);
    printf("GUIDANCE COMMAND LOCAL: (%.2f, %.2f, %.2f) m/s^2\n", 
           state->accelerationCommandLocal.a, 
           state->accelerationCommandLocal.b, 
           state->accelerationCommandLocal.c);
    printf("GUIDANCE COMMAND ECI: (%.2f, %.2f, %.2f) m/s^2\n", 
           state->accelerationCommandECI.a, 
           state->accelerationCommandECI.b, 
           state->accelerationCommandECI.c);
    printf("DRAG IN ECI: (%.2f, %.2f, %.2f) m/s^2\n", 
           state->dragAccelerationECI.a, 
           state->dragAccelerationECI.b, 
           state->dragAccelerationECI.c);
    printf("GRAVITY IN ECI: (%.2f, %.2f, %.2f) m/s^2\n", 
           state->gravityAccelerationECI.a, 
           state->gravityAccelerationECI.b, 
           state->gravityAccelerationECI.c);
    printf("Projectile ECI position: (%.2f, %.2f, %.2f) m\n", 
           state->positionECI.a, 
           state->positionECI.b, 
           state->positionECI.c);
    printf("Projectile ECI velocity: (%.2f, %.2f, %.2f) m/s\n", 
           state->velocityECI.a, 
           state->velocityECI.b, 
           state->velocityECI.c);
    
    /* Calculate current theta and psi for display */
    Vector3 velocityLocal = ECIToLocalVelocity(
        state->velocityECI, 
        state->originPosition, 
        state->targetPosition
    );
    double velocityMagnitude = Vector3_Norm(velocityLocal);
    double theta = asin(velocityLocal.c / velocityMagnitude);
    double psi = atan2(velocityLocal.b, velocityLocal.a);
    
    printf("Theta: %.2f deg, Psi: %.2f deg\n", theta * RAD_TO_DEG, psi * RAD_TO_DEG);
    printf("Distance to target: %.2f m\n\n", Guidance_GetDistanceToTarget(state));
    
    if (state->terminalPhaseActive) {
        printf("Terminal phase active\n");
    }
}
