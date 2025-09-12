/******************************************************************************
 * ISA Flight Software
 * 
 * File: guidance.c
 * Description: Main entry point for the guidance system test


 Just a implementation of the guidance system  from matlab to C. not final
 for the final implementation, we will use the guidance system from the guidance folder.
 to see the updates check the test/guidance folder.
 *****************************************************************************/

#include <stdio.h>
#include <math.h>
#include "../../include/common/types.h"
#include "../../include/common/config.h"
#include "../../include/math/vector3.h"
#include "../../include/math/coordinate_transforms.h"
#include "guidance.h"

int main(void) {
    /* Test case parameters from MATLAB Test_OBG.m */
    
    /* Launch point coordinates (degrees) */
    GeodeticPosition originPosition = {
        .latitude = 8.529175797045042,
        .longitude = 76.88543289537785,
        .altitude = 0.0  /* meters */
    };
    
    /* Target coordinates (degrees) */
    GeodeticPosition targetPosition = {
        .latitude = 8.494073555483473,
        .longitude = 77.0021649331456,
        .altitude = 0.0  /* meters */
    };
    
    /* Projectile states at apogee (guidance phase start) in ECI frame */
    Vector3 initialPositionECI = Vector3_Create(
        1423879.23052435,      /* meters */
        6148086.22985647,      /* meters */
        937590.246010134       /* meters */
    );
    
    Vector3 initialVelocityECI = Vector3_Create(
        -298.618702873767,     /* m/s */
        83.5238474692337,      /* m/s */
        -92.5184011193217      /* m/s */
    );
    
    /* Desired impact angles (radians) */
    double thetaF = -90.0 * DEG_TO_RAD;  /* desired elevation at impact */
    double psiF = 0.0 * DEG_TO_RAD;      /* desired azimuth at impact */
    
    /* Initialize guidance state */
    GuidanceState guidanceState;
    Status status = Guidance_Initialize(
        &guidanceState,
        originPosition,
        targetPosition,
        initialPositionECI,
        initialVelocityECI,
        thetaF,
        psiF
    );
    
    if (status != STATUS_OK) {
        printf("Error initializing guidance: %d\n", status);
        return 1;
    }
    
    printf("GUIDANCE PHASE STARTED (APOGEE)\n");
    printf("Initial projectile ECI position: (%.2f, %.2f, %.2f) m\n", 
           initialPositionECI.a, initialPositionECI.b, initialPositionECI.c);
    printf("Initial projectile ECI velocity: (%.2f, %.2f, %.2f) m/s\n", 
           initialVelocityECI.a, initialVelocityECI.b, initialVelocityECI.c);
    printf("Target Distance: %.2f m\n\n", Guidance_GetDistanceToTarget(&guidanceState));
    
    /* Main simulation loop - match MATLAB implementation */
    double timeStep = guidanceState.simParams.timeStep;
    double maxSimulationTime = guidanceState.simParams.maxSimulationTime;
    double minVelocity = guidanceState.simParams.minProjectileVelocity;
    
    Vector3 positionECI = initialPositionECI;
    Vector3 velocityECI = initialVelocityECI;
    double time = 0.0;
    double previousDistance = Guidance_GetDistanceToTarget(&guidanceState);
    double currentDistance;
    
    while (time < maxSimulationTime && 
           (currentDistance = Guidance_GetDistanceToTarget(&guidanceState)) <= previousDistance &&
           Vector3_Norm(velocityECI) > minVelocity) {
        
        /* Update guidance state with current position and velocity */
        Guidance_UpdateState(&guidanceState, positionECI, velocityECI);
        
        /* Execute guidance algorithm */
        Guidance_Execute(&guidanceState, timeStep);
        
        /* Get acceleration commands */
        Vector3 accelerationCommandECI = Guidance_GetAccelerationCommandECI(&guidanceState);
        
        /* Add drag and gravity */
        Vector3 dragAccelerationECI = guidanceState.dragAccelerationECI;
        Vector3 gravityAccelerationECI = guidanceState.gravityAccelerationECI;
        
        /* Calculate total acceleration - MATLAB: A_total_ECI = A_M_ECI - A_D_ECI + grav_ECI */
        Vector3 totalAccelerationECI = Vector3_Add(
            Vector3_Add(accelerationCommandECI, gravityAccelerationECI),
            dragAccelerationECI  /* Note: dragAccelerationECI is already negative in our implementation */
        );
        
        /* Integrate acceleration to get velocity */
        velocityECI = Vector3_Add(velocityECI, Vector3_ScalarMultiply(totalAccelerationECI, timeStep));
        
        /* Integrate velocity to get position */
        positionECI = Vector3_Add(positionECI, Vector3_ScalarMultiply(velocityECI, timeStep));
        
        /* Print current state */
        Guidance_PrintState(&guidanceState);
        
        /* Store previous distance for comparison */
        previousDistance = currentDistance;
        
        /* Update time */
        time += timeStep;
        
        /* Check if we've passed the target (distance starts increasing) */
        if ((currentDistance = Guidance_GetDistanceToTarget(&guidanceState)) > previousDistance) {
            printf("Target impact at t=%.2f s\n", time);
            printf("Final ECI position: (%.2f, %.2f, %.2f) m\n", 
                   positionECI.a, positionECI.b, positionECI.c);
            printf("Final distance to target: %.2f m\n", previousDistance);
            break;
        }
    }
    
    /* Check why we exited the loop */
    if (time >= maxSimulationTime) {
        printf("Simulation time exceeded.\n");
    } else if (Vector3_Norm(velocityECI) <= minVelocity) {
        printf("Projectile velocity too low.\n");
    }
    
    return 0;
}
