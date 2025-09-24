/******************************************************************************
 * ISA Flight Software - Guidance Module Test Harness
 * @file guidance_test.c
 * @brief Test harness for guidance module using exact MATLAB Test_OBG.m parameters
 * @details Simulates the complete guidance trajectory from apogee to impact
 * @author Flight Software Team, Spacelabs (MATLAB test case by Aerospace Team, Spacelabs)
 * @date 2025
 * @version 1.0
 *
 * Test Case: Exact reproduction of Test_OBG.m MATLAB simulation
 * - Launch from (8.529°, 76.885°, 0m) to target at (8.494°, 77.002°, 0m)
 * - Initial conditions at apogee (guidance phase start)
 * - Desired impact angle: -90° elevation, 0° azimuth (vertical impact)
 * - Real-time simulation with 10ms time steps
 ******************************************************************************/

#include <stdio.h>
#include <stdlib.h>  /* For exit() */
#include <unistd.h>  /* For usleep() */
#include <math.h>    /* For fabs() */
#include <stdbool.h> /* For bool type */

#include "guidance.h"  /* Guidance module interface */
#include "vector3.h"   /* Vector operations for physics integration */
#include "coordinate_transforms.h"  /* Coordinate transformations if needed */

/* Test case parameters - exact copy from MATLAB Test_OBG.m */
#define ORIGIN_LATITUDE     8.529175797045042f   /* Launch latitude (degrees) */
#define ORIGIN_LONGITUDE    76.88543289537785f   /* Launch longitude (degrees) */
#define ORIGIN_ALTITUDE     0.0f                 /* Launch altitude (meters) */

#define TARGET_LATITUDE     8.494073555483473f   /* Target latitude (degrees) */
#define TARGET_LONGITUDE    77.0021649331456f    /* Target longitude (degrees) */
#define TARGET_ALTITUDE     0.0f                 /* Target altitude (meters) */

#define INITIAL_POSITION_ECI_X   1423879.23052435f   /* Initial X position (meters) */
#define INITIAL_POSITION_ECI_Y   6148086.22985647f   /* Initial Y position (meters) */
#define INITIAL_POSITION_ECI_Z   937590.246010134f   /* Initial Z position (meters) */

#define INITIAL_VELOCITY_ECI_X   -298.618702873767f   /* Initial X velocity (m/s) */
#define INITIAL_VELOCITY_ECI_Y   83.5238474692337f    /* Initial Y velocity (m/s) */
#define INITIAL_VELOCITY_ECI_Z   -92.5184011193217f   /* Initial Z velocity (m/s) */

#define DESIRED_ELEVATION_ANGLE  -90.0f  /* Desired elevation at impact (degrees) */
#define DESIRED_AZIMUTH_ANGLE    0.0f    /* Desired azimuth at impact (degrees) */

/* Simulation parameters - match MATLAB implementation */
#define SIMULATION_TIME_STEP_MS  10      /* Minor cycle = 10ms (100 Hz) */
#define SIMULATION_DURATION_S    60.0f   /* Maximum simulation time (seconds) */

/******************************************************************************
 * @brief Main test function - replicates MATLAB Test_OBG.m exactly
 ******************************************************************************/
int main(void) {
    printf("--- ISA Flight Software: Guidance Module Test ---\n");
    printf("Test Case: MATLAB Test_OBG.m trajectory simulation\n");
    printf("Launch: (%.6f°, %.6f°, %.1fm) → Target: (%.6f°, %.6f°, %.1fm)\n",
           ORIGIN_LATITUDE, ORIGIN_LONGITUDE, ORIGIN_ALTITUDE,
           TARGET_LATITUDE, TARGET_LONGITUDE, TARGET_ALTITUDE);
    printf("Desired Impact: Elevation=%.1f°, Azimuth=%.1f°\n\n",
           DESIRED_ELEVATION_ANGLE, DESIRED_AZIMUTH_ANGLE);

    /* 1. Initialize guidance state and output structures */
    GuidanceState_t guidanceState;
    GuidanceOutput_t guidanceOutput;

    /* 2. Initialize the guidance module */
    GuidanceError_t initResult = guidanceInit(&guidanceState);
    if (initResult != GUIDANCE_SUCCESS) {
        printf("ERROR: Failed to initialize guidance module (error %d)\n", initResult);
        exit(1);
    }
    printf("✓ Guidance module initialized successfully\n");

    /* 3. Configure launch origin */
    GuidanceError_t originResult = guidanceSetOrigin(&guidanceState,
                                                     ORIGIN_LATITUDE,
                                                     ORIGIN_LONGITUDE,
                                                     ORIGIN_ALTITUDE);
    if (originResult != GUIDANCE_SUCCESS) {
        printf("ERROR: Failed to set origin (error %d)\n", originResult);
        exit(1);
    }
    printf("✓ Launch origin configured: (%.6f°, %.6f°, %.1fm)\n",
           ORIGIN_LATITUDE, ORIGIN_LONGITUDE, ORIGIN_ALTITUDE);

    /* 4. Configure target position */
    GuidanceError_t targetResult = guidanceSetTarget(&guidanceState,
                                                     TARGET_LATITUDE,
                                                     TARGET_LONGITUDE,
                                                     TARGET_ALTITUDE);
    if (targetResult != GUIDANCE_SUCCESS) {
        printf("ERROR: Failed to set target (error %d)\n", targetResult);
        exit(1);
    }
    printf("✓ Target configured: (%.6f°, %.6f°, %.1fm)\n",
           TARGET_LATITUDE, TARGET_LONGITUDE, TARGET_ALTITUDE);

    /* 5. Set desired impact angles */
    GuidanceError_t anglesResult = guidanceSetImpactAngles(&guidanceState,
                                                           DESIRED_ELEVATION_ANGLE,
                                                           DESIRED_AZIMUTH_ANGLE);
    if (anglesResult != GUIDANCE_SUCCESS) {
        printf("ERROR: Failed to set impact angles (error %d)\n", anglesResult);
        exit(1);
    }
    printf("✓ Desired impact angles: Elevation=%.1f°, Azimuth=%.1f°\n",
           DESIRED_ELEVATION_ANGLE, DESIRED_AZIMUTH_ANGLE);

    /* 6. Set initial projectile state (at apogee, guidance phase start) */
    Vector3f_t initialPositionECI = Vector3f_Create(
        INITIAL_POSITION_ECI_X,
        INITIAL_POSITION_ECI_Y,
        INITIAL_POSITION_ECI_Z
    );

    Vector3f_t initialVelocityECI = Vector3f_Create(
        INITIAL_VELOCITY_ECI_X,
        INITIAL_VELOCITY_ECI_Y,
        INITIAL_VELOCITY_ECI_Z
    );

    GuidanceError_t updateResult = guidanceUpdateState(&guidanceState,
                                                       initialPositionECI,
                                                       initialVelocityECI);
    if (updateResult != GUIDANCE_SUCCESS) {
        printf("ERROR: Failed to update initial state (error %d)\n", updateResult);
        exit(1);
    }

    printf("✓ Initial state set (apogee conditions)\n");
    printf("  Position ECI: (%.2f, %.2f, %.2f) m\n",
           initialPositionECI.x, initialPositionECI.y, initialPositionECI.z);
    printf("  Velocity ECI: (%.2f, %.2f, %.2f) m/s\n",
           initialVelocityECI.x, initialVelocityECI.y, initialVelocityECI.z);

    /* 7. Initialize simulation variables */
    float timeStep = (float)SIMULATION_TIME_STEP_MS / 1000.0f;  /* Convert ms to seconds */
    float currentTime = 0.0f;  /* Start from beginning to match MATLAB t=0 */
    
    /* Current state (will be updated each iteration) */
    Vector3f_t currentPositionECI = initialPositionECI;
    Vector3f_t currentVelocityECI = initialVelocityECI;
    
    /* Initialize previousDistance to actual distance WITHOUT offset - exact MATLAB bug reproduction */
    /* MATLAB line 52-53: r = norm(R_t_local - R_m_local); r_prev = r; */
    Vector3f_t positionLocal, targetLocal;
    guidanceUpdateState(&guidanceState, currentPositionECI, currentVelocityECI);
    CoordinateError_t coordResult = ECIToLocal(currentPositionECI, guidanceState.originPosition, guidanceState.targetPosition, &positionLocal);
    if (coordResult != COORD_SUCCESS) {
        printf("ERROR: Initial coordinate transformation failed\n");
        exit(1);
    }
    coordResult = ECIToLocal(guidanceState.targetPositionECI, guidanceState.originPosition, guidanceState.targetPosition, &targetLocal);
    if (coordResult != COORD_SUCCESS) {
        printf("ERROR: Target coordinate transformation failed\n");
        exit(1);
    }
    Vector3f_t initialTargetVector = Vector3f_Subtract(targetLocal, positionLocal);
    float previousDistance = Vector3f_Norm(initialTargetVector);  /* Actual distance WITHOUT offset - MATLAB r_prev initialization bug */

    printf("\n--- GUIDANCE PHASE STARTED (APOGEE) ---\n");
    printf("Time Step: %.3f s, Max Time: %.1f s\n", timeStep, SIMULATION_DURATION_S);
    printf("Initial Distance to Target: %.2f m\n\n", previousDistance);

    /* 8. Main simulation loop - replicates MATLAB while loop exactly */
    int totalCycles = (int)(SIMULATION_DURATION_S / timeStep);
    float currentDistance;

    for (int cycle = 0; cycle < totalCycles; cycle++) {
        /* Update guidance state with current position/velocity */
        guidanceUpdateState(&guidanceState, currentPositionECI, currentVelocityECI);

        /* Print current state BEFORE guidance execution to show correct time (matches MATLAB output format) */
        guidancePrintState(&guidanceState);

        /* Execute guidance algorithm */
        GuidanceError_t executeResult = guidanceExecute(&guidanceState, timeStep, &guidanceOutput);
        if (executeResult != GUIDANCE_SUCCESS) {
            printf("ERROR: Guidance execution failed at t=%.2f s (error %d)\n",
                   currentTime, executeResult);
            exit(1);
        }

        /* Get total acceleration (guidance + gravity - drag) - match MATLAB: A_total_ECI = A_M_ECI - A_D_ECI + grav_ECI */
        Vector3f_t totalAccelerationECI = Vector3f_Add(
            Vector3f_Subtract(guidanceOutput.accelerationCommandECI,
                             guidanceState.dragAccelerationECI),
            guidanceState.gravityAccelerationECI
        );

        /* Integrate equations of motion */
        /* v = v + a*dt */
        currentVelocityECI = Vector3f_Add(
            currentVelocityECI,
            Vector3f_Scale(timeStep, totalAccelerationECI)
        );

        /* r = r + v*dt */
        currentPositionECI = Vector3f_Add(
            currentPositionECI,
            Vector3f_Scale(timeStep, currentVelocityECI)
        );

        /* Update simulation variables */
        currentTime += timeStep;
        currentDistance = guidanceOutput.distanceToTarget;

        /* Check termination condition: distance starts increasing (passed target) */
        /* MATLAB bug reproduction: currentDistance (guidance distance WITH offset) vs previousDistance */
        /* First iteration: previousDistance = actual distance WITHOUT offset, currentDistance = guidance distance WITH offset */
        /* Subsequent iterations: both are guidance distances WITH offset */
        if (currentDistance > previousDistance) {
            printf("🎯 TARGET IMPACT at t=%.2f s\n", currentTime);
            printf("Final ECI position: (%.2f, %.2f, %.2f) m\n",
                   currentPositionECI.x, currentPositionECI.y, currentPositionECI.z);
            printf("Final distance to target: %.2f m\n", previousDistance);
            printf("Impact achieved! ✓\n");
            break;
        }

        previousDistance = currentDistance;

        /* Sleep to simulate real-time execution (optional - comment out for faster testing) */
        usleep(SIMULATION_TIME_STEP_MS * 1000);
    }

    /* Check why simulation ended */
    if (currentTime >= SIMULATION_DURATION_S) {
        printf("⚠️  Simulation time exceeded (%.1f s) - no impact detected\n", SIMULATION_DURATION_S);
        printf("Final distance to target: %.2f m\n", currentDistance);
    }

    printf("\n--- Guidance Module Test Complete ---\n");
    printf("Compare results with MATLAB Test_OBG.m output\n");

    return 0;
}

