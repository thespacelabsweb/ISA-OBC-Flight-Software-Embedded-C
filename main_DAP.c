/******************************************************************************
 * ISA Digital Autopilot (DAP) Module
 * 
 * File: main_DAP.c
 * Description: Main for DAP system test
 *****************************************************************************/

#include <stdio.h>
#include <math.h>
#include "include/common/types.h"
#include "include/common/config.h"
#include "include/dap/dap.h"
#include "include/math/vector3.h"

int main(void) {
    /* Test input values */
    float phi = 1.0;                /* Roll angle in radians */
    float accelerationY = 0.1;      /* Y-axis (pitch) acceleration in m/s^2 */
    float accelerationZ = 0.1;      /* Z-axis (yaw) acceleration in m/s^2 */
    float rollRate = 20.0 * PI / 180.0;  /* Roll rate in rad/s */
    float pitchRate = 20.0; /* Pitch rate in rad/s */
    float yawRate = 20.0 ;   /* Yaw rate in rad/s */
    float accelerationYCommand = 0.4; /* Commanded Y-axis acceleration */
    float accelerationZCommand = 0.4; /* Commanded Z-axis acceleration */
    float timeStep = 0.1;           /* Time step in seconds */
    
    /* Use default DAP parameters */
    DAPParameters dapParams = DAP_DEFAULT;
    
    /* Execute DAP algorithm */
    Vector3 dapOutput = DAP_Execute(
        phi,
        accelerationY,
        accelerationZ,
        rollRate,
        pitchRate,
        yawRate,
        accelerationYCommand,
        accelerationZCommand,
        dapParams,
        timeStep
    );
    
    /* Print results */
    printf("DAP Output: (%.4f, %.4f, %.4f)\n", dapOutput.a, dapOutput.b, dapOutput.c);
    printf("  Roll Command: %.4f\n", dapOutput.a);
    printf("  Pitch Command: %.4f\n", dapOutput.b);
    printf("  Yaw Command: %.4f\n", dapOutput.c);
    
    return 0;
}
