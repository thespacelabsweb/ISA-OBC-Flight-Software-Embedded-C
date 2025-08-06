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
    double phi=2.0;
    double accelerationY=0.1;
    double accelerationZ=0.1;
    double rollRate=2*2*PI;
    double pitchRate=20*PI/180;
    double yawRate=20*PI/180;
    double accelerationYCommand=0;
    double accelerationZCommand=0;
    DAPParameters dapParams;
    double timeStep = 0.1; // Example time step, adjust as needed

    Vector3 DAPOutput = DAP_Execute(
     phi,
     accelerationY,
     accelerationZ,
     rollRate,
     pitchRate,
     yawRate,
     accelerationYCommand,
     accelerationZCommand,  dapParams,  timeStep);


    printf("DAP Output: (%.2f, %.2f, %.2f)\n", DAPOutput.a, DAPOutput.b, DAPOutput.c);
    
    return 0;
}
