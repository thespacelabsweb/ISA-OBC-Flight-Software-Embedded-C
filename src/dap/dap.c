/******************************************************************************
 * ISA Flight Software
 * 
 * File: dap.c
 * Description: Digital Autopilot module implementation 
 *****************************************************************************/


#include <stdio.h>
#include <math.h>
#include "../../include/common/config.h"
#include "../../include/dap/dap.h"

Status DAP_Execute(
    double phi,
    double accelerationY,
    double accelerationZ,
    double rollRate,
    double pitchRate,
    double yawRate,
    double accelerationYCommand,
    double accelerationZCommand, DAPParameters dapParams, double timeStep
) 

{
    // Initialize
    float intr=0.0; // initialize
    double deltar = ComputeDeltaRollCommand(phi, rollRate, intr, dapParams, timeStep);
    double deltap = ComputeDeltaPitchCommand(accelerationY, pitchRate, accelerationYCommand, dapParams, timeStep);
    double deltay = ComputeDeltaYawCommand(accelerationZ, yawRate, accelerationZCommand, dapParams, timeStep);
}

double ComputeDeltaRollCommand(double phi, double rollRate, double intr, DAPParameters dapParams, double timeStep) 
{
    double phiCommand =0.0; /* assuming roll command is zero */ 
    float phiErr = phiCommand - phi;
    float phiLimit;
    if (phiErr < dapParams.phiMinimum) /* rate limiter */
    {
        phiLimit = dapParams.phiMinimum;
    } else if (phiErr > dapParams.phiMaximum) 
    {
        phiLimit = dapParams.phiMaximum;
    } else {
        phiLimit = phiErr;  
    }
    float out = phiLimit - dapParams.Kr_roll * rollRate;
    intr +=out*timeStep;
    if (intr > dapParams.IntegratorMaximum) 
    {
        intr = dapParams.IntegratorMaximum;  
    } 
    else if (intr < dapParams.IntegratorMinimum) 
    {
        intr = dapParams.IntegratorMinimum;  
    }
    float deltaCommandRoll = dapParams.Kp_roll * out + intr*dapParams.Ki_roll;
    return deltaCommandRoll;
}

double ComputeDeltaPitchCommand(double accelerationY, double pitchRate, double accelerationYCommand, DAPParameters dapParams, double timeStep) 
{
    float currentPitchrate = pitchRate; /* current pitch rate */ 
    float qDot=(currentPitchrate-previousPitchrate)/timeStep; /* derivative of pitch rate */ 
    float accErrorPitch = accelerationYCommand - (dapParams.Kr_pitch*pitchRate)-((accelerationY+dapParams.c*qDot)*dapParams.Ka_pitch);
    previousPitchrate = currentPitchrate; /* update previous pitch rate */ 
    float deltaCommandPitch = accErrorPitch*dapParams.Ks_pitch;
}

double ComputeDeltaYawCommand(double accelerationZ, double yawRate, double accelerationZCommand, DAPParameters dapParams, double timeStep) 
{
    float currentYawrate = yawRate; /* current yaw rate */ 
    float rDot=(currentYawrate-previousYawrate)/timeStep; /* derivative of pitch rate */
    float accErrorYaw = accelerationZCommand - (dapParams.Kr_yaw*yawRate)-((accelerationZ+dapParams.c*rDot)*dapParams.Ka_yaw);
    previousYawrate = currentYawrate; /* update previous yaw rate */ 
    float deltaCommandYaw = accErrorYaw*dapParams.Ks_yaw;
}