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
    double accy,
    double accz,
    double rollRate,
    double pitchRate,
    double yawRate,
    double accyCommand,
    double acczCommand, DAPParameters dapParams, double timeStep
) 

{
    // Initialize
    float intr=0.0; // initialize
    double deltar = ComputeDeltaRollCommand(phi, rollRate, intr, dapParams, timeStep);
    double deltap = ComputeDeltaPitchCommand(accy, pitchRate, accyCommand, dapParams, timeStep);
    double deltay = ComputeDeltaYawCommand(accz, yawRate, acczCommand, dapParams, timeStep);
}

double ComputeDeltaRollCommand(double phi, double rollRate, double intr, DAPParameters dapParams, double timeStep) 
{
    double phiCommand =0.0; // assuming roll command is zero
    float phiErr = phiCommand - phi;
    float phiLimit;
    if (phiErr < dapParams.phiMinimum) 
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

double ComputeDeltaPitchCommand(double accy, double pitchRate, double accelerationYCommand, DAPParameters dapParams, double timeStep) 
{
    float qDot=0.0; // derivative of pitch rate 
    float accErrorPitch = accelerationYCommand - (dapParams.Kr_pitch*pitchRate)-((accy+dapParams.c*qDot)*dapParams.Ka_pitch);

    float deltaCommandPitch = accErrorPitch*dapParams.Ks_pitch;
}

double ComputeDeltaYawCommand(double accz, double yawRate, double accelerationZCommand, DAPParameters dapParams, double timeStep) 
{
    float rDot=0.0; // derivative of yaw rate 
    float accErrorYaw = accelerationZCommand - (dapParams.Kr_pitch*yawRate)-((accz+dapParams.c*rDot)*dapParams.Ka_pitch);

    float deltaCommandYaw = accErrorYaw*dapParams.Ks_pitch;
}