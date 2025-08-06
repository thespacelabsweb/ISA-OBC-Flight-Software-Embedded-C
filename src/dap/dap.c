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
#include "../../include/math/vector3.h"

/* Module-level variables for rate derivative calculations */
static double previousPitchrate = 0.0;
static double previousYawrate = 0.0;
static double rollIntegrator = 0.0;

Vector3 DAP_Execute(
    double phi,
    double accelerationY,
    double accelerationZ,
    double rollRate,
    double pitchRate,
    double yawRate,
    double accelerationYCommand,
    double accelerationZCommand, 
    DAPParameters dapParams, 
    double timeStep
)
{
    double deltar = ComputeDeltaRollCommand(phi, rollRate, rollIntegrator, dapParams, timeStep);
    double deltap = ComputeDeltaPitchCommand(accelerationY, pitchRate, accelerationYCommand, dapParams, timeStep);
    double deltay = ComputeDeltaYawCommand(accelerationZ, yawRate, accelerationZCommand, dapParams, timeStep);
    
    /* Store roll integrator value for next iteration */
    rollIntegrator = deltar;
    
    Vector3 output = Vector3_Create(deltar, deltap, deltay);
    return output;
}

double ComputeDeltaRollCommand(double phi, double rollRate, double intr, DAPParameters dapParams, double timeStep) 
{
    double phiCommand = 0.0; /* assuming roll command is zero */ 
    double phiErr = phiCommand - phi;
    double phiLimit;
    
    /* Apply roll angle limits */
    if (phiErr < dapParams.phiMinimum) 
    {
        phiLimit = dapParams.phiMinimum;
    } 
    else if (phiErr > dapParams.phiMaximum) 
    {
        phiLimit = dapParams.phiMaximum;
    } 
    else 
    {
        phiLimit = phiErr;  
    }
    
    /* Calculate proportional term */
    double out = phiLimit - dapParams.Kr_roll * rollRate;
    
    /* Update integrator with anti-windup */
    intr += out * timeStep;
    if (intr > dapParams.IntegratorMaximum) 
    {
        intr = dapParams.IntegratorMaximum;  
    } 
    else if (intr < dapParams.IntegratorMinimum) 
    {
        intr = dapParams.IntegratorMinimum;  
    }
    
    /* Calculate final control command */
    double deltaCommandRoll = dapParams.Kp_roll * out + intr * dapParams.Ki_roll;
    return deltaCommandRoll;
}

double ComputeDeltaPitchCommand(double accelerationY, double pitchRate, double accelerationYCommand, DAPParameters dapParams, double timeStep) 
{
    double currentPitchrate = pitchRate;
    
    /* Calculate pitch rate derivative */
    double qDot = (currentPitchrate - previousPitchrate) / timeStep;
    
    /* Calculate acceleration error */
    double accErrorPitch = accelerationYCommand - (dapParams.Kr_pitch * pitchRate) - 
                          ((accelerationY + dapParams.c * qDot) * dapParams.Ka_pitch);
    
    /* Update previous pitch rate for next iteration */
    previousPitchrate = currentPitchrate;
    
    /* Calculate final control command */
    double deltaCommandPitch = accErrorPitch * dapParams.Ks_pitch;
    return deltaCommandPitch;
}

double ComputeDeltaYawCommand(double accelerationZ, double yawRate, double accelerationZCommand, DAPParameters dapParams, double timeStep) 
{
    double currentYawrate = yawRate;
    
    /* Calculate yaw rate derivative */
    double rDot = (currentYawrate - previousYawrate) / timeStep;
    
    /* Calculate acceleration error */
    double accErrorYaw = accelerationZCommand - (dapParams.Kr_yaw * yawRate) - 
                        ((accelerationZ + dapParams.c * rDot) * dapParams.Ka_yaw);
    
    /* Update previous yaw rate for next iteration */
    previousYawrate = currentYawrate;
    
    /* Calculate final control command */
    double deltaCommandYaw = accErrorYaw * dapParams.Ks_yaw;
    return deltaCommandYaw;
}