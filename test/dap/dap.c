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
static float previousPitchrate = 0.0;
static float previousYawrate = 0.0;
static float rollIntegrator = 0.0;

Vector3 DAP_Execute(
    float phi,
    float accelerationY,
    float accelerationZ,
    float rollRate,
    float pitchRate,
    float yawRate,
    float accelerationYCommand,
    float accelerationZCommand, 
    DAPParameters dapParams, 
    float timeStep
)
{
    float deltar = ComputeDeltaRollCommand(phi, rollRate, rollIntegrator, dapParams, timeStep);
    float deltap = ComputeDeltaPitchCommand(accelerationY, pitchRate, accelerationYCommand, dapParams, timeStep);
    float deltay = ComputeDeltaYawCommand(accelerationZ, yawRate, accelerationZCommand, dapParams, timeStep);
    
    /* Store roll integrator value for next iteration */
    rollIntegrator = deltar;
    
    Vector3 output = Vector3_Create(deltar, deltap, deltay);
    return output;
}

float ComputeDeltaRollCommand(float phi, float rollRate, float intr, DAPParameters dapParams, float timeStep) 
{
    float phiCommand = 0.0; /* assuming roll command is zero */ 
    float phiErr = phiCommand - phi;
    float phiLimit;
    
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
    float out = phiLimit - dapParams.Kr_roll * rollRate;
    
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
    float deltaCommandRoll = dapParams.Kp_roll * out + intr * dapParams.Ki_roll;
    return deltaCommandRoll;
}

float ComputeDeltaPitchCommand(float accelerationY, float pitchRate, float accelerationYCommand, DAPParameters dapParams, float timeStep) 
{
    float currentPitchrate = pitchRate;
    
    /* Calculate pitch rate derivative */
    float qDot = (currentPitchrate - previousPitchrate) / timeStep;
    
    /* Calculate acceleration error */
    float accErrorPitch = accelerationYCommand - (dapParams.Kr_pitch * pitchRate) - 
                          ((accelerationY + dapParams.c * qDot) * dapParams.Ka_pitch);
    
    /* Update previous pitch rate for next iteration */
    previousPitchrate = currentPitchrate;
    
    /* Calculate final control command */
    float deltaCommandPitch = accErrorPitch * dapParams.Ks_pitch;
    return deltaCommandPitch;
}

float ComputeDeltaYawCommand(float accelerationZ, float yawRate, float accelerationZCommand, DAPParameters dapParams, float timeStep) 
{
    float currentYawrate = yawRate;
    
    /* Calculate yaw rate derivative */
    float rDot = (currentYawrate - previousYawrate) / timeStep;
    
    /* Calculate acceleration error */
    float accErrorYaw = accelerationZCommand - (dapParams.Kr_yaw * yawRate) - 
                        ((accelerationZ + dapParams.c * rDot) * dapParams.Ka_yaw);
    
    /* Update previous yaw rate for next iteration */
    previousYawrate = currentYawrate;
    
    /* Calculate final control command */
    float deltaCommandYaw = accErrorYaw * dapParams.Ks_yaw;
    return deltaCommandYaw;
}