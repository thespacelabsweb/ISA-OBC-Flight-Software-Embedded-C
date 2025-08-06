/******************************************************************************
 * ISA Flight Software
 * 
 * File: dap.h
 * Description: Digital Autopilot module interface (placeholder)
 *****************************************************************************/

#ifndef DAP_H
#define DAP_H

#include "../common/types.h"

/**
 * @brief DAP module 
 */

double previousPitchrate = 0.0;
double previousYawrate =0.0;
Status DAP_Execute(
    double phi,
    double accelerationY,
    double accelerationZ,
    double rollRate,
    double pitchRate,
    double yawRate,
    double accelerationYCommand,
    double accelerationZCommand, DAPParameters dapParams, double timeStep
) ;
double ComputeDeltaRollCommand(double phi, double rollRate, double intr, DAPParameters dapParams, double dt); 
double ComputeDeltaPitchCommand(double accelerationY, double pitchRate, double accelerationYCommand, DAPParameters dapParams, double dt); 
double ComputeDeltaYawCommand(double accelerationZ, double yawRate, double accelerationZCommand, DAPParameters dapParams, double dt);


#endif /* DAP_H */
