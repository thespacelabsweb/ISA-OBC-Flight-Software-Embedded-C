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

double prevrDot = 0.0;
Status DAP_Execute(
    double phi,
    double accy,
    double accz,
    double rollRate,
    double pitchRate,
    double yawRate,
    double accyCommand,
    double acczCommand, DAPParameters dapParams, double timeStep
) ;
double ComputeDeltaRollCommand(double phi, double rollRate, double intr, DAPParameters dapParams, double dt); 
double ComputeDeltaPitchCommand(double accy, double pitchRate, double accelerationYCommand, DAPParameters dapParams, double dt); 
double ComputeDeltaYawCommand(double accz, double yawRate, double accelerationZCommand, DAPParameters dapParams, double dt);


#endif /* DAP_H */
