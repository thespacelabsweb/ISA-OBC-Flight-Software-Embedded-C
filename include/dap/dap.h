/******************************************************************************
 * ISA Flight Software
 * 
 * File: dap.h
 * Description: Digital Autopilot module interface
 *****************************************************************************/

#ifndef DAP_H
#define DAP_H

#include "../common/types.h"
#include "../math/vector3.h"

/**
 * @brief DAP module interface for flight control
 */

/**
 * @brief Execute the Digital Autopilot control algorithm
 * 
 * @param phi Current roll angle in radians
 * @param accelerationY Current Y-axis acceleration in m/s^2
 * @param accelerationZ Current Z-axis acceleration in m/s^2
 * @param rollRate Current roll rate in rad/s
 * @param pitchRate Current pitch rate in rad/s
 * @param yawRate Current yaw rate in rad/s
 * @param accelerationYCommand Commanded Y-axis acceleration in m/s^2
 * @param accelerationZCommand Commanded Z-axis acceleration in m/s^2
 * @param dapParams DAP controller parameters
 * @param timeStep Time step in seconds
 * @return Vector3 Control commands for roll, pitch, and yaw
 */
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
);

/**
 * @brief Compute roll control command
 * 
 * @param phi Current roll angle in radians
 * @param rollRate Current roll rate in rad/s
 * @param intr Current integrator value
 * @param dapParams DAP controller parameters
 * @param dt Time step in seconds
 * @return double Roll control command
 */
double ComputeDeltaRollCommand(
    double phi, 
    double rollRate, 
    double intr, 
    DAPParameters dapParams, 
    double dt
);

/**
 * @brief Compute pitch control command
 * 
 * @param accelerationY Current Y-axis acceleration in m/s^2
 * @param pitchRate Current pitch rate in rad/s
 * @param accelerationYCommand Commanded Y-axis acceleration in m/s^2
 * @param dapParams DAP controller parameters
 * @param dt Time step in seconds
 * @return double Pitch control command
 */
double ComputeDeltaPitchCommand(
    double accelerationY, 
    double pitchRate, 
    double accelerationYCommand, 
    DAPParameters dapParams, 
    double dt
);

/**
 * @brief Compute yaw control command
 * 
 * @param accelerationZ Current Z-axis acceleration in m/s^2
 * @param yawRate Current yaw rate in rad/s
 * @param accelerationZCommand Commanded Z-axis acceleration in m/s^2
 * @param dapParams DAP controller parameters
 * @param dt Time step in seconds
 * @return double Yaw control command
 */
double ComputeDeltaYawCommand(
    double accelerationZ, 
    double yawRate, 
    double accelerationZCommand, 
    DAPParameters dapParams, 
    double dt
);

#endif /* DAP_H */
