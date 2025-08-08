/******************************************************************************
 * ISA Flight Software
 * 
 * File: config.h
 * Description: Configuration parameters for the guidance system
 *****************************************************************************/

#ifndef CONFIG_H
#define CONFIG_H

#include "types.h"

/**
 * @brief WGS84 ellipsoid default parameters
 */
static const WGS84Parameters WGS84_DEFAULT = {
    .semiMajorAxis = 6378137.0,         /* Semi-major axis in meters */
    .flattening = 1.0 / 298.257223563,  /* Flattening */
    .eccentricity2 = 0.0066943799901    /* Calculated as 2*f - f*f */
};

/**
 * @brief Default guidance parameters (matching MATLAB implementation)
 */
static const GuidanceParameters GUIDANCE_DEFAULT = {
    .a = 0.100001,              /* Impact angle control parameter a */
    .b = 0.100001,              /* Impact angle control parameter b */
    .m = 0.90001,               /* Impact angle control parameter m */
    .n = 1.30001,               /* Impact angle control parameter n */
    .d = 7.5,                   /* Impact angle control parameter d */
    .K = 21.00000001,           /* Impact angle control parameter K */
    .sigmaMax = 73.0 * DEG_TO_RAD, /* Maximum look angle in radians */
    .N = 3.0                    /* Guidance gain */
};

/**
 * @brief Default projectile parameters (matching MATLAB implementation)
 */
static const ProjectileParameters PROJECTILE_DEFAULT = {
    .mass = 47.0,               /* Mass in kg */
    .diameter = 0.155,          /* Diameter in meters */
    .dragCoeff = 0.2808,        /* Drag coefficient */
    .refArea = 0.0                /* Will be calculated as (pi*diameter^2)/4 */
};

/**
 * @brief Default environmental parameters (matching MATLAB implementation)
 */
static const EnvironmentParameters ENVIRONMENT_DEFAULT = {
    .gravity = 9.8,             /* Gravity acceleration in m/s^2 */
    .airDensity = 1.225         /* Air density in kg/m^3 */
};

/**
 * @brief Default simulation parameters (matching MATLAB implementation)
 */
static const SimulationParameters SIMULATION_DEFAULT = {
    .timeStep = 0.01,           /* Time step in seconds */
    .maxSimulationTime = 200.0, /* Maximum simulation time in seconds */
    .minProjectileVelocity = 1.0, /* Minimum velocity threshold in m/s */
    .terminalPhaseDistance = 100.0 /* Distance to switch to terminal phase in meters */
};

static const DAPParameters DAP_DEFAULT = {
    /* Roll controller constants */  
   .Kp_roll =  11.8899,  /* Proportional gain for roll */    
   .Ki_roll = 2.0, /* Integrator gain for roll */       
   .Kr_roll = 0.0841,  /* Rate gain for roll */  
   .IntegratorMinimum= -0.01745, /* Integrator min limit - roll */ 
   .IntegratorMaximum = 0.01745, /* Integrator max limit - roll */ 
   .phiMinimum= -0.014677,  /* Roll min limit */ 
   .phiMaximum=  0.014677, /* Roll max limit */ 
   /* Pitch controller constants */
   .Ks_pitch =  0.2497,  /* Proportional gain for pitch */
   .Ka_pitch = 1.3629, /* Acceleration gain for pitch */
   .Kr_pitch = 3.4342,  /* Rate gain for pitch */
   /* Yaw controller constants */
   .Ks_yaw =  0.2497,  /* Proportional gain for Yaw */
   .Ka_yaw = 1.3629, /* Acceleration gain for Yaw */
   .Kr_yaw = 3.4342,  /* Rate gain for Yaw */
   .c= 0.0, /* Distance between IMU and CG */
};

#endif /* CONFIG_H */
