/******************************************************************************
 * ISA Flight Software
 * 
 * File: sequencer.h
 * Description: Sequencer module interface
 *****************************************************************************/

#ifndef SEQUENCER_H
#define SEQUENCER_H

#include "../common/types.h"

/**
 * @brief Sequencer states
 */
typedef enum {
    SEQ_STATE_INIT,           /* Initial state after OBC reset (T0) */
    SEQ_STATE_T0_WINDOW,      /* Monitoring window after T0 to check roll rate < 7rps */
    SEQ_STATE_T1_REACHED,     /* Canard deployment state when roll rate criteria met */
    SEQ_STATE_T1_WINDOW,      /* Monitoring window after T1 to check roll rate < 2rps */
    SEQ_STATE_T2_REACHED,     /* Roll control activation state when roll rate criteria met */
    SEQ_STATE_PITCH_YAW_ON,   /* State after T2+Δt4 when pitch/yaw control is activated */
    SEQ_STATE_GUIDANCE_ON,    /* State after T2+2s when guidance is activated */
    SEQ_STATE_T2_WINDOW,      /* Monitoring window after guidance activation to check for GT3 flag */
    SEQ_STATE_T3_REACHED,     /* Proximity sensor activation state (3.5s before target) */
    SEQ_STATE_TERMINAL        /* Final state during terminal approach to target */
} SequencerState;

/**
 * @brief Sequencer configuration parameters
 */
typedef struct {
    double dt4;                  /* Delay between T2 and pitch/yaw activation (seconds) */
    double rollRateThreshold1;   /* Roll rate threshold for T1 (rps) - 7 rps */
    double rollRateThreshold2;   /* Roll rate threshold for T2 (rps) - 2 rps */
    double rollRateThreshold3;   /* Roll rate threshold for IMU in loop (rps) - 5 rps */
    double windowStartOffset;    /* Sensing window start offset (seconds) */
    double windowDuration;       /* Sensing window duration (seconds) */
    int consecutiveChecksRequired; /* Required consecutive checks for state transitions */
    double proximityTimeBeforeTarget; /* Time before target for T3 (seconds) - 3.5s */
} SequencerConfig;

/**
 * @brief Sequencer status structure
 */
typedef struct {
    SequencerState state;     /* Current sequencer state */
    double missionTime;       /* Mission time since T0 (seconds) */
    double t0Time;            /* T0 timestamp */
    double t1Time;            /* T1 timestamp */
    double t2Time;            /* T2 timestamp */
    double t3Time;            /* T3 timestamp */
    Bool navActive;           /* Navigation module active flag */
    Bool rollControlActive;   /* Roll control active flag */
    Bool pitchYawActive;      /* Pitch/yaw control active flag */
    Bool guidanceActive;      /* Guidance module active flag */
    Bool proximityActive;     /* Proximity sensor active flag */
    Bool fsaFlag;             /* Flight Sequencing Algorithm flag (for T1) */
    Bool dapFlag;             /* Digital Autopilot flag (for T2) */
    Bool imuInLoopFlag;       /* IMU in the loop flag (after T1) */
} SequencerStatus;

/**
 * @brief Initialize the sequencer module
 * 
 * @param config Pointer to sequencer configuration structure (NULL for default)
 * @return Status Status code
 */
Status Sequencer_Initialize(const SequencerConfig* config);

/**
 * @brief Execute one step of the sequencer state machine
 * 
 * @param timeStep Time step in seconds
 * @return Status Status code
 */
Status Sequencer_Execute(double timeStep);

/**
 * @brief Get current sequencer status
 * 
 * @param status Pointer to status structure to fill
 * @return Status Status code
 */
Status Sequencer_GetStatus(SequencerStatus* status);

/**
 * @brief Get current mission time
 * 
 * @return double Mission time in seconds
 */
double Sequencer_GetMissionTime(void);

/**
 * @brief Check if navigation is active
 * 
 * @return Bool TRUE if active, FALSE otherwise
 */
Bool Sequencer_IsNavigationActive(void);

/**
 * @brief Check if roll control is active
 * 
 * @return Bool TRUE if active, FALSE otherwise
 */
Bool Sequencer_IsRollControlActive(void);

/**
 * @brief Check if pitch/yaw control is active
 * 
 * @return Bool TRUE if active, FALSE otherwise
 */
Bool Sequencer_IsPitchYawActive(void);

/**
 * @brief Check if guidance is active
 * 
 * @return Bool TRUE if active, FALSE otherwise
 */
Bool Sequencer_IsGuidanceActive(void);

/**
 * @brief Check if proximity sensor is active
 * 
 * @return Bool TRUE if active, FALSE otherwise
 */
Bool Sequencer_IsProximityActive(void);

/**
 * @brief Check if FSA flag is active
 * 
 * @return Bool TRUE if active, FALSE otherwise
 */
Bool Sequencer_IsFSAFlagActive(void);

/**
 * @brief Check if DAP flag is active
 * 
 * @return Bool TRUE if active, FALSE otherwise
 */
Bool Sequencer_IsDAPFlagActive(void);

/**
 * @brief Check if IMU in loop flag is active
 * 
 * @return Bool TRUE if active, FALSE otherwise
 */
Bool Sequencer_IsIMUInLoopActive(void);

/**
 * @brief Get current sequencer state
 * 
 * @return SequencerState Current state
 */
SequencerState Sequencer_GetState(void);

/**
 * @brief Get string representation of sequencer state
 * 
 * @param state Sequencer state
 * @return const char* String representation
 */
const char* Sequencer_GetStateString(SequencerState state);

/*
 * Simulation-specific functions that will be replaced with real hardware interfaces
 */

/**
 * @brief Set the roll rate for simulation
 * 
 * @param rollRate Roll rate in rps
 * @return Status Status code
 */
Status Sequencer_Sim_SetRollRate(double rollRate);

/**
 * @brief Set the GT3 flag from guidance for simulation
 * 
 * @param gt3Flag GT3 flag value
 * @return Status Status code
 */
Status Sequencer_Sim_SetGT3Flag(Bool gt3Flag);

/**
 * @brief Reset the sequencer to initial state (for simulation)
 * 
 * @return Status Status code
 */
Status Sequencer_Sim_Reset(void);

#endif /* SEQUENCER_H */