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
 * @brief Sequencer state machine states
 */
typedef enum {
    SEQ_STATE_INIT = 0,
    SEQ_STATE_T0_WINDOW,
    SEQ_STATE_T1_REACHED,
    SEQ_STATE_T1_WINDOW,
    SEQ_STATE_T2_REACHED,
    SEQ_STATE_PITCH_YAW_ON,
    SEQ_STATE_GUIDANCE_ON,
    SEQ_STATE_T2_WINDOW,
    SEQ_STATE_T3_REACHED,
    SEQ_STATE_TERMINAL
} SequencerState;

/**
 * @brief Sequencer configuration parameters
 */
typedef struct {
    double dt4;                       /* Delay after T2 before enabling pitch/yaw (seconds) */
    double rollRateThreshold1;        /* Threshold for T1 (canard deployment) in rps */
    double rollRateThreshold2;        /* Threshold for T2 (roll control) in rps */
    double rollRateThreshold3;        /* Threshold for IMU-in-loop in rps */
    double windowStartOffset;         /* Sensing window start offset (seconds) for T1/T2 */
    double windowDuration;            /* Sensing window duration (seconds) for T1/T2 */
    double t3WindowStartOffset;       /* Sensing window start offset (seconds) for T3 */
    double t3WindowDuration;          /* Sensing window duration (seconds) for T3 */
    int    consecutiveChecksRequired; /* Number of consecutive checks to validate condition */
    double proximityTimeBeforeTarget; /* Time before target to enable proximity (seconds) */
} SequencerConfig;

/**
 * @brief Snapshot of sequencer status
 */
typedef struct {
    SequencerState state;
    double missionTime;
    double t0Time;
    double t1Time;
    double t2Time;
    double t3Time;

    Bool navActive;
    Bool rollControlActive;
    Bool pitchYawActive;
    Bool guidanceActive;
    Bool proximityActive;

    Bool fsaFlag;
    Bool dapFlag;
    Bool imuInLoopFlag;
} SequencerStatus;

/**
 * @brief Initialize the sequencer module
 * @param config Optional configuration (pass NULL for defaults)
 * @return Status
 */
Status Sequencer_Initialize(const SequencerConfig* config);

/**
 * @brief Execute one step of the sequencer state machine
 * @param timeStep Time step in seconds
 * @return Status
 */
Status Sequencer_Execute(double timeStep);

/**
 * @brief Retrieve the current sequencer status snapshot
 * @param status Output pointer to receive status
 * @return Status
 */
Status Sequencer_GetStatus(SequencerStatus* status);

/**
 * @brief Getters for convenience
 */
double        Sequencer_GetMissionTime(void);
Bool          Sequencer_IsNavigationActive(void);
Bool          Sequencer_IsRollControlActive(void);
Bool          Sequencer_IsPitchYawActive(void);
Bool          Sequencer_IsGuidanceActive(void);
Bool          Sequencer_IsProximityActive(void);
Bool          Sequencer_IsFSAFlagActive(void);
Bool          Sequencer_IsDAPFlagActive(void);
Bool          Sequencer_IsIMUInLoopActive(void);
SequencerState Sequencer_GetState(void);
const char*   Sequencer_GetStateString(SequencerState state);

/**
 * @brief Simulation hooks (to be replaced by real interfaces on hardware)
 */
Status Sequencer_Sim_SetRollRate(double rollRate);
Status Sequencer_Sim_SetGT3Flag(Bool gt3Flag);
Status Sequencer_Sim_Reset(void);

#endif /* SEQUENCER_H */


