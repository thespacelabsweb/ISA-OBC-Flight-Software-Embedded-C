#ifndef SEQUENCER_H
#define SEQUENCER_H

#include <stdbool.h>
#include <stdint.h>
/**
 * @brief Defines the states of the sequencer's state machine, corresponding
 * to the major phases of the mission flight plan.
 */

typedef enum {
    SEQ_STATE_PRE_FIRING, //obc is on, but projectile has not been fired
    SEQ_STATE_FIRING_DETECTED, //G-Switch has triggered. Mission timer starts.
    SEQ_STATE_HIGH_SPIN, //Just exited the barrel, roll is very high
    SEQ_STATE_CANARD_DEPLOY, //Roll rate has dropped, canards deployed
    SEQ_STATE_ROLL_CONTROL, //Roll is low enough for the DAP to take control.
    SEQ_STATE_PITCH_YAW_CONTROL, //DAP's pitch and yaw control is now active
    SEQ_STATE_GUIDANCE,  //The main guidance phase is active
    // SEQ_STATE_TERMINAL, //Nearing the target, preparing for impact.
    SEQ_STATE_IMPACT, // Mission complete.
} SequencerState;

/**
 * @brief Holds all status flags and timing information managed by the sequencer.
 * This structure provides a complete snapshot of the mission's progress.
 */
typedef struct {
    // -- The Core State ---

    SequencerState state;

    //--- The Master Clock ---
    uint32_t globalMinorCycleCount; //The global counter that ticks every 10ms (minor cycle)

    // --- Event Timestamps ---
    // Stores the cycle count when major events happen.
    uint32_t t0_Set_CycleCount; // Cycle when T0 (firing) was set.
    uint32_t t1_Set_CycleCount; // Cycle when T1 (Roll < 7rps) was set.
    uint32_t t2_Set_CycleCount; // Cycle when T2 (Roll < 2rps was set.)

    // Counter for consecutive checks, ie, 3 consecutive cycles
    uint8_t rollRateCheck_ConsecutiveCount;

    // --- Flag for other modules ---
    // These are the "outputs of the sequencer"

    bool fsaCanardDeploy_Flag; //Flag to activate the Canard/Fin
    bool dapRollControl_Flag; // Flag for DAP's roll control
    bool dapPitchYawControl_Flag; // Flag for DAP's Pitch/Yaw control
    bool GUID_START_Flag; // Flag to enable the Guidance Module
    bool proximityEnable_Flag; // Flag for proximity.
} SequencerStatus;

// --- Function Prototypes ---
void Sequencer_Initialize(SequencerStatus* status);
void Sequencer_Execute(SequencerStatus* status, bool gSwitchActive, double rollRate, bool guidanceTerminal_Flag);
const char* Sequencer_GetStateString(SequencerState state);

#endif /* SEQUENCER_H */