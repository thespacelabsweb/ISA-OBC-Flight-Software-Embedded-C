#include "sequencer.h"
#include <stdio.h>

/**
* @brief Initializes the sequencer to its pre-firing state.
@param status A pointer to the SequencerStatus structure to be initialized.
*/

// --- Configuration Constants ---
// T1 Window cycle definition (Waiting for canard deployment)
#define T1_WINDOW_START_CYCLES 10 // 0.1 seconds (10 cycles)
#define T1_WINDOW_END_CYCLES 500  // 5.0 seconds (500 cycles)
#define ROLL_RATE_T1_RPS 7.0
#define CONSECUTIVE_CHECKS_REQUIRED 3

// T2 Window Cycle definition (Waiting for Roll Control)
#define T2_WINDOW_START_CYCLES 10 // 0.1 seconds
#define T2_WINDOW_END_CYCLES 500  // 5.0 seconds
#define ROLL_RATE_T2_RPS 2.0

#define PITCH_YAW_DELAY_CYCLES 500 // 5.0 seconds (500 cycles)
#define GUIDANCE_START_DELAY_CYCLES                                            \
  200 // 2.0 Seconds T2 + 2 Seconds after pitch/yaw

void Sequencer_Initialize(SequencerStatus *status) {
  if (status == NULL) {
    return; // Safety check for misra c, for false memory writing. pani kittathe
            // irikan.
  }

  // Set the initial state to PRE_FIRING
  status->state = SEQ_STATE_PRE_FIRING;

  // Initialize all timers and counters to zero
  status->globalMinorCycleCount = 0;
  status->t0_Set_CycleCount = 0;
  status->t1_Set_CycleCount = 0;
  status->t2_Set_CycleCount = 0;

  // Ensure all output flags are set to FALSE at startup
  status->fsaCanardDeploy_Flag = FALSE;
  status->dapRollControl_Flag = FALSE;
  status->dapPitchYawControl_Flag = FALSE;
  status->GUID_START_Flag = FALSE;
}

/**
 * @brief Executes one step of the sequencer state machine.
 * @param status A pointer to the SequencerStatus structure.
 * @param gSwitchActive The current state of the G-Switch (firing detector).
 */

void Sequencer_Execute(SequencerStatus *status,
                       Bool gSwitchActive double rollRate) {
  if (status == NULL) {
    return;
  }

  // This is the master clock. it increments every time this function is called.

  status->globalMinorCycleCount++;
  // The switch statement checks which state we are currently in
  switch (status->state) {
  case SEQ_STATE_PRE_FIRING:
    // waiting for the T0 State
    // The G-switch firing is what sets T0
    if (gSwitchActive == TRUE) {
      // The projectile has been fired!
      printf("EVENT: Firing Detected!\n");

      // Record the exact time of the event.
      status->t0_Set_CycleCount = status->globalMinorCycleCount;

      // Move to the next state on the checklist.

      status->state = SEQ_STATE_FIRING_DETECTED;
    }
    break; // End of logic for this state.

  // Projectile Fired
  case SEQ_STATE_FIRING_DETECTED:
    // Transistional state , the projectile will me immediately to the high spin
    // rates
    printf("STATE TRANSISTION: FIRING_DETECTED -> HIGH_SPIN RATES\n");
    status->state = SEQ_STATE_HIGH_SPIN;
    break;

  // HIGH SPIN RATE
  case SEQ_STATE_HIGH_SPIN:
    // Check if it is within the time window to look for  T1 event.
    if (status->globalMinorCycleCount >
            (status->t0_Set_CycleCount + T1_WINDOW_START_CYCLES) &&
        status->globalMinorCycleCount <
            (status->t0_Set_CycleCount + T1_WINDOW_END_CYCLES)) {
      // check whether it is in the window, check the roll rate
      if (rollRate < ROLL_RATE_T1_RPS) {
        // iF condition is met, increment the consecutive counter
        status->rollRateCheck_ConsecutiveCount++;
      } else {
        // If it fails, reset the counter to Zero
        status->rollRateCheck_ConsecutiveCount = 0;
      }
      // here it check if we have met the condition for 3 cycles in a row
      if (status->rollRateCheck_ConsecutiveCount >=
          CONSECUTIVE_CHECKS_REQUIRED) {
        printf("EVENT: T1 Set! (Roll Rate < 7 rps for 3 cycles)\n");

        // Record the timestamp for T1
        status->t1_Set_CycleCount = status->globalMinorCycleCount;

        // Set the flag for canard deployment
        status->fsaCanardDeploy_Flag = TRUE;

        // Reset the counter for the next check
        status->rollRateCheck_ConsecutiveCount = 0;

        // Move to the next state
        status->state = SEQ_STATE_CANARD_DEPLOY;
      } else if (status->globalMinorCycleCount >=
                 (status->t0_Set_CycleCount + T1_WINDOW_END_CYCLES)) {
        // iF the window is passed without setting T1,it's an exit condition
        printf("Warning - T1 Window Out!\n");
        // have to do sth here
      }
      break;

    case SEQ_STATE_CANARD_DEPLOY:
      // Threshold for T2
      if (status->globalMinorCycleCount >
              (status->t1_Set_CycleCount + T2_WINDOW_START_CYCLES) &&
          status->globalMinorCycleCount <
              (status->t1_Set_CycleCount + T2_WINDOW_END_CYCLES)) {

        // check the roll rate is below 2rps
        if (rollRate < ROLL_RATE_T2_RPS) {
          status->rollRateCheck_ConsecutiveCount++;
        } else {
          // Reset if the condition fails.
          status->rollRateCheck_ConsecutiveCount = 0;
        }

        // Check for 3 consecutive cycles
        if (status->rollRateCheck_ConsecutiveCount >=
            CONSECUTIVE_CHECKS_REQUIRED) {
          printf("EVENT: T2 Set! (Roll Rate < 2 rps for 3 cycles)\n");

          // Record the timestamp for T2
          status->t2_Set_CycleCount = status->globalMinorCycleCount;

          // Enable Roll Control
          // Flag for DAP
          printf("ACTION: Roll Control On\n");
          status->dapRollControl_Flag = TRUE;

          // Reset the counter, ready for further checks
          status->state = SEQ_STATE_ROLL_CONTROL;
        }
      } else if (status->globalMinorCycleCount >=
                 (status->t1_Set_CycleCount + T2_WINDOW_END_CYCLES)) {
        printf("ERROR: T2 Window Out!");
      }
      break;
    }
  case SEQ_STATE_ROLL_CONTROL:
    // In this state, it will wait for time delays after T2.
    // First check if it's time to enable pitch/yaw control.
    if (status->dapPitchYawControl_Flag == FALSE &&
        (status->globalMinorCycleCount >=
         status->t2_Set_CycleCount + PITCH_YAW_DELAY_CYCLES)) {
      printf("ACTION: Pitch and Yaw Control ON\n");
      status->dapPitchYawControl_Flag = TRUE;
    }
    // Next, check if it's time to start guidance. This happens after pitch and
    // yaw is stable
    if (status->GUID_START_Flag == FALSE &&
        (status->globalMinorCycleCount >=
         status->t2_Set_CycleCount + GUIDANCE_START_DELAY_CYCLES)) {
      printf("ACTION: GUID_START flag set for Guidance Module\n");
      status->GUID_START_Flag = TRUE;

      // Now guidance is active, move to the main guidance state.
      status->state = SEQ_STATE_GUIDANCE;
    }
    break;
  }
