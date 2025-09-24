/******************************************************************************
* ISA Flight Software
* @file sequencer.c
* @brief Projectile Flight Computer Sequencer Module
* @details Manages mission phases and state transitions for precision-guided projectile
* @author Ananthu Dev, Project Engineer, Spacelabs. 
* @date 2025
* @version 2.0
* 
* MISRA C: Compliant Implementation
*****************************************************************************/

#include "sequencer.h"
#include <string.h> //for memset

SequencerError_t sequencerInit(SequencerState_t* state)
{
    // FLight software rule: Always validate input parameters first
    if (state == NULL) {
        return SEQ_ERROR_INVALID_PARAM;
    }

    //Intialize all state to zero (safest starting condtion)
    memset(state, 0, sizeof(SequencerState_t));

    // Explicitly set initial values eventhough memset made them 0
    //this make the code self-documenting
    state->isT0Set = false;
    state->isT1Set = false;
    state->isT2Set = false;
    state->isT3Set = false;

    // Intialize all flags to false
    state->isFsaActivateFlagSent = false;
    state->isCanardDeployFlagSent = false;
    state->isCanardControlFlagSent = false;
    state->isGuidStartFlagSent = false;

    // Initialize counters and timers to zero
    state->mainClockCycles = 0U;
    state->fsaActivateFlagSendTime = 0U;
    state->canardDeployFlagSendTime = 0U;
    state->canardControlFlagSendTime = 0U;
    state->guidStartFlagSendTime = 0U;
    state->t1RollRateCount = 0U;
    state->t2RollRateCount = 0U;

    // G-switch starts inactive
    state->isOBCReset= false;

    return SEQ_SUCCESS;
}

SequencerError_t sequencerSetOBCReset(SequencerState_t* state, bool isActive)
{
    // Parameter validation
    if (state == NULL) {
        return SEQ_ERROR_INVALID_PARAM;
    }

    // Set G-switch state
    state->isOBCReset= isActive;

    if (isActive) {
        //g switch activation triggers the start of timing
        // This is T=0 moment  - reset the OBC
        state->mainClockCycles = 0U;

        // Set T0 phase immediately when G -switch activates
        state->isT0Set = true;

        // Reset all confirmation counters
        state->t1RollRateCount = 0U;
        state->t2RollRateCount = 0U;
    }
// it doesn't anything when G-switch goes inactive
// Oncel launched, it stays launched    
    return SEQ_SUCCESS;
}

// Helper function to check the roll rate condition
static bool isRollRateOkForT1(float rollRateFp)
{
    // Roll rate threshold for T1 is 7.0 rps (70 in fixed point)
    return (rollRateFp <= SEQ_ROLL_RATE_T1_THRESHOLD);
}

static SequencerError_t processT1Logic(SequencerState_t* state,
                                        float rollRateFp,
                                        SequencerOutput_t* output)
{
    //First check if T1 window is out (T > T1WindowOut)
    if (state->mainClockCycles > SEQ_T1_WINDOW_OUT_TIME) {
        // Window out - Set T1 Immediately
        state->isT1Set = true;
        output->setT1 = true;
        state->t1SetTime = state->mainClockCycles; // Record T1 recordtime

        // Send the canard deploy flag if not alraedy sent
        if (!state->isCanardDeployFlagSent) {
            output->canardDeployFlag = true;
            state->isCanardDeployFlagSent = true;
        }

        return SEQ_SUCCESS;
    }
    // Check for T1 window sensing
    if (state->mainClockCycles > SEQ_T1_WINDOW_IN_TIME) {
        // Check if the roll rate <= 7rps conditions are met
        if (isRollRateOkForT1(rollRateFp)) {
            //if the roll rate is good increment the confirmation counter
            state->t1RollRateCount++;

            // Check if the condition has been met for the required number of cycles
            if (state->t1RollRateCount >= SEQ_CONFIRMATION_CYCLES) {
                //  Set T1
                state ->isT1Set = true;
                output->setT1 = true;
                state->t1SetTime = state->mainClockCycles;

                //Send the canard deploy flag if not sent
                if (!state->isCanardDeployFlagSent) {
                    output->canardDeployFlag = true;
                    state->isCanardDeployFlagSent = true;
                }

                return SEQ_SUCCESS;
            }
        } else {
            // Roll rate is not within the threshold - reset the counter
            state ->t1RollRateCount = 0U;
        }
    }
    return SEQ_SUCCESS;
}

// T2 logic starts

// Helper function to check the roll rate condition for T2
static bool isRollRateOkForT2(float rollRateFp)
{
    return (rollRateFp <= SEQ_ROLL_RATE_T2_THRESHOLD);
}
static SequencerError_t processT2Logic(SequencerState_t* state,
                                    float rollRateFp,
                                    SequencerOutput_t* output)
{
     // handle the time out conditions
     if (state->mainClockCycles > (state->t1SetTime + SEQ_T2_WINDOW_OUT_TIME)) {
        // Window expired - set t2 immediately
        state->isT2Set = true;
        output->setT2 = true;
        state->t2SetTime = state->mainClockCycles; // Record T2 set time
        // Schedule the Canard control flag
        state->canardControlFlagSendTime = state->mainClockCycles + SEQ_CANARD_CONTROL_ON_FLAG_DELAY;
        return SEQ_SUCCESS;
     }
     //check the T2 detection window
     if (state->mainClockCycles > (state->t1SetTime + SEQ_T2_WINDOW_IN_TIME)) {
        // Check if the roll rate is below the T2 threshold(<= 2 rps).
        if (isRollRateOkForT2(rollRateFp)) {
            state->t2RollRateCount++;
            //check if the roll rate persists for 3 consecutive cycles.
            if (state->t2RollRateCount >= SEQ_CONFIRMATION_CYCLES) {
                // Set T2
                state->isT2Set = true;
                output->setT2 = true;
                state->t2SetTime = state->mainClockCycles; // Record T2 set time
                // Schedule the canard control flag,
                state->canardControlFlagSendTime = state->mainClockCycles + SEQ_CANARD_CONTROL_ON_FLAG_DELAY;
                return SEQ_SUCCESS;
            }
        } else {
            //Roll rate is not within the threshold - reset the counter .
            state->t2RollRateCount = 0U;
        }
     }
     return SEQ_SUCCESS;
}


//T2 logic ends


//T3 logic starts
// Implementation of T3 logic
static SequencerError_t processT3Logic(SequencerState_t* state,
                                     uint32_t tGo,
                                     SequencerOutput_t* output)
{
    // wait for the scheduled time to send the Canard Control Flag.
    if (!state->isCanardControlFlagSent) {
        if (state->mainClockCycles > state->canardControlFlagSendTime) {
            output->canardControlFlag = true;
            state->isCanardControlFlagSent = true;

            // Schedule the flags to be sent
            state->fsaActivateFlagSendTime = state->mainClockCycles + SEQ_FSA_FLAG_DELAY;
            state->guidStartFlagSendTime = state->mainClockCycles + SEQ_GUID_START_FLAG_DELAY;
        }
    } else {
        // Control flag has been sent. Now, process the parallel FSA and Guidance flags.

        // Check if its time to send the FSA flag.
        if (!state->isFsaActivateFlagSent && (state->mainClockCycles > state->fsaActivateFlagSendTime)) {
            output->fsaActivateFlag = true;
            state->isFsaActivateFlagSent = true;
        }
        // check if it's time to send the Guidance Start Flag.
        if (!state->isGuidStartFlagSent && (state->mainClockCycles > state->guidStartFlagSendTime)) {
            output->sendGuidStartFlag = true;
            state->isGuidStartFlagSent = true;
            state->isT3Set = true;
            output->setT3 = true;
            state->t3SetTime = state->mainClockCycles; // Record T3 set time
            output->enableProximitySensor = true;
            return SEQ_SUCCESS;
        }
    }
    
    return SEQ_SUCCESS;
}
//T3 logic ends

//Window priority
SequencerError_t sequencerExecute(SequencerState_t* state,
                                 float rollRateFp,
                                 uint32_t tGo,
                                 SequencerOutput_t* output )
{
    // Parameter validation first (critical for flight software)
    if((state == NULL) || (output == NULL)) {
        return SEQ_ERROR_INVALID_PARAM;
        // Early exit on error
    }
    //clear all outputs first (safe starting state)
    memset(output, 0, sizeof(SequencerOutput_t)); // Safe starting state

    //Increment main clock if G-switch is active
    if (state->isOBCReset){
        state->mainClockCycles++;
    }
    // Priority-based logic for ISA
    // check T3 first, then t2, then t1, then t0

    if (state->isT3Set) {
        //T3 is set - this is the end, sequencer exits
        return SEQ_SUCCESS;
    }
    if (state->isT2Set) {
        //the porjectile in T2 phase - check for t3 condtions
        return processT3Logic(state, tGo, output);
    }
    if (state->isT1Set) {
        // the projectile is in T1 phase - check for T2 conditions
        return processT2Logic(state, rollRateFp, output);
    }

    if (state->isT0Set) {
        // the projectile is in T0 phase, check for T1 conditions
        return processT1Logic(state, rollRateFp, output);
    }

    //If we get here, no phases are set (should not happen after G-switch)
    return SEQ_ERROR_INVALID_STATE;
}







    

