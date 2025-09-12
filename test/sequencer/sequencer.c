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
        
        // Calculate flag send times for T2 phase
        state->fsaActivateFlagSendTime = state->mainClockCycles;

        // Delay canard flag by defined delay after FSA flag
        state->canardDeployFlagSendTime = state->fsaActivateFlagSendTime + SEQ_CANARD_DEPLOY_FLAG_DELAY ;
        
        return SEQ_SUCCESS;

    }
    // Check it is in the T1 Window (T > T1windowIn)
    if (state->mainClockCycles > SEQ_T1_WINDOW_IN_TIME) {
        // Check roll rate <= 7 rps
        if (isRollRateOkForT1(rollRateFp)) {
            // Roll rate good - increment counter
            state->t1RollRateCount++;

            // Check if counter= 3
            if (state->t1RollRateCount >= SEQ_CONFIRMATION_CYCLES) {
                //Set T1
                state->isT1Set = true; // transition to T1
                output->setT1 = true; // command to set T1
                
                // Calculate flag send times for T2 phase
                state->fsaActivateFlagSendTime = state->mainClockCycles;
                state->canardDeployFlagSendTime = state->fsaActivateFlagSendTime + SEQ_CANARD_DEPLOY_FLAG_DELAY;
                
                return SEQ_SUCCESS;
            }
        } else {
            // Roll rate not good - reset counter
            state->t1RollRateCount = 0U;
        }
    }
    // If we get here, conditions not met -exit
    return SEQ_SUCCESS;
}

// Helper function to check the roll rate condition for T2
static bool isRollRateOkForT2(float rollRateFp)
{
    return (rollRateFp <= SEQ_ROLL_RATE_T2_THRESHOLD);
}
static SequencerError_t processT2Logic(SequencerState_t* state,
                                    float rollRateFp,
                                    SequencerOutput_t* output)
{
    // First check if FSA flag is already sent
    if (!state->isFsaActivateFlagSent) {
        // Check if its time to send FSA flag
        if (state->mainClockCycles > state->fsaActivateFlagSendTime) {
            output->fsaActivateFlag = true;
            state->isFsaActivateFlagSent = true;
            return SEQ_SUCCESS;
        }
    } else if (!state->isCanardDeployFlagSent) {
        // FSA flag sent, check if its time to send canard flag
        if (state->mainClockCycles > state->canardDeployFlagSendTime) {
            output->canardDeployFlag = true;
            state->isCanardDeployFlagSent = true;
            return SEQ_SUCCESS;
        }
    } else {
        // both flags sent, check T2 window conditions
        // First check if T2 window is out
        if (state->mainClockCycles > (state->fsaActivateFlagSendTime + SEQ_T2_WINDOW_OUT_TIME)) {
            // window out - Set T2 immediately
            state->isT2Set = true;
            output->setT2 = true;

            // Calculate flag send times for T3 phase
            // Control flag after defined delay
            state->canardControlFlagSendTime = state->mainClockCycles + SEQ_CANARD_CONTROL_ON_FLAG_DELAY;
            // GUID_START flag after control flag delay
            state->guidStartFlagSendTime = state->mainClockCycles + SEQ_GUID_START_FLAG_DELAY;

            return SEQ_SUCCESS;
        }

        // Check it is in the T2 window
        if (state->mainClockCycles > (state->fsaActivateFlagSendTime + SEQ_T2_WINDOW_IN_TIME)) {
            // check roll rate <= 2rps
            if (isRollRateOkForT2(rollRateFp)) {
                // Roll rate good - increment counter
                state->t2RollRateCount++;

                // Check if counter = 3
                if (state->t2RollRateCount >= SEQ_CONFIRMATION_CYCLES) {
                    // Set T2
                    state->isT2Set = true;
                    output->setT2 = true;

                    // Calculate flag send times for T3 phase
                    state->canardControlFlagSendTime = state->mainClockCycles + SEQ_CANARD_CONTROL_ON_FLAG_DELAY; // Control flag after defined delay
                    // GUID_START flag after control flag delay
                    state->guidStartFlagSendTime = state->mainClockCycles + SEQ_GUID_START_FLAG_DELAY; // T2 + delta t for guidance flag send delay
                    
                    return SEQ_SUCCESS;
                }
            } else {
                // Roll rate not good - reset counter
                state->t2RollRateCount = 0U;
            }
        }
    }
}
// Implementation of T3 logic
static SequencerError_t processT3Logic(SequencerState_t* state,
                                     uint32_t tGo,
                                     SequencerOutput_t* output)
{
    // First check if Control flag is already sent
    if (!state->isCanardControlFlagSent) {
        // Check if it's time to send control flag
        // Control flag is sent after T2 + defined delay
        if (state->mainClockCycles > state->canardControlFlagSendTime) {
            // Send control flag
            output->canardControlFlag = true; 
            state->isCanardControlFlagSent = true;
            return SEQ_SUCCESS;
        }
    } else if (!state->isGuidStartFlagSent) {
        // Control flag sent, check if it's time to send GUID_START flag
        if (state->mainClockCycles > state->guidStartFlagSendTime) {
            output->sendGuidStartFlag = true;
            state->isGuidStartFlagSent = true;
            return SEQ_SUCCESS;
        }
    } else {
        // Both flags sent, check T3 window conditions
        
        // First check if T3 window is out
        if (state->mainClockCycles > (state->canardControlFlagSendTime + SEQ_T3_WINDOW_OUT_TIME)) {
            // T3 window out - set T3 and enable proximity sensor
            state->isT3Set = true;
            output->setT3 = true;
            output->enableProximitySensor = true; // Enable proximity sensor at T3
            return SEQ_SUCCESS;
        }
        
        // Check if in T3 window
        if (state->mainClockCycles > (state->canardControlFlagSendTime + SEQ_T3_WINDOW_IN_TIME)) {
            // Check tGo from guidance
            if (tGo > 0U) {  // tGo parameter indicates guidance is ready for T3
                // Set T3 and enable proximity sensor
                state->isT3Set = true;
                output->setT3 = true;
                output->enableProximitySensor = true;
                return SEQ_SUCCESS;
            }
        }
    }
    
    // If we get here, conditions not met - exit
    return SEQ_SUCCESS;
}

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







    

