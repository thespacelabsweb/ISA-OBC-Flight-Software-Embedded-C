/******************************************************************************
* ISA Flight Software
* @file sequencer.h
* @brief Projectile Flight Computer Sequencer Module
* @details Manages mission phases and state transitions for precision-guided projectile
* @author Ananthu Dev, Project Engineer, Spacelabs. 
* @date 2025
* @version 2.0
* 
* MISRA C: Compliant Implementation
*****************************************************************************/

#ifndef SEQUENCER_H
#define SEQUENCER_H

#include <stdint.h>
#include <stdbool.h>


// Datatypes
#define SEQ_CONFIRMATION_CYCLES    3U
#define SEQ_ROLL_RATE_T1_THRESHOLD 7.0f  // 7.O rps 
#define SEQ_ROLL_RATE_T2_THRESHOLD 2.0f  // 2.0 rps

// Timing Configuration
#define SEQ_T1_WINDOW_IN_TIME      10U  // T1 window starts at T0 + 0.1s (l0 cycles)
#define SEQ_T1_WINDOW_OUT_TIME     500U  // T1 window ends at T0 + 5s (500 cycles)

#define SEQ_T2_WINDOW_IN_TIME      10U //T2 window starts at T1 + 0.1s -> T1 + 5 secs
#define SEQ_T2_WINDOW_OUT_TIME     500U //T2 window ends at T1 + 0.1s -> T1 + 5 secs

#define SEQ_T3_WINDOW_IN_TIME      10U //T3 window starts at T1 + 0.1s -> T1 + 5 secs
#define SEQ_T3_WINDOW_OUT_TIME     500U //T3 window ends at T1 + 0.1s -> T1 + 5 secs

                                        // Disregard the comments vineetha chechi and ananthu chnaged it for testing purposes           
#define SEQ_CANARD_DEPLOY_FLAG_DELAY      0U   // No delay after FSA flag (0 cycles) //changed
#define SEQ_CANARD_CONTROL_ON_FLAG_DELAY     0U  // Delay after T2 set (10 cycles = 0.1s) // changed
#define SEQ_FSA_FLAG_DELAY                 0U // delay for FSA flag if there is any
#define SEQ_GUID_START_FLAG_DELAY  200U // Delay after Control flag (200 cycles = 2s) //changed


// Error codes for flight software
typedef enum {
    SEQ_SUCCESS =  0U,
    SEQ_ERROR_INVALID_PARAM = 1U,
    SEQ_ERROR_INVALID_STATE = 2U
} SequencerError_t;

//State Structure
typedef struct {
    // Time Windows (says what phase is currently in)
    bool isT0Set;
    bool isT1Set;
    bool isT2Set;
    bool isT3Set;

    // Flags sent check
    bool isFsaActivateFlagSent;
    bool isCanardDeployFlagSent;
    bool isCanardControlFlagSent;
    bool isGuidStartFlagSent;

    //Timing - tells at when events happen
    uint32_t mainClockCycles;      // counts minor cycles since launch
    uint32_t t1SetTime;            // when T1 was set
    uint32_t t2SetTime;            // when T2 was set
    uint32_t t3SetTime;            // when T3 was set

    uint32_t fsaActivateFlagSendTime;      // when to send the FSA flag
    uint32_t canardDeployFlagSendTime;   // when to send canard flag
    uint32_t canardControlFlagSendTime;  // when to send control flag
    uint32_t guidStartFlagSendTime; // when to send guidance start flag

    // Roll rate confirmation
    uint8_t t1RollRateCount;
    uint8_t t2RollRateCount;

    // System Status
    bool isOBCReset;
} SequencerState_t;

// Output structure - what sequencer tells other systems what to do
typedef struct {
    // Flags to send
    bool fsaActivateFlag;              // Changed from fsaFlag
    bool canardDeployFlag;           // Changed from canardFlag
    bool canardControlFlag;          // Changed from controlFlag
    bool sendGuidStartFlag;        // Changed from guidStartFlag
    bool enableProximitySensor;    // Changed from proximitySensorFlag


    // Phase transistions
    bool setT0;
    bool setT1;
    bool setT2;
    bool setT3;
} SequencerOutput_t;

// Main sequencer function called every minor cycle
SequencerError_t sequencerExecute(SequencerState_t* state, 
                                    float rollRateFp, //Roll rate in RPS
                                    uint32_t tGo,   //Form guidance system
                                    SequencerOutput_t* output);
//Intialize sequencer at system startup
SequencerError_t sequencerInit(SequencerState_t* state);

//Set OBC Reset (called when launch is detected)
SequencerError_t sequencerSetOBCReset(SequencerState_t* state, bool isActive); 

#endif /* SEQUENCER_H */