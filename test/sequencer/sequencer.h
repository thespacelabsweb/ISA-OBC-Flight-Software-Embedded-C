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
#define SEQ_ROLL_RATE_T1_THRESHOLD 70U  // 7.O rps in fixed point
#define SEQ_ROLL_RATE_T2_THRESHOLD 20U  // 2.0 rps in fixed point

// Timing Configuration
#define SEQ_T1_WINDOW_IN_TIME      10U  // T1 window starts at T0 + 0.1s (l0 cycles)
#define SEQ_T1_WINDOW_OUT_TIME     500U  // T1 window ends at T0 + 5s (500 cycles)

#define SEQ_T2_WINDOW_IN_TIME      10U //T2 window starts at T1 + 0.1s -> T1 + 5 secs
#define SEQ_T2_WINDOW_OUT_TIME     500U //T2 window ends at T1 + 0.1s -> T1 + 5 secs

#define SEQ_T3_WINDOW_IN_TIME      10U //T3 window starts at T1 + 0.1s -> T1 + 5 secs
#define SEQ_T3_WINDOW_OUT_TIME     500U //T3 window ends at T1 + 0.1s -> T1 + 5 secs

#define SEQ_CANARD_FLAG_DELAY      0U   // No delay after FSA flag (0 cycles)
#define SEQ_CONTROL_FLAG_DELAY     10U  // Delay after T2 set (10 cycles = 0.1s)
#define SEQ_GUID_START_FLAG_DELAY  200U // Delay after Control flag (200 cycles = 2s)


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
    bool isFsaFlagSent;
    bool isCanardFlagSent;
    bool isControlFlagSent;
    bool isGuidStartFlagSent;

    //Timing - tells at when events happen
    uint32_t mainClockCycles;      // counts minor cycles since launch
    uint32_t fsaFlagSendTime;      // when to send the FSA flag
    uint32_t canardFlagSendTime;   // when to send canard flag
    uint32_t controlFlagSendTime;  // when to send control flag
    uint32_t guidStartFlagSendTime; // when to send guidance start flag

    // Roll rate confirmation
    uint8_t t1RollRateCount;
    uint8_t t2RollRateCount;

    // System Status
    bool isGswitchActive;
} SequencerState_t;

// Output structure - what sequencer tells other systems what to do
typedef struct {
    // Flags to send
    bool sendFsaFlag;              // Changed from fsaFlag
    bool sendCanardFlag;           // Changed from canardFlag
    bool sendControlFlag;          // Changed from controlFlag
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
                                    uint16_t rollRateFp, //Roll rate *10
                                    uint32_t tGo,   //Form guidance system
                                    SequencerOutput_t* output);
//Intialize sequencer at system startup
SequencerError_t sequencerInit(SequencerState_t* state);

//Set G-Switch activation (called when launch is detected)
SequencerError_t sequencerSetGswitch(SequencerState_t* state, bool isActive);                                