/******************************************************************************
 * ISA Flight Software
 * 
 * File: sequencer.c
 * Description: Sequencer module implementation
 *****************************************************************************/

#include <stdio.h>
#include <string.h>
#include "../../include/sequencer/sequencer.h"

/* Default configuration values */
static const SequencerConfig DEFAULT_CONFIG = {
    .dt4 = 0.5,                   /* 500ms delay for pitch/yaw activation after T2 */
    .rollRateThreshold1 = 7.0,    /* 7 rps threshold for T1 (canard deployment) */
    .rollRateThreshold2 = 2.0,    /* 2 rps threshold for T2 (roll control) */
    .rollRateThreshold3 = 5.0,    /* 5 rps threshold for IMU in the loop (after T1) */
    .windowStartOffset = 0.1,     /* 100ms window start offset */
    .windowDuration = 5.0,        /* 5s window duration */
    .consecutiveChecksRequired = 3, /* 3 consecutive checks required */
    .proximityTimeBeforeTarget = 3.5 /* 3.5s before reaching target for T3 */
};

/* Sequencer internal state */
typedef struct {
    SequencerState state;         /* Current state */
    SequencerConfig config;       /* Configuration parameters */
    
    /* Timing */
    double missionTime;           /* Time since T0 (seconds) */
    double t0Time;                /* T0 timestamp */
    double t1Time;                /* T1 timestamp */
    double t2Time;                /* T2 timestamp */
    double t3Time;                /* T3 timestamp */
    
    /* Status flags */
    Bool navActive;               /* Navigation active flag */
    Bool rollControlActive;       /* Roll control active flag */
    Bool pitchYawActive;          /* Pitch/yaw control active flag */
    Bool guidanceActive;          /* Guidance active flag */
    Bool proximityActive;         /* Proximity sensor active flag */
    Bool fsaFlag;                 /* Flight Sequencing Algorithm flag (for T1) */
    Bool dapFlag;                 /* Digital Autopilot flag (for T2) */
    Bool imuInLoopFlag;           /* IMU in the loop flag (after T1) */
    
    /* Sensor data (simulated for now) */
    double rollRate;              /* Current roll rate (rps) */
    Bool gt3Flag;                 /* GT3 flag from guidance */
    
    /* State transition tracking */
    int consecutiveCount;         /* Counter for consecutive checks */
    int imuLoopConsecutiveCount;  /* Counter for IMU in loop consecutive checks */
} SequencerInternalState;

/* Global sequencer state */
static SequencerInternalState seqState;

/* State name strings for debugging */
static const char* STATE_NAMES[] = {
    "INIT",
    "T0_WINDOW",
    "T1_REACHED",
    "T1_WINDOW",
    "T2_REACHED",
    "PITCH_YAW_ON",
    "GUIDANCE_ON",
    "T2_WINDOW",
    "T3_REACHED",
    "TERMINAL"
};

/* Private function prototypes */
static void LogStateTransition(SequencerState oldState, SequencerState newState);

/* Initialize the sequencer module */
Status Sequencer_Initialize(const SequencerConfig* config) {
    /* Initialize with default or provided configuration */
    if (config != NULL) {
        memcpy(&seqState.config, config, sizeof(SequencerConfig));
    } else {
        memcpy(&seqState.config, &DEFAULT_CONFIG, sizeof(SequencerConfig));
    }
    
    /* Initialize state */
    seqState.state = SEQ_STATE_INIT;
    seqState.missionTime = 0.0;
    seqState.t0Time = 0.0;
    seqState.t1Time = 0.0;
    seqState.t2Time = 0.0;
    seqState.t3Time = 0.0;
    
    /* Initialize flags */
    seqState.navActive = BOOL_FALSE;
    seqState.rollControlActive = BOOL_FALSE;
    seqState.pitchYawActive = BOOL_FALSE;
    seqState.guidanceActive = BOOL_FALSE;
    seqState.proximityActive = BOOL_FALSE;
    seqState.fsaFlag = BOOL_FALSE;
    seqState.dapFlag = BOOL_FALSE;
    seqState.imuInLoopFlag = BOOL_FALSE;
    
    /* Initialize simulated sensor data */
    seqState.rollRate = 10.0;  /* Initial roll rate above thresholds */
    seqState.gt3Flag = BOOL_FALSE;
    
    /* Initialize state transition tracking */
    seqState.consecutiveCount = 0;
    seqState.imuLoopConsecutiveCount = 0;
    
    printf("Sequencer initialized. Mission time: T+%.2f s\n", seqState.missionTime);
    
    return STATUS_OK;
}

/* Execute one step of the sequencer state machine */
Status Sequencer_Execute(double timeStep) {
    SequencerState oldState = seqState.state;
    
    /* Update mission time */
    seqState.missionTime += timeStep;
    
    /* State machine logic */
    switch (seqState.state) {
        case SEQ_STATE_INIT:
            /* T0 - OBC Reset */
            seqState.t0Time = seqState.missionTime;
            seqState.state = SEQ_STATE_T0_WINDOW;
            seqState.consecutiveCount = 0;
            break;
            
        case SEQ_STATE_T0_WINDOW:
            /* Check if we're in the sensing window */
            if (seqState.missionTime >= seqState.t0Time + seqState.config.windowStartOffset && 
                seqState.missionTime <= seqState.t0Time + seqState.config.windowStartOffset + seqState.config.windowDuration) {
                
                /* Check roll rate */
                if (seqState.rollRate < seqState.config.rollRateThreshold1) {
                    seqState.consecutiveCount++;
                    
                    /* Check if we have enough consecutive checks */
                    if (seqState.consecutiveCount >= seqState.config.consecutiveChecksRequired) {
                        seqState.state = SEQ_STATE_T1_REACHED;
                    }
                } else {
                    /* Reset consecutive count if condition not met */
                    seqState.consecutiveCount = 0;
                }
            } else if (seqState.missionTime > seqState.t0Time + seqState.config.windowStartOffset + seqState.config.windowDuration) {
                /* Window expired without meeting condition */
                printf("WARNING: T0 Window expired without meeting roll rate condition\n");
                /* In a real system, this might trigger a fault handling routine */
                /* For simulation, we'll continue with the next state anyway */
                seqState.state = SEQ_STATE_T1_REACHED;
            }
            break;
            
        case SEQ_STATE_T1_REACHED:
            /* T1 - Canard Deployment */
            seqState.t1Time = seqState.missionTime;
            
            /* Activate IMU in navigation */
            seqState.navActive = BOOL_TRUE;
            
            /* Set FSA flag for canard deployment */
            seqState.fsaFlag = BOOL_TRUE;
            printf("T1: Canard Deployment at T+%.2f s\n", seqState.missionTime);
            printf("FSA flag activated for canard deployment\n");
            
            /* Transition to T1 window */
            seqState.state = SEQ_STATE_T1_WINDOW;
            seqState.consecutiveCount = 0;
            break;
            
        case SEQ_STATE_T1_WINDOW:
            /* Check if we're in the sensing window */
            if (seqState.missionTime >= seqState.t1Time + seqState.config.windowStartOffset && 
                seqState.missionTime <= seqState.t1Time + seqState.config.windowStartOffset + seqState.config.windowDuration) {
                
                /* First check for IMU in the loop condition (roll rate < 5 rps) */
                if (seqState.rollRate < seqState.config.rollRateThreshold3 && !seqState.imuInLoopFlag) {
                    seqState.imuLoopConsecutiveCount++;
                    
                    /* Check if we have enough consecutive checks */
                    if (seqState.imuLoopConsecutiveCount >= seqState.config.consecutiveChecksRequired) {
                        seqState.imuInLoopFlag = BOOL_TRUE;
                        printf("IMU in the loop activated at T+%.2f s (roll rate < 5 rps)\n", seqState.missionTime);
                    }
                } else if (seqState.rollRate >= seqState.config.rollRateThreshold3 && !seqState.imuInLoopFlag) {
                    /* Reset consecutive count if condition not met */
                    seqState.imuLoopConsecutiveCount = 0;
                }
                
                /* Then check for T2 condition (roll rate < 2 rps) */
                if (seqState.rollRate < seqState.config.rollRateThreshold2) {
                    seqState.consecutiveCount++;
                    
                    /* Check if we have enough consecutive checks */
                    if (seqState.consecutiveCount >= seqState.config.consecutiveChecksRequired) {
                        seqState.state = SEQ_STATE_T2_REACHED;
                    }
                } else {
                    /* Reset consecutive count if condition not met */
                    seqState.consecutiveCount = 0;
                }
            } else if (seqState.missionTime > seqState.t1Time + seqState.config.windowStartOffset + seqState.config.windowDuration) {
                /* Window expired without meeting condition */
                printf("WARNING: T1 Window expired without meeting roll rate condition\n");
                /* In a real system, this might trigger a fault handling routine */
                /* For simulation, we'll continue with the next state anyway */
                seqState.state = SEQ_STATE_T2_REACHED;
            }
            break;
            
        case SEQ_STATE_T2_REACHED:
            /* T2 - Roll Control On */
            seqState.t2Time = seqState.missionTime;
            
            /* Activate roll control in DAP */
            seqState.rollControlActive = BOOL_TRUE;
            
            /* Set DAP flag for roll control */
            seqState.dapFlag = BOOL_TRUE;
            printf("T2: Roll Control Activated at T+%.2f s\n", seqState.missionTime);
            printf("DAP flag activated for roll control\n");
            
            /* Schedule next state transitions */
            seqState.state = SEQ_STATE_PITCH_YAW_ON;
            break;
            
        case SEQ_STATE_PITCH_YAW_ON:
            /* T2+Δt4 - Pitch and Yaw Control On */
            if (seqState.missionTime >= seqState.t2Time + seqState.config.dt4) {
                /* Activate pitch/yaw control in DAP */
                seqState.pitchYawActive = BOOL_TRUE;
                
                /* Update DAP flag for pitch/yaw control */
                printf("T2+Δt4: Pitch and Yaw Control Activated at T+%.2f s\n", seqState.missionTime);
                printf("DAP flag updated for pitch/yaw control\n");
                
                /* Transition to guidance activation */
                seqState.state = SEQ_STATE_GUIDANCE_ON;
            }
            break;
            
        case SEQ_STATE_GUIDANCE_ON:
            /* T2+2s - Guidance Initialization */
            if (seqState.missionTime >= seqState.t2Time + 2.0) {
                /* Activate guidance */
                seqState.guidanceActive = BOOL_TRUE;
                
                printf("T2+2s: Closed Loop Guidance Activated at T+%.2f s\n", seqState.missionTime);
                printf("Guidance flag activated\n");
                
                /* Transition to T2 window */
                seqState.state = SEQ_STATE_T2_WINDOW;
                seqState.consecutiveCount = 0;
            }
            break;
            
        case SEQ_STATE_T2_WINDOW:
            /* Check if we're in the sensing window */
            if (seqState.missionTime >= seqState.t2Time + seqState.config.windowStartOffset && 
                seqState.missionTime <= seqState.t2Time + seqState.config.windowStartOffset + seqState.config.windowDuration) {
                
                /* Check GT3 flag from guidance */
                if (seqState.gt3Flag) {
                    seqState.consecutiveCount++;
                    
                    /* Check if we have enough consecutive checks */
                    if (seqState.consecutiveCount >= seqState.config.consecutiveChecksRequired) {
                        seqState.state = SEQ_STATE_T3_REACHED;
                    }
                } else {
                    /* Reset consecutive count if condition not met */
                    seqState.consecutiveCount = 0;
                }
            } else if (seqState.missionTime > seqState.t2Time + seqState.config.windowStartOffset + seqState.config.windowDuration) {
                /* Window expired without meeting condition */
                printf("WARNING: T2 Window expired without receiving GT3 flag\n");
                /* In a real system, this might trigger a fault handling routine */
                /* For simulation, we'll continue with the next state anyway */
                seqState.state = SEQ_STATE_T3_REACHED;
            }
            break;
            
        case SEQ_STATE_T3_REACHED:
            /* T3 - Proximity Sensor Enable (3.5s before target) */
            seqState.t3Time = seqState.missionTime;
            
            /* Activate proximity sensor */
            seqState.proximityActive = BOOL_TRUE;
            
            printf("T3: Proximity Sensor Enabled at T+%.2f s (3.5s before target)\n", seqState.missionTime);
            
            /* Transition to terminal state */
            seqState.state = SEQ_STATE_TERMINAL;
            break;
            
        case SEQ_STATE_TERMINAL:
            /* Terminal phase - nothing more to do in sequencer */
            break;
            
        default:
            /* Invalid state */
            printf("Error: Invalid sequencer state\n");
            return STATUS_ERROR;
    }
    
    /* Log state transition if state changed */
    if (oldState != seqState.state) {
        LogStateTransition(oldState, seqState.state);
    }
    
    return STATUS_OK;
}

/* Get current sequencer status */
Status Sequencer_GetStatus(SequencerStatus* status) {
    if (status == NULL) {
        return STATUS_INVALID_PARAM;
    }
    
    status->state = seqState.state;
    status->missionTime = seqState.missionTime;
    status->t0Time = seqState.t0Time;
    status->t1Time = seqState.t1Time;
    status->t2Time = seqState.t2Time;
    status->t3Time = seqState.t3Time;
    status->navActive = seqState.navActive;
    status->rollControlActive = seqState.rollControlActive;
    status->pitchYawActive = seqState.pitchYawActive;
    status->guidanceActive = seqState.guidanceActive;
    status->proximityActive = seqState.proximityActive;
    status->fsaFlag = seqState.fsaFlag;
    status->dapFlag = seqState.dapFlag;
    status->imuInLoopFlag = seqState.imuInLoopFlag;
    
    return STATUS_OK;
}

/* Get current mission time */
double Sequencer_GetMissionTime(void) {
    return seqState.missionTime;
}

/* Check if navigation is active */
Bool Sequencer_IsNavigationActive(void) {
    return seqState.navActive;
}

/* Check if roll control is active */
Bool Sequencer_IsRollControlActive(void) {
    return seqState.rollControlActive;
}

/* Check if pitch/yaw control is active */
Bool Sequencer_IsPitchYawActive(void) {
    return seqState.pitchYawActive;
}

/* Check if guidance is active */
Bool Sequencer_IsGuidanceActive(void) {
    return seqState.guidanceActive;
}

/* Check if proximity sensor is active */
Bool Sequencer_IsProximityActive(void) {
    return seqState.proximityActive;
}

/* Check if FSA flag is active */
Bool Sequencer_IsFSAFlagActive(void) {
    return seqState.fsaFlag;
}

/* Check if DAP flag is active */
Bool Sequencer_IsDAPFlagActive(void) {
    return seqState.dapFlag;
}

/* Check if IMU in loop flag is active */
Bool Sequencer_IsIMUInLoopActive(void) {
    return seqState.imuInLoopFlag;
}

/* Get current sequencer state */
SequencerState Sequencer_GetState(void) {
    return seqState.state;
}

/* Get string representation of sequencer state */
const char* Sequencer_GetStateString(SequencerState state) {
    if (state >= SEQ_STATE_INIT && state <= SEQ_STATE_TERMINAL) {
        return STATE_NAMES[state];
    }
    return "UNKNOWN";
}

/*
 * Simulation-specific functions that will be replaced with real hardware interfaces
 */

/* Set the roll rate for simulation */
Status Sequencer_Sim_SetRollRate(double rollRate) {
    seqState.rollRate = rollRate;
    return STATUS_OK;
}

/* Set the GT3 flag from guidance for simulation */
Status Sequencer_Sim_SetGT3Flag(Bool gt3Flag) {
    seqState.gt3Flag = gt3Flag;
    return STATUS_OK;
}

/* Reset the sequencer to initial state (for simulation) */
Status Sequencer_Sim_Reset(void) {
    seqState.state = SEQ_STATE_INIT;
    seqState.missionTime = 0.0;
    seqState.t0Time = 0.0;
    seqState.t1Time = 0.0;
    seqState.t2Time = 0.0;
    seqState.t3Time = 0.0;
    
    seqState.navActive = BOOL_FALSE;
    seqState.rollControlActive = BOOL_FALSE;
    seqState.pitchYawActive = BOOL_FALSE;
    seqState.guidanceActive = BOOL_FALSE;
    seqState.proximityActive = BOOL_FALSE;
    seqState.fsaFlag = BOOL_FALSE;
    seqState.dapFlag = BOOL_FALSE;
    seqState.imuInLoopFlag = BOOL_FALSE;
    
    seqState.rollRate = 10.0;
    seqState.gt3Flag = BOOL_FALSE;
    
    seqState.consecutiveCount = 0;
    seqState.imuLoopConsecutiveCount = 0;
    
    printf("Sequencer reset to initial state\n");
    
    return STATUS_OK;
}

/* Private function implementations */

/* Log state transition */
static void LogStateTransition(SequencerState oldState, SequencerState newState) {
    printf("Sequencer state transition: %s -> %s at T0+ %f s\n",
           Sequencer_GetStateString(oldState),
           Sequencer_GetStateString(newState),
           seqState.missionTime);
    
    /* Log additional information based on the new state */
    switch (newState) {
        case SEQ_STATE_T0_WINDOW:
            printf("T0: OBC Reset, monitoring for roll rate < 7 rps\n");
            break;
            
        case SEQ_STATE_T1_REACHED:
            /* T1 message is now printed in the state handler */
            break;
            
        case SEQ_STATE_T1_WINDOW:
            printf("Monitoring for roll rate < 5 rps (IMU in loop) and < 2 rps (T2)\n");
            break;
            
        case SEQ_STATE_T2_REACHED:
            /* T2 message is now printed in the state handler */
            break;
            
        case SEQ_STATE_PITCH_YAW_ON:
            printf("Waiting for T2+(0.5s) to activate pitch/yaw control\n");
            break;
            
        case SEQ_STATE_GUIDANCE_ON:
            printf("Waiting for T2+(2s) to activate guidance\n");
            break;
            
        case SEQ_STATE_T2_WINDOW:
            printf("Monitoring for GT3 flag from guidance\n");
            break;
            
        case SEQ_STATE_T3_REACHED:
            /* T3 message is now printed in the state handler */
            break;
            
        default:
            break;
    }
}