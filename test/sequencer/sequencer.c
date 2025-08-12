/* Define POSIX feature test macro for nanosleep/timespec on POSIX systems */
#ifndef _WIN32
#ifndef _POSIX_C_SOURCE
#define _POSIX_C_SOURCE 199309L
#endif
#endif

/******************************************************************************
 * ISA Flight Software
 *
 * File: sequencer.c
 * Description: Sequencer test module with simple simulation harness
 *****************************************************************************/

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#include <time.h>
#ifndef _WIN32
#include <unistd.h>
#endif

#include "common/types.h"
#include "sequencer/sequencer.h"

/*----------------------------------------------------------------------------*/
/* Module state                                                               */
/*----------------------------------------------------------------------------*/

static SequencerConfig g_config;
static SequencerStatus g_status;

/* Simulation inputs */
static double g_simulatedRollRateRps = 0.0;
static Bool   g_simulatedGT3Flag = FALSE;

/* Consecutive detection counters */
static int g_t1ConsecutiveCount = 0;
static int g_imuConsecutiveCount = 0;
static int g_t2ConsecutiveCount = 0;
static int g_t3ConsecutiveCount = 0;

/* Timing constants (test harness) */
static const double GUIDANCE_INIT_DELAY_SEC = 2.0; /* T2 + 2s */

/*----------------------------------------------------------------------------*/
/* Internal utilities                                                         */
/*----------------------------------------------------------------------------*/

static void setDefaultConfig(SequencerConfig* config)
{
    /* Defaults derived from provided reference behavior */
    config->dt4 = 2.0;                    /* Pitch/Yaw enable delay after T2 */
    config->rollRateThreshold1 = 7.0;     /* T1 (canard deploy) threshold */
    config->rollRateThreshold2 = 2.0;     /* T2 (roll control) threshold */
    config->rollRateThreshold3 = 5.0;     /* IMU-in-loop message threshold */
    config->windowStartOffset = 0.1;      /* Window opens 0.1 s after anchor */
    config->windowDuration = 5.0;         /* Window lasts 5.0 s */
    config->t3WindowStartOffset = 0.1;    /* T3 window opens 0.1 s after T2 */
    config->t3WindowDuration = 5.0;       /* T3 window lasts 5.0 s */
    config->consecutiveChecksRequired = 3;/* Require 3 consecutive cycles */
    config->proximityTimeBeforeTarget = 5.0; /* Default placeholder */
}

static void resetStatus(SequencerStatus* status)
{
    memset(status, 0, sizeof(*status));
    status->state = SEQ_STATE_INIT;
    status->missionTime = 0.0;
    status->t0Time = 0.0;
    status->t1Time = 0.0;
    status->t2Time = 0.0;
    status->t3Time = 0.0;
    status->navActive = FALSE;
    status->rollControlActive = FALSE;
    status->pitchYawActive = FALSE;
    status->guidanceActive = FALSE;
    status->proximityActive = FALSE;
    status->fsaFlag = FALSE;
    status->dapFlag = FALSE;
    status->imuInLoopFlag = FALSE;
}

static void printStateTransition(const char* message)
{
    printf(" [sequence] %s\n", message);
}

static Bool isWithinWindow(double currentTime, double anchorTime)
{
    const double startTime = anchorTime + g_config.windowStartOffset;
    const double endTime = startTime + g_config.windowDuration;
    return (currentTime >= startTime) && (currentTime <= endTime) ? TRUE : FALSE;
}

static Bool isWithinT3Window(double currentTime, double anchorTime)
{
    const double startTime = anchorTime + g_config.t3WindowStartOffset;
    const double endTime = startTime + g_config.t3WindowDuration;
    return (currentTime >= startTime) && (currentTime <= endTime) ? TRUE : FALSE;
}

/*----------------------------------------------------------------------------*/
/* Public API                                                                 */
/*----------------------------------------------------------------------------*/

Status Sequencer_Initialize(const SequencerConfig* config)
{
    if (config == NULL)
    {
        setDefaultConfig(&g_config);
    }
    else
    {
        g_config = *config;
    }

    resetStatus(&g_status);
    g_status.state = SEQ_STATE_T0_WINDOW; /* Immediately enter sensing window */
    g_status.t0Time = 0.0;
    return STATUS_OK;
}

Status Sequencer_Execute(double timeStep)
{
    if (timeStep <= 0.0 || !isfinite(timeStep))
    {
        return STATUS_INVALID_PARAM;
    }

    g_status.missionTime += timeStep;

    /* Read simulated inputs */
    const double rollRateRps = g_simulatedRollRateRps;

    /*
     * T1 window: from T0 + offset to T0 + offset + duration
     * Condition: roll rate < 7 rps for N consecutive cycles
     * Action: set FSA flag, mark T1 time/state
     */
    if (g_status.t1Time == 0.0)
    {
        if (isWithinWindow(g_status.missionTime, g_status.t0Time))
        {
            if (rollRateRps < g_config.rollRateThreshold1)
            {
                g_t1ConsecutiveCount++;
                if (g_t1ConsecutiveCount >= g_config.consecutiveChecksRequired)
                {
                    g_status.t1Time = g_status.missionTime;
                    g_status.fsaFlag = TRUE;
                    g_status.state = SEQ_STATE_T1_REACHED;
                    printStateTransition("[Canard Deployment] Roll Rate < 7 RPS (T1), flag to FSA");
                }
            }
            else
            {
                g_t1ConsecutiveCount = 0;
            }
        }
        else
        {
            g_t1ConsecutiveCount = 0;
        }
    }

    /* IMU in loop: roll rate < 5 rps for N consecutive cycles (independent flag) */
    if (g_status.imuInLoopFlag == FALSE)
    {
        if (rollRateRps < g_config.rollRateThreshold3)
        {
            g_imuConsecutiveCount++;
            if (g_imuConsecutiveCount >= g_config.consecutiveChecksRequired)
            {
                g_status.imuInLoopFlag = TRUE;
                printStateTransition("[IMU LOOP] Roll Rate < 5 RPS (active)");
            }
        }
        else
        {
            g_imuConsecutiveCount = 0;
        }
    }

    /*
     * T2 window: from T1 + offset to T1 + offset + duration
     * Condition: roll rate < 2 rps for N consecutive cycles
     * Action: DAP flag, roll control ON, mark T2 time/state
     */
    if (g_status.t1Time > 0.0 && g_status.t2Time == 0.0)
    {
        if (isWithinWindow(g_status.missionTime, g_status.t1Time))
        {
            if (rollRateRps < g_config.rollRateThreshold2)
            {
                g_t2ConsecutiveCount++;
                if (g_t2ConsecutiveCount >= g_config.consecutiveChecksRequired)
                {
                    g_status.rollControlActive = TRUE;
                    g_status.dapFlag = TRUE;
                    g_status.t2Time = g_status.missionTime;
                    g_status.state = SEQ_STATE_T2_REACHED;
                    printStateTransition("ROLL CONTROL ON (T2 Activated)");
                }
            }
            else
            {
                g_t2ConsecutiveCount = 0;
            }
        }
        else
        {
            g_t2ConsecutiveCount = 0;
        }
    }

    /* Pitch/Yaw enable after dt4 seconds from T2 */
    if (g_status.rollControlActive == TRUE && g_status.pitchYawActive == FALSE)
    {
        if ((g_status.missionTime - g_status.t2Time) >= g_config.dt4)
        {
            g_status.pitchYawActive = TRUE;
            g_status.state = SEQ_STATE_PITCH_YAW_ON;
            printStateTransition("Pitch And Yaw Control ON");
        }
    }

    /* Guidance enable after 2 seconds from T2 (per requirements) */
    if (g_status.rollControlActive == TRUE && g_status.guidanceActive == FALSE)
    {
        if ((g_status.missionTime - g_status.t2Time) >= GUIDANCE_INIT_DELAY_SEC)
        {
            g_status.guidanceActive = TRUE;
            g_status.state = SEQ_STATE_GUIDANCE_ON;
            printStateTransition("Closed Loop Guidance Initialization");
        }
    }

    /*
     * T3 window: from T2 + offset to T2 + offset + duration
     * Condition: receive GT3 flag for N consecutive cycles
     * Action: mark T3 time/state, enable proximity
     */
    if (g_status.t2Time > 0.0 && g_status.t3Time == 0.0)
    {
        if (isWithinT3Window(g_status.missionTime, g_status.t2Time))
        {
            if (g_simulatedGT3Flag == TRUE)
            {
                g_t3ConsecutiveCount++;
                if (g_t3ConsecutiveCount >= g_config.consecutiveChecksRequired)
                {
                    g_status.t3Time = g_status.missionTime;
                    g_status.proximityActive = TRUE;
                    g_status.state = SEQ_STATE_T3_REACHED;
                    printStateTransition("Proximity Sensor Enabled (T3 REACHED)");
                }
            }
            else
            {
                g_t3ConsecutiveCount = 0;
            }
        }
        else
        {
            g_t3ConsecutiveCount = 0;
        }
    }

    return STATUS_OK;
}

Status Sequencer_GetStatus(SequencerStatus* status)
{
    if (status == NULL)
    {
        return STATUS_INVALID_PARAM;
    }
    *status = g_status;
    return STATUS_OK;
}

double Sequencer_GetMissionTime(void) { return g_status.missionTime; }
Bool   Sequencer_IsNavigationActive(void) { return g_status.navActive; }
Bool   Sequencer_IsRollControlActive(void) { return g_status.rollControlActive; }
Bool   Sequencer_IsPitchYawActive(void) { return g_status.pitchYawActive; }
Bool   Sequencer_IsGuidanceActive(void) { return g_status.guidanceActive; }
Bool   Sequencer_IsProximityActive(void) { return g_status.proximityActive; }
Bool   Sequencer_IsFSAFlagActive(void) { return g_status.fsaFlag; }
Bool   Sequencer_IsDAPFlagActive(void) { return g_status.dapFlag; }
Bool   Sequencer_IsIMUInLoopActive(void) { return g_status.imuInLoopFlag; }
SequencerState Sequencer_GetState(void) { return g_status.state; }

const char* Sequencer_GetStateString(SequencerState state)
{
    switch (state)
    {
        case SEQ_STATE_INIT: return "INIT";
        case SEQ_STATE_T0_WINDOW: return "T0_WINDOW";
        case SEQ_STATE_T1_REACHED: return "T1_REACHED";
        case SEQ_STATE_T1_WINDOW: return "T1_WINDOW";
        case SEQ_STATE_T2_REACHED: return "T2_REACHED";
        case SEQ_STATE_PITCH_YAW_ON: return "PITCH_YAW_ON";
        case SEQ_STATE_GUIDANCE_ON: return "GUIDANCE_ON";
        case SEQ_STATE_T2_WINDOW: return "T2_WINDOW";
        case SEQ_STATE_T3_REACHED: return "T3_REACHED";
        case SEQ_STATE_TERMINAL: return "TERMINAL";
        default: return "UNKNOWN";
    }
}

Status Sequencer_Sim_SetRollRate(double rollRate)
{
    if (!isfinite(rollRate) || rollRate < 0.0)
    {
        return STATUS_INVALID_PARAM;
    }
    g_simulatedRollRateRps = rollRate;
    return STATUS_OK;
}

Status Sequencer_Sim_SetGT3Flag(Bool gt3Flag)
{
    g_simulatedGT3Flag = gt3Flag;
    return STATUS_OK;
}

Status Sequencer_Sim_Reset(void)
{
    g_simulatedRollRateRps = 0.0;
    g_simulatedGT3Flag = FALSE;
    return STATUS_OK;
}

/*----------------------------------------------------------------------------*/
/* Simple test harness (executable entry point)                               */
/*----------------------------------------------------------------------------*/

int main(void)
{
    Sequencer_Initialize(NULL);

    /* Simulation parameters */
    enum { MINOR_CYCLE_US = 1000 };       /* 1 ms minor cycle */
    enum { PRINT_INTERVAL_US = 100000 };  /* Print every 100 ms */
    const double durationSec = 30.0;      /* Total simulation time */
    const double rollRateDecayPerSecond = 1.2; /* RPS/s */

    /* State for simulation */
    double rollRateRps = 10.0;           /* Start at 10 RPS */
    const uint64_t durationUs = (uint64_t)(durationSec * 1000000.0);

    for (uint64_t simTimeUs = 0; simTimeUs <= durationUs; simTimeUs += MINOR_CYCLE_US)
    {
        const double dtSec = (double)MINOR_CYCLE_US / 1000000.0;

        /* Update simulated roll rate */
        Sequencer_Sim_SetRollRate(rollRateRps);

        /* Execute sequencer step */
        Sequencer_Execute(dtSec);

        /* Periodic status print */
        if ((simTimeUs % PRINT_INTERVAL_US) == 0U)
        {
            SequencerStatus status;
            Sequencer_GetStatus(&status);
            printf("t=%.3fs, roll=%.2f rps, state=%s\n",
                   status.missionTime,
                   rollRateRps,
                   Sequencer_GetStateString(status.state));
        }

        /* Apply decay to roll rate */
        rollRateRps -= rollRateDecayPerSecond * dtSec;
        if (rollRateRps < 0.0)
        {
            rollRateRps = 0.0;
        }

        /* Optional: pace the loop to approximate real time */
        {
            struct timespec ts;
            ts.tv_sec = (time_t)(MINOR_CYCLE_US / 1000000U);
            ts.tv_nsec = (long)((MINOR_CYCLE_US % 1000000U) * 1000U);
            nanosleep(&ts, NULL);
        }
    }

    return 0;
}


