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
    config->windowStartOffset = 0.0;
    config->windowDuration = 0.0;
    config->consecutiveChecksRequired = 1;
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

    /* T1: IMU loop and canard deployment criteria (roll rate < threshold1) */
    if (rollRateRps < g_config.rollRateThreshold1 && g_status.imuInLoopFlag == FALSE)
    {
        g_status.imuInLoopFlag = TRUE;
        g_status.t1Time = g_status.missionTime;
        g_status.state = SEQ_STATE_T1_REACHED;
        printStateTransition("[Canard Deployment] Roll Rate < 7 RPS");
    }

    /* Informational message around threshold3 while IMU loop is active */
    if (rollRateRps < g_config.rollRateThreshold3 && g_status.imuInLoopFlag == TRUE)
    {
        /* No state change, informational */
        /* Optional: throttle prints if integrating with real-time system */
        /* Here, print sparingly when crossing threshold3 */
        static Bool printedThreshold3 = FALSE;
        if (printedThreshold3 == FALSE)
        {
            printStateTransition("[IMU LOOP] Roll Rate < 5 RPS");
            printedThreshold3 = TRUE;
        }
    }

    /* T2: Enable roll control when roll rate < threshold2 */
    if (rollRateRps < g_config.rollRateThreshold2 && g_status.rollControlActive == FALSE)
    {
        g_status.rollControlActive = TRUE;
        g_status.dapFlag = TRUE;
        g_status.t2Time = g_status.missionTime;
        g_status.state = SEQ_STATE_T2_REACHED;
        printStateTransition("ROLL CONTROL ON (T2 Activated)");
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

    /* Guidance enable after 5 seconds from T2 (per reference behavior) */
    if (g_status.rollControlActive == TRUE && g_status.guidanceActive == FALSE)
    {
        if ((g_status.missionTime - g_status.t2Time) >= 5.0)
        {
            g_status.guidanceActive = TRUE;
            g_status.state = SEQ_STATE_GUIDANCE_ON;
            printStateTransition("Closed Loop Guidance Initialization");
        }
    }

    /* Placeholder: proximity and terminal logic could be added when targets available */
    if (g_simulatedGT3Flag == TRUE)
    {
        g_status.proximityActive = TRUE;
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


