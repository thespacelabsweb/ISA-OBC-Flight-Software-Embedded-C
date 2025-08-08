/******************************************************************************
 * ISA Flight Software
 * 
 * File: main_sequencer_sim.c
 * Description: Main program for real-time sequencer simulation
 *****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <string.h>

/* Include our headers first to avoid conflicts */
#include "include/common/types.h"
#include "include/sequencer/sequencer.h"

/* System headers after our headers */
#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#include <sys/time.h>
#endif

/* Function to simulate decreasing roll rate over time */
static double SimulateRollRate(double initialRate, double decayFactor, double elapsedTime) {
    return initialRate * pow(decayFactor, elapsedTime);
}

/* Platform-independent sleep function (milliseconds) */
static void SleepMs(unsigned int milliseconds) {
#ifdef _WIN32
    Sleep(milliseconds);
#else
    usleep(milliseconds * 1000);
#endif
}

/* Platform-independent high-precision time function (milliseconds) */
static double GetTimeMs(void) {
#ifdef _WIN32
    LARGE_INTEGER frequency;
    LARGE_INTEGER count;
    QueryPerformanceFrequency(&frequency);
    QueryPerformanceCounter(&count);
    return (double)count.QuadPart * 1000.0 / (double)frequency.QuadPart;
#else
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (double)tv.tv_sec * 1000.0 + (double)tv.tv_usec / 1000.0;
#endif
}

/* Calculate CPU load percentage */
static double CalculateCpuLoad(double processingTimeMs, double cycleTimeMs) {
    return (processingTimeMs / cycleTimeMs) * 100.0;
}

int main(void) {
    Status status;
    SequencerStatus seqStatus;
    
    /* Initialize sequencer with default configuration */
    status = Sequencer_Initialize(NULL);
    if (status != STATUS_OK) {
        printf("Error initializing sequencer\n");
        return 1;
    }
    
    /* Real-time execution parameters */
    const double CYCLE_TIME_MS = 10.0;      /* 10ms cycle time (100Hz) */
    const double MAX_RUNTIME_SEC = 30.0;     /* 30 seconds max runtime */
    
    /* Roll rate simulation parameters */
    double initialRollRate = 10.0;           /* Initial roll rate in rps */
    double rollRateDecayFactor = 0.95;       /* Roll rate decay per second */
    double currentRollRate = initialRollRate;
    
    /* Performance monitoring */
    double cycleStartTimeMs;
    double processingTimeMs;
    double sleepTimeMs;
    double cpuLoad;
    double maxCpuLoad = 0.0;
    double avgCpuLoad = 0.0;
    int cycleCount = 0;
    int missedDeadlines = 0;
    
    /* Timing statistics */
    double minProcessingTimeMs = 1000000.0;
    double maxProcessingTimeMs = 0.0;
    double totalProcessingTimeMs = 0.0;
    
    printf("\n=== REAL-TIME SEQUENCER SIMULATION START ===\n\n");
    printf("Target cycle time: %.2f ms (%.1f Hz)\n", CYCLE_TIME_MS, 1000.0 / CYCLE_TIME_MS);
    
    /* Get initial time */
    double startTimeMs = GetTimeMs();
    double currentTimeMs;
    double elapsedTimeSec;
    
    /* Main real-time execution loop */
    while (BOOL_TRUE) {
        /* Record cycle start time */
        cycleStartTimeMs = GetTimeMs();
        
        /* Calculate elapsed time in seconds */
        currentTimeMs = cycleStartTimeMs;
        elapsedTimeSec = (currentTimeMs - startTimeMs) / 1000.0;
        
        /* Check if we've reached the maximum runtime */
        if (elapsedTimeSec >= MAX_RUNTIME_SEC) {
            printf("\nMaximum runtime reached (%.2f seconds)\n", MAX_RUNTIME_SEC);
            break;
        }
        
        /* Update simulated roll rate */
        currentRollRate = SimulateRollRate(initialRollRate, rollRateDecayFactor, elapsedTimeSec);
        Sequencer_Sim_SetRollRate(currentRollRate);
        
        /* Set GT3 flag when close to target (after 20 seconds) */
        if (elapsedTimeSec > 20.0) {
            Sequencer_Sim_SetGT3Flag(BOOL_TRUE);
        }
        
        /* Execute sequencer */
        status = Sequencer_Execute(CYCLE_TIME_MS / 1000.0);  /* Convert to seconds */
        if (status != STATUS_OK) {
            printf("Error executing sequencer\n");
            break;
        }
        
        /* Get sequencer status */
        Sequencer_GetStatus(&seqStatus);
        
        /* Print status every second */
        if (fmod(elapsedTimeSec, 1.0) < CYCLE_TIME_MS / 1000.0) {
            printf("T+%.2f: State=%s, Roll=%.2f rps\n",
                   seqStatus.missionTime,
                   Sequencer_GetStateString(seqStatus.state),
                   currentRollRate);
            printf("  Flags: Nav=%d, Roll=%d, PitchYaw=%d, Guidance=%d, Proximity=%d, FSA=%d, DAP=%d, IMU=%d\n",
                   seqStatus.navActive,
                   seqStatus.rollControlActive,
                   seqStatus.pitchYawActive,
                   seqStatus.guidanceActive,
                   seqStatus.proximityActive,
                   seqStatus.fsaFlag,
                   seqStatus.dapFlag,
                   seqStatus.imuInLoopFlag);
        }
        
        /* Check if we've reached the terminal state */
        if (seqStatus.state == SEQ_STATE_TERMINAL) {
            printf("\nReached terminal state. Simulation complete.\n");
            break;
        }
        
        /* Calculate processing time */
        processingTimeMs = GetTimeMs() - cycleStartTimeMs;
        
        /* Update timing statistics */
        if (processingTimeMs < minProcessingTimeMs) {
            minProcessingTimeMs = processingTimeMs;
        }
        if (processingTimeMs > maxProcessingTimeMs) {
            maxProcessingTimeMs = processingTimeMs;
        }
        totalProcessingTimeMs += processingTimeMs;
        cycleCount++;
        
        /* Calculate CPU load for this cycle */
        cpuLoad = CalculateCpuLoad(processingTimeMs, CYCLE_TIME_MS);
        avgCpuLoad += cpuLoad;
        if (cpuLoad > maxCpuLoad) {
            maxCpuLoad = cpuLoad;
        }
        
        /* Calculate sleep time to maintain cycle rate */
        sleepTimeMs = CYCLE_TIME_MS - processingTimeMs;
        
        /* Check for missed deadline */
        if (sleepTimeMs < 0) {
            missedDeadlines++;
            /* We missed the deadline, continue immediately to the next cycle */
            continue;
        }
        
        /* Sleep for the remaining time in the cycle */
        SleepMs((unsigned int)sleepTimeMs);
    }
    
    /* Calculate final statistics */
    avgCpuLoad /= (double)cycleCount;
    
    /* Print performance statistics */
    printf("\n=== REAL-TIME SEQUENCER SIMULATION COMPLETE ===\n");
    printf("Final state: %s\n", Sequencer_GetStateString(Sequencer_GetState()));
    printf("Mission time: T+%.2f s\n", Sequencer_GetMissionTime());
    printf("\nPerformance Statistics:\n");
    printf("Total cycles executed: %d\n", cycleCount);
    printf("Missed deadlines: %d (%.2f%%)\n", missedDeadlines, (double)missedDeadlines * 100.0 / (double)cycleCount);
    printf("Processing time - Min: %.3f ms, Max: %.3f ms, Avg: %.3f ms\n",
           minProcessingTimeMs, maxProcessingTimeMs, totalProcessingTimeMs / (double)cycleCount);
    printf("CPU load - Avg: %.2f%%, Max: %.2f%%\n", avgCpuLoad, maxCpuLoad);
    
    return 0;
}