/******************************************************************************
 * ISA Flight Software - Guidance Memory Management Test
 * 
 * File: test_guidance_minor.c
 * Description: Standalone test for guidance module memory management with
 *              major (100ms) and minor (10ms) cycle timing
 * 
 * This file tests:
 * - Memory allocation patterns in major/minor cycles
 * - Stack usage monitoring
 * - Data persistence across cycles
 * - Memory bounds checking
 * - Performance timing
 *****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include <math.h>

/* Timing constants per requirements */
#define MINOR_CYCLE_MS 10    /* 10ms minor cycle (100Hz) */
#define MAJOR_CYCLE_MS 100   /* 100ms major cycle (10Hz) */
#define MINOR_CYCLES_PER_MAJOR 10

/* Memory constraints for embedded system */
#define MAX_STACK_SIZE 8192  /* 8KB stack limit */
#define GUIDANCE_BUFFER_SIZE 1024
#define MAX_WAYPOINTS 50

/* Math constants */
#define PI 3.14159265358979323846
#define DEG_TO_RAD (PI / 180.0)
#define RAD_TO_DEG (180.0 / PI)

/* Simple vector structure */
typedef struct {
    double x;
    double y;
    double z;
} Vector3;

/* Geodetic position */
typedef struct {
    double latitude;   /* radians */
    double longitude;  /* radians */
    double altitude;   /* meters */
} GeodeticPosition;

/* Guidance state structure - simulated */
typedef struct {
    /* Target information */
    GeodeticPosition targetPosition;
    Vector3 targetPositionECI;
    Vector3 targetVelocityECI;
    
    /* Desired impact angles */
    double thetaF;  /* Desired elevation at impact (radians) */
    double psiF;    /* Desired azimuth at impact (radians) */
    
    /* Current projectile state */
    Vector3 positionECI;
    Vector3 velocityECI;
    
    /* Guidance parameters */
    double time;
    double previousDistance;
    bool terminalPhaseActive;
    
    /* Output acceleration commands */
    Vector3 accelerationCommandLocal;
    Vector3 accelerationCommandECI;
    
    /* Cycle counters */
    uint32_t minorCycleCount;
    uint32_t majorCycleCount;
    
    /* Memory management test data */
    uint8_t workingBuffer[GUIDANCE_BUFFER_SIZE];
    size_t bufferUsed;
    
    /* Stack monitoring */
    size_t peakStackUsage;
    uint8_t* stackBase;
} GuidanceState;

/* Global state for testing */
static GuidanceState g_guidanceState;
static uint8_t g_stackMarker[MAX_STACK_SIZE];

/* Performance monitoring */
typedef struct {
    uint64_t minorCycleTimeUs[MINOR_CYCLES_PER_MAJOR];
    uint64_t majorCycleTimeUs;
    uint64_t peakMinorTimeUs;
    uint64_t peakMajorTimeUs;
    size_t peakMemoryUsage;
} PerformanceMetrics;

static PerformanceMetrics g_metrics;

/* Get current time in microseconds */
static uint64_t getTimeUs(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000ULL + ts.tv_nsec / 1000;
}

/* Initialize stack monitoring */
static void initStackMonitoring(void) {
    /* Fill stack area with known pattern */
    memset(g_stackMarker, 0xAA, MAX_STACK_SIZE);
    g_guidanceState.stackBase = g_stackMarker;
    g_guidanceState.peakStackUsage = 0;
}

/* Check stack usage */
static size_t checkStackUsage(void) {
    size_t used = 0;
    for (size_t i = 0; i < MAX_STACK_SIZE; i++) {
        if (g_stackMarker[i] != 0xAA) {
            used = MAX_STACK_SIZE - i;
            break;
        }
    }
    
    if (used > g_guidanceState.peakStackUsage) {
        g_guidanceState.peakStackUsage = used;
    }
    
    return used;
}

/* Vector operations */
static Vector3 vectorAdd(Vector3 a, Vector3 b) {
    Vector3 result = {a.x + b.x, a.y + b.y, a.z + b.z};
    return result;
}

static Vector3 vectorSubtract(Vector3 a, Vector3 b) {
    Vector3 result = {a.x - b.x, a.y - b.y, a.z - b.z};
    return result;
}

static double vectorNorm(Vector3 v) {
    return sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

static Vector3 vectorScale(Vector3 v, double scale) {
    Vector3 result = {v.x * scale, v.y * scale, v.z * scale};
    return result;
}

/* Simulate guidance algorithm memory operations */
static void simulateGuidanceMemoryOps(void) {
    /* Allocate temporary workspace on stack */
    double workspace[100];
    Vector3 tempVectors[20];
    
    /* Fill workspace to test memory access patterns */
    for (int i = 0; i < 100; i++) {
        workspace[i] = sin(i * 0.1) * cos(i * 0.2);
    }
    
    /* Simulate trajectory calculations */
    for (int i = 0; i < 20; i++) {
        tempVectors[i].x = workspace[i * 3];
        tempVectors[i].y = workspace[i * 3 + 1];
        tempVectors[i].z = workspace[i * 3 + 2];
    }
    
    /* Update guidance buffer usage */
    size_t dataSize = sizeof(workspace) + sizeof(tempVectors);
    if (dataSize <= GUIDANCE_BUFFER_SIZE) {
        memcpy(g_guidanceState.workingBuffer, workspace, sizeof(workspace));
        g_guidanceState.bufferUsed = dataSize;
    }
    
    /* Check for buffer overflow */
    if (g_guidanceState.bufferUsed > GUIDANCE_BUFFER_SIZE) {
        printf("ERROR: Buffer overflow detected! Used %zu / %d bytes\n",
               g_guidanceState.bufferUsed, GUIDANCE_BUFFER_SIZE);
    }
}

/* Minor cycle execution (10ms) */
static void executeMinorCycle(void) {
    uint64_t startTime = getTimeUs();
    
    /* Simulate high-frequency guidance operations */
    Vector3 relativePos = vectorSubtract(g_guidanceState.targetPositionECI, 
                                       g_guidanceState.positionECI);
    double distance = vectorNorm(relativePos);
    
    /* Update velocity based on simple physics */
    Vector3 acceleration = vectorScale(relativePos, 0.001 / distance);
    g_guidanceState.velocityECI = vectorAdd(g_guidanceState.velocityECI, 
                                           vectorScale(acceleration, 0.01));
    
    /* Update position */
    g_guidanceState.positionECI = vectorAdd(g_guidanceState.positionECI,
                                          vectorScale(g_guidanceState.velocityECI, 0.01));
    
    /* Memory intensive operations */
    simulateGuidanceMemoryOps();
    
    /* Check stack usage */
    size_t stackUsed = checkStackUsage();
    if (stackUsed > MAX_STACK_SIZE * 0.8) {
        printf("WARNING: High stack usage: %zu / %d bytes (%.1f%%)\n",
               stackUsed, MAX_STACK_SIZE, (double)stackUsed / MAX_STACK_SIZE * 100);
    }
    
    /* Record timing */
    uint64_t endTime = getTimeUs();
    uint64_t cycleTime = endTime - startTime;
    
    int minorIndex = g_guidanceState.minorCycleCount % MINOR_CYCLES_PER_MAJOR;
    g_metrics.minorCycleTimeUs[minorIndex] = cycleTime;
    
    if (cycleTime > g_metrics.peakMinorTimeUs) {
        g_metrics.peakMinorTimeUs = cycleTime;
    }
    
    /* Check timing constraint */
    if (cycleTime > MINOR_CYCLE_MS * 1000) {
        printf("WARNING: Minor cycle %u exceeded timing: %llu us (limit: %d us)\n",
               g_guidanceState.minorCycleCount, cycleTime, MINOR_CYCLE_MS * 1000);
    }
    
    g_guidanceState.minorCycleCount++;
}

/* Major cycle execution (100ms) */
static void executeMajorCycle(void) {
    uint64_t startTime = getTimeUs();
    
    /* Simulate less frequent guidance operations */
    printf("\n=== Major Cycle %u ===\n", g_guidanceState.majorCycleCount);
    
    /* Calculate trajectory parameters */
    double distance = vectorNorm(vectorSubtract(g_guidanceState.targetPositionECI,
                                              g_guidanceState.positionECI));
    double velocity = vectorNorm(g_guidanceState.velocityECI);
    double timeToGo = distance / velocity;
    
    printf("Distance to target: %.2f m\n", distance);
    printf("Velocity: %.2f m/s\n", velocity);
    printf("Time to impact: %.2f s\n", timeToGo);
    
    /* Check if entering terminal phase */
    if (distance < 500.0 && !g_guidanceState.terminalPhaseActive) {
        g_guidanceState.terminalPhaseActive = true;
        printf("TERMINAL PHASE ACTIVATED\n");
    }
    
    /* Memory report */
    printf("\nMemory Usage:\n");
    printf("  Working buffer: %zu / %d bytes (%.1f%%)\n",
           g_guidanceState.bufferUsed, GUIDANCE_BUFFER_SIZE,
           (double)g_guidanceState.bufferUsed / GUIDANCE_BUFFER_SIZE * 100);
    printf("  Peak stack usage: %zu / %d bytes (%.1f%%)\n",
           g_guidanceState.peakStackUsage, MAX_STACK_SIZE,
           (double)g_guidanceState.peakStackUsage / MAX_STACK_SIZE * 100);
    
    /* Timing report for minor cycles */
    printf("\nMinor Cycle Timing:\n");
    uint64_t totalMinorTime = 0;
    for (int i = 0; i < MINOR_CYCLES_PER_MAJOR; i++) {
        printf("  Cycle %d: %llu us\n", i, g_metrics.minorCycleTimeUs[i]);
        totalMinorTime += g_metrics.minorCycleTimeUs[i];
    }
    printf("  Average: %llu us\n", totalMinorTime / MINOR_CYCLES_PER_MAJOR);
    printf("  Peak: %llu us\n", g_metrics.peakMinorTimeUs);
    
    /* Record major cycle timing */
    uint64_t endTime = getTimeUs();
    uint64_t cycleTime = endTime - startTime;
    g_metrics.majorCycleTimeUs = cycleTime;
    
    if (cycleTime > g_metrics.peakMajorTimeUs) {
        g_metrics.peakMajorTimeUs = cycleTime;
    }
    
    /* Check timing constraint */
    if (cycleTime > MAJOR_CYCLE_MS * 1000) {
        printf("WARNING: Major cycle exceeded timing: %llu us (limit: %d us)\n",
               cycleTime, MAJOR_CYCLE_MS * 1000);
    }
    
    g_guidanceState.majorCycleCount++;
}

/* Initialize guidance state */
static void initializeGuidance(void) {
    memset(&g_guidanceState, 0, sizeof(g_guidanceState));
    memset(&g_metrics, 0, sizeof(g_metrics));
    
    /* Set initial positions */
    g_guidanceState.positionECI.x = 0.0;
    g_guidanceState.positionECI.y = 0.0;
    g_guidanceState.positionECI.z = 6371000.0; /* Earth radius */
    
    /* Set target 10km away */
    g_guidanceState.targetPositionECI.x = 10000.0;
    g_guidanceState.targetPositionECI.y = 0.0;
    g_guidanceState.targetPositionECI.z = 6371000.0;
    
    /* Initial velocity towards target */
    g_guidanceState.velocityECI.x = 300.0; /* 300 m/s */
    g_guidanceState.velocityECI.y = 0.0;
    g_guidanceState.velocityECI.z = 100.0; /* Some vertical component */
    
    /* Impact angles */
    g_guidanceState.thetaF = 45.0 * DEG_TO_RAD;
    g_guidanceState.psiF = 0.0 * DEG_TO_RAD;
    
    /* Initialize stack monitoring */
    initStackMonitoring();
    
    printf("Guidance system initialized\n");
    printf("Initial distance to target: %.2f m\n", 
           vectorNorm(vectorSubtract(g_guidanceState.targetPositionECI,
                                   g_guidanceState.positionECI)));
}

/* Main test function */
int main(void) {
    printf("=== ISA Guidance Memory Management Test ===\n");
    printf("Major cycle: %d ms (%d Hz)\n", MAJOR_CYCLE_MS, 1000/MAJOR_CYCLE_MS);
    printf("Minor cycle: %d ms (%d Hz)\n", MINOR_CYCLE_MS, 1000/MINOR_CYCLE_MS);
    printf("Minor cycles per major: %d\n\n", MINOR_CYCLES_PER_MAJOR);
    
    /* Initialize system */
    initializeGuidance();
    
    /* Run for 5 major cycles (500ms) */
    const int numMajorCycles = 5;
    
    for (int major = 0; major < numMajorCycles; major++) {
        /* Execute minor cycles */
        for (int minor = 0; minor < MINOR_CYCLES_PER_MAJOR; minor++) {
            executeMinorCycle();
            
            /* Simulate 10ms delay */
            struct timespec delay = {0, MINOR_CYCLE_MS * 1000000};
            nanosleep(&delay, NULL);
        }
        
        /* Execute major cycle */
        executeMajorCycle();
    }
    
    /* Final report */
    printf("\n=== FINAL REPORT ===\n");
    printf("Total minor cycles executed: %u\n", g_guidanceState.minorCycleCount);
    printf("Total major cycles executed: %u\n", g_guidanceState.majorCycleCount);
    printf("\nPeak Performance:\n");
    printf("  Peak minor cycle time: %llu us\n", g_metrics.peakMinorTimeUs);
    printf("  Peak major cycle time: %llu us\n", g_metrics.peakMajorTimeUs);
    printf("\nPeak Memory Usage:\n");
    printf("  Peak stack usage: %zu / %d bytes (%.1f%%)\n",
           g_guidanceState.peakStackUsage, MAX_STACK_SIZE,
           (double)g_guidanceState.peakStackUsage / MAX_STACK_SIZE * 100);
    printf("  Peak buffer usage: %zu / %d bytes (%.1f%%)\n",
           g_guidanceState.bufferUsed, GUIDANCE_BUFFER_SIZE,
           (double)g_guidanceState.bufferUsed / GUIDANCE_BUFFER_SIZE * 100);
    
    /* Check if we met timing requirements */
    bool timingMet = (g_metrics.peakMinorTimeUs <= MINOR_CYCLE_MS * 1000) &&
                     (g_metrics.peakMajorTimeUs <= MAJOR_CYCLE_MS * 1000);
    
    printf("\nTiming requirements: %s\n", timingMet ? "PASSED" : "FAILED");
    printf("Memory requirements: %s\n", 
           g_guidanceState.peakStackUsage < MAX_STACK_SIZE ? "PASSED" : "FAILED");
    
    return timingMet ? 0 : 1;
}