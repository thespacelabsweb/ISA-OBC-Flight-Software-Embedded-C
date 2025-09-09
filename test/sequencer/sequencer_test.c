#include <stdio.h>
#include <unistd.h> // For usleep()
#include <stdbool.h>
#include <math.h>   // For fabs()

#include "sequencer.h" // Your sequencer's header file

// --- Simulation Parameters ---
#define SIMULATION_STEP_MS 10    // Minor cycle is 10 ms (100 Hz)
#define SIMULATION_DURATION_S 60 // Run the simulation for 60 seconds

int main(void)
{
    printf("--- Sequencer Simulation Started ---\n");

    // 1. Storage for Sequencer State and Output
    SequencerState_t state;
    SequencerOutput_t output;

    // 2. Initialize the sequencer
    sequencerInit(&state);

    // --- Simulation Inputs ---
    // These variables represent the "real world" for the sequencer
    float rollRateFp = 6000.0f; // Start with a high roll rate of 6000.0 rps
    uint32_t tGo = 0;           // Guidance tGo signal is initially 0

    // --- Interactive Launch ---
    printf("Projectile is in PRE-LAUNCH state. Waiting for launch command.\n");
    printf("Fire the Projectile? (y/n): ");
    while (getchar() != 'y')
    {
        // Wait for user to press 'y'
    }
    printf("\nLaunch Command Received! T=0\n\n");
    sequencerSetGswitch(&state, true); // Trigger the launch

    // --- Main Simulation Loop ---
    int total_cycles = SIMULATION_DURATION_S * (1000 / SIMULATION_STEP_MS);
    for (int i = 0; i < total_cycles; i++)
    {

        // --- Simulate Real-World Parameter Changes ---

        // 1. Simulate roll rate decay after launch
        // Slowly decays until it hits a stable 1.8 rps
        if (state.isT0Set && rollRateFp > 0.0f)
        {
            rollRateFp *= 0.95f; //a 5% exponential decay per cycle.
        }

        // 2. Simulate the guidance signal becoming ready after 25 seconds
        if (state.mainClockCycles > 2500)
        { // 2500 cycles = 25 seconds
            tGo = 1;
        }

        // --- Execute the sequencer for one cycle ---
        sequencerExecute(&state, rollRateFp, tGo, &output);

        // --- Print the Status for Observation ---
        printf("Cycle: %-5u | Roll Rate: %6.1f rps | ",
               state.mainClockCycles,
               (double)rollRateFp);

        // Print the current active phase
        if (state.isT3Set)
            printf("Phase: T3 | ");
        else if (state.isT2Set)
            printf("Phase: T2 | ");
        else if (state.isT1Set)
            printf("Phase: T1 | ");
        else if (state.isT0Set)
            printf("Phase: T0 | ");

        // Print any commands (outputs) that were generated this cycle
        printf("Commands: [ ");
        if (output.sendFsaFlag)
            printf("SendFSA ");
        if (output.sendCanardFlag)
            printf("SendCanard ");
        if (output.sendControlFlag)
            printf("SendControl ");
        if (output.sendGuidStartFlag)
            printf("SendGuidStart ");
        if (output.enableProximitySensor)
            printf("EnableProx ");
        printf("]\n");

        // Stop the simulation if the final phase is complete
        if (state.isT3Set && output.enableProximitySensor)
        {
            printf("\n--- Mission Complete: Proximity Sensor Enabled ---\n");
            break;
        }

        // Simulate the 10 ms delay between cycles
        usleep(SIMULATION_STEP_MS * 1000);
    }

    printf("\n--- Sequencer Simulation Finished ---\n");
    return 0;
}