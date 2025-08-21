#include <stdio.h>
#include <unistd.h> // For sleep function for simulation demo
#include <stdbool.h>

#include "sequencer.h"


// --- Simulation parameters --
#define SIMULATION_STEP_MS 10  // Minor cycle 10 ms 
#define SIMULATION_DURATION_S 30 //Running time for the simulation.


int main (void){
    printf("--- Sequencer Simulation Started --- \n");

    // 1. Storage for Sequencer Status
    SequencerStatus status;

    // 2. Initialize the sequencer
    Sequencer_Initialize(&status);

    // --- Simulation Inputs ---
    // Dummy inputs for sequencer
    bool gSwitchActive = false;
    double rollRate = 100.0; // high roll rate in rps
    bool guidanceTerminal_Flag = false;

    // --- Interactive Launch ---
    printf("Projectile is in PRE-FIRING state. Waiting for launch command.\n");
    printf("Fire the Projectile? (y/n): ");

    while (getchar() != 'y') {
        // Do nothing until 'y' is pressed
    }
    // Setting the gswitch to TRUE for one cycle to trigger the launch
    gSwitchActive = true;
    printf("\nLaunch Command Recieved!\n\n");

    // --- Main simulation loop ---

    int total_cycles = SIMULATION_DURATION_S * (1000 / SIMULATION_STEP_MS);
    for (int i = 0; i< total_cycles; i++) {

        // --- Simulate the real world parameters---

        // 1. Simulate roll rate decay: The projectile slows down after launch
        if (status.state > SEQ_STATE_FIRING_DETECTED) {
            rollRate *= 0.99; // Decrease roll rate by 1% each cycle
        }
        // 2. Simulate the terminal phase signal from guidance
        // simulate the guidance is sending the signal after 25 seconds
        if (status.globalMinorCycleCount > 2500) {
            guidanceTerminal_Flag = true;
        }

        // --- Execute the sequencer ---
        // Calling the sequencer with simulated inouts
        Sequencer_Execute(&status, gSwitchActive, rollRate, guidanceTerminal_Flag);

        // --- Print the Status ---
        // Printing the status in specific moments to see whats happening.
        printf("Cycle: %-5u | State: %-25s | Roll Rate: %6.2f rps | Flags: [FSA:%d DAP_R:%d DAP_PY:%d GUID:%d PROX:%d]\n",
            status.globalMinorCycleCount,
            Sequencer_GetStateString(status.state),
            rollRate,
            status.fsaCanardDeploy_Flag,
            status.dapRollControl_Flag,
            status.dapPitchYawControl_Flag,
            status.GUID_START_Flag,
            status.proximityEnable_Flag);


            // --- Reset event ---
            // The g switch is only active for a moment, so reset it after the first cycle
            if (gSwitchActive == true) {
                gSwitchActive = false;
            }
            // Stop the simulation , if the mission is complete
            if (status.state == SEQ_STATE_IMPACT) {
                printf("\n---Mission Complete ---\n");
                break;
            }
            // Simulate the 10 ms delay between cycles
            usleep(SIMULATION_STEP_MS * 1000);
    }

    printf("\n--- Sequencer Simulation Finished ---\n");
    return 0;
}