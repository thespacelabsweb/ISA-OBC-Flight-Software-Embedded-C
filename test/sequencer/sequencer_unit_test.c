#include <stdio.h>
#include <stdbool.h>
#include "sequencer.h" // Include your header file

// Global variable to track test failures
static int test_failures = 0;

// A simple macro for assertions. It checks a condition and prints the result.
#define ASSERT(condition, test_name) \
    do { \
        if (condition) { \
            printf("[PASS]: %s\n", test_name); \
        } else { \
            printf("[FAIL]: %s (Line: %d)\n", test_name, __LINE__); \
            test_failures++; \
        } \
    } while (0)

// -----------------------------------------------------------------------------
// TEST CASES
// -----------------------------------------------------------------------------

/**
 * @brief Tests if the sequencer initializes to a safe, zeroed state.
 */
void test_sequencerInit_returnsCleanState(void) {
    SequencerState_t state;
    // Set a field to a non-zero value to ensure init works
    state.isT1Set = true;

    SequencerError_t err = sequencerInit(&state);

    ASSERT(err == SEQ_SUCCESS, "Init returns success code");
    ASSERT(state.isT0Set == false, "Init clears isT0Set");
    ASSERT(state.isT1Set == false, "Init clears isT1Set");
    ASSERT(state.isGswitchActive == false, "Init clears isGswitchActive");
    ASSERT(state.mainClockCycles == 0, "Init clears mainClockCycles");
}

/**
 * @brief Tests the T0 -> T1 transition timeout failsafe.
 * The roll rate condition is never met, forcing a timeout.
 */
void test_transition_T0_to_T1_byTimeout(void) {
    SequencerState_t state;
    SequencerOutput_t output;
    sequencerInit(&state);
    sequencerSetGswitch(&state, true);

    bool transition_happened = false;
    uint16_t high_roll_rate = 100; // 10.0 rps > threshold

    for (uint32_t i = 0; i < (SEQ_T1_WINDOW_OUT_TIME + 5); ++i) {
        sequencerExecute(&state, high_roll_rate, 0, &output);
        if (output.setT1) {
            // Check that the transition happened on the correct cycle
            ASSERT(state.mainClockCycles == (SEQ_T1_WINDOW_OUT_TIME + 1), "T1 transition occurred on timeout cycle");
            transition_happened = true;
            break;
        }
    }
    ASSERT(transition_happened, "T1 transition via timeout occurred");
}


/**
 * @brief Simulates a full, nominal mission from launch to T3.
 * This is the "happy path" test.
 */
void test_fullMission_nominalPath(void) {
    SequencerState_t state;
    SequencerOutput_t output;
    sequencerInit(&state);
    sequencerSetGswitch(&state, true);

    // --- Test Simulation Variables ---
    uint16_t roll_rate = 100; // Start with high roll rate (10.0 rps)
    uint32_t t_go = 0;
    int test_phase = 0; // Use a simple state machine for the test itself

    printf("\n--- Starting Full Mission Simulation ---\n");

    for (uint32_t i = 0; i < 1000; ++i) {
        // --- CORRECTED: Set inputs for the NEXT cycle based on the CURRENT cycle count ---
        // To affect cycle 11, we must set the input when the clock is 10.
        if (state.mainClockCycles == 10) {
            roll_rate = 60; // 6.0 rps < 7.0 rps
        }
        // To affect cycle 20, we must set the input when the clock is 19.
        if (state.mainClockCycles == 19) {
            roll_rate = 15; // 1.5 rps < 2.0 rps
        }
        // To affect cycle 300, we must set the input when the clock is 299.
        if (state.mainClockCycles == 299) {
            t_go = 1;
        }

        // --- Execute the sequencer for the current cycle (clock will increment inside) ---
        sequencerExecute(&state, roll_rate, t_go, &output);

        // --- Check outputs and assert results ---
        if (test_phase == 0) {
            if (output.setT1) {
                printf("  [INFO] T1 transition at cycle %u\n", state.mainClockCycles);
                // Condition met on cycle 11, confirmed for 3 cycles (11, 12, 13). Transition on 13.
                ASSERT(state.mainClockCycles == (10 + SEQ_CONFIRMATION_CYCLES), "T1 transition occurred 3 cycles after condition met");
                test_phase = 1; // Move to next test phase
            }
        }
        else if (test_phase == 1) {
            if (output.setT2) {
                printf("  [INFO] T2 transition at cycle %u\n", state.mainClockCycles);
                // T1 at cycle 13. Window opens at cycle 24 (13+10+1). Check starts at cycle 24.
                // Roll rate good since cycle 20. Confirmed 3 cycles (24, 25, 26). Transition on 26.
                ASSERT(state.mainClockCycles == 26, "T2 transition occurred after T2 window delay");
                test_phase = 2; // Move to next test phase
            }
        }
        else if (test_phase == 2) {
            if (output.setT3) {
                printf("  [INFO] T3 transition at cycle %u\n", state.mainClockCycles);
                ASSERT(state.mainClockCycles == 300, "T3 transition occurred when tGo was set");
                ASSERT(output.enableProximitySensor == true, "Proximity sensor enabled at T3");
                test_phase = 3; // Mission complete
                break; // Exit simulation loop
            }
        }
    }
    printf("--- Simulation Ended ---\n");
    ASSERT(test_phase == 3, "Full nominal mission sequence completed successfully");
}


// -----------------------------------------------------------------------------
// TEST RUNNER
// -----------------------------------------------------------------------------

int main(void) {
    printf("Starting Sequencer Unit Tests...\n\n");

    test_sequencerInit_returnsCleanState();
    test_transition_T0_to_T1_byTimeout();
    test_fullMission_nominalPath();


    printf("\n---------------------------------------\n");
    if (test_failures == 0) {
        printf("All tests passed!\n");
    } else {
        printf("%d test(s) failed.\n", test_failures);
    }
    printf("---------------------------------------\n");

    return test_failures;
}