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

#ifdef SEQUENCER_H
#define SEQUENCER_H


/**
MISRA C Rule 2.5 Macro definitions should be used for constants
All timing and threshold values are defined as macros for clarity
 */

/**
* @brief Minor cycle timing constants (MISRA C compliant)
* These define the fundamental timing of the system
 */

#define SEQ_MINOR_CYCLE_MS             10U     /* Minor cycle period in milliseconds*/
#define SEQ_MINOR_CYCLE_FREQ_HZ        100U    /* Minor cycle frequency in Hz*/
#define SEQ_CYCLES_PER_SECOND          100U    /* Number of minor cycles per second */
#define SEQ_MS_TO_SECONDS(ms)          ((ms) / SEQ_MINOR_CYCLE_MS)  /* Convert ms to cycles */

/**
 * @brief Roll rate thresholds (MISRA C Rule 2.5: Use macros for constants)
 * These are the critical thresholds from the flight requirements
 */

#define SEQ_ROLL_RATE_T1_THRESHOLD   7.0       /* Roll rate threshold for T1 in rps*/
#define SEQ_ROLL_RATE_T2_THRESHOLD   2.0       /* Roll rate threshold for T2 in rps*/

/**
 * @brief Confirmation requirements (MISRA C Rule 2.5)
 * Safety requirement: conditions must be met for multiple consecutive cycles
 */

 #define SEQ_CONFIRMATION_CYCLES     3U       /* Required consecutive cycles for confirmation */

/**
 * @brief T2 timing delays (MISRA C Rule 2.5)
 * These define the precise timing sequence after T2 is set
 */

#define SEQ_T2_PLUS_DELAY           0.01     /* 10ms delay after T2 in seconds */