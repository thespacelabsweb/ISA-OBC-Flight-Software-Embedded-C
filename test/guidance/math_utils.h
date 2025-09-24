/******************************************************************************
 * ISA Flight Software - Guidance Module Support Library
 * @file math_utils.h
 * @brief Mathematical utility functions for guidance algorithms
 * @details Implements specialized math functions needed for guidance calculations
 * @author Flight Software Team, Spacelabs
 * @date 2025
 * @version 1.0
 *
 * MISRA C: Compliant Implementation (fixed-width types, parameter validation, error handling)
 ******************************************************************************/

#ifndef MATH_UTILS_H
#define MATH_UTILS_H

#include <stdint.h>
#include <stdbool.h>

/* Flight software error codes for math utilities */
typedef enum {
    MATH_SUCCESS = 0U,
    MATH_ERROR_INVALID_PARAM = 1U,
    MATH_ERROR_DOMAIN_ERROR = 2U
} MathError_t;

/* Function prototypes - MISRA C compliant with parameter validation */

/******************************************************************************
 * @brief Signed power function: sig(m,x) = sign(x) * |x|^m
 * @details Used in impact angle control calculations for guidance algorithms
 * @param m Exponent (must be > 0 for real results)
 * @param x Input value
 * @return float Result of sign(x) * |x|^m, or 0.0f on error
 ******************************************************************************/
float sig(float m, float x);

/******************************************************************************
 * @brief Safe sign function: returns -1, 0, or +1
 * @details More robust than standard sign function for floating-point edge cases
 * @param x Input value
 * @return float -1.0f if x < 0, 0.0f if x == 0, +1.0f if x > 0
 ******************************************************************************/
float sign_safe(float x);

/******************************************************************************
 * @brief Safe absolute value function
 * @details Handles floating-point edge cases
 * @param x Input value
 * @return float Absolute value of x
 ******************************************************************************/
float abs_safe(float x);

/******************************************************************************
 * @brief Safe power function with domain checking
 * @details Computes base^exponent with overflow/underflow protection
 * @param base Base value
 * @param exponent Exponent value
 * @return float Result of base^exponent, or 0.0f on domain error
 ******************************************************************************/
float pow_safe(float base, float exponent);

#endif /* MATH_UTILS_H */

