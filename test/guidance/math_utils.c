/******************************************************************************
 * ISA Flight Software - Guidance Module Support Library
 * @file math_utils.c
 * @brief Mathematical utility functions implementation for guidance algorithms
 * @details Implements specialized math functions needed for guidance calculations
 * @author Flight Software Team, Spacelabs
 * @date 2025
 * @version 1.0
 *
 * MISRA C: Compliant Implementation (fixed-width types, parameter validation, error handling)
 ******************************************************************************/

#include "math_utils.h"
#include <math.h>    /* For powf(), fabsf(), copysignf() */
#include <float.h>   /* For FLT_MAX, FLT_MIN */
#include <errno.h>   /* For errno checking */

/* Numerical constants for stability */
#define MATH_EPSILON 1e-10f
#define MAX_EXPONENT 100.0f  /* Prevent overflow in power functions */

/******************************************************************************
 * @brief Signed power function: sig(m,x) = sign(x) * |x|^m
 * Used in impact angle control calculations for guidance algorithms
 *
 * MATLAB equivalent: y = (sign(x))*(abs(x)^m)
 ******************************************************************************/
float sig(float m, float x) {
    /* Parameter validation */
    if (m <= 0.0f) {
        return 0.0f;  /* Invalid exponent */
    }

    /* Handle zero case */
    if (fabsf(x) < MATH_EPSILON) {
        return 0.0f;
    }

    /* Compute sign(x) * |x|^m */
    float abs_x = fabsf(x);
    float abs_x_to_m = pow_safe(abs_x, m);

    /* Apply sign */
    return copysignf(abs_x_to_m, x);
}

/******************************************************************************
 * @brief Safe sign function: returns -1, 0, or +1
 ******************************************************************************/
float sign_safe(float x) {
    if (x > MATH_EPSILON) {
        return 1.0f;
    } else if (x < -MATH_EPSILON) {
        return -1.0f;
    } else {
        return 0.0f;
    }
}

/******************************************************************************
 * @brief Safe absolute value function
 ******************************************************************************/
float abs_safe(float x) {
    return fabsf(x);
}

/******************************************************************************
 * @brief Safe power function with domain checking
 ******************************************************************************/
float pow_safe(float base, float exponent) {
    /* Parameter validation */
    if (fabsf(base) < MATH_EPSILON && exponent < 0.0f) {
        return 0.0f;  /* Division by zero */
    }

    /* Prevent overflow by limiting exponent */
    if (fabsf(exponent) > MAX_EXPONENT) {
        if (exponent > 0.0f) {
            return (base > 0.0f) ? FLT_MAX : -FLT_MAX;
        } else {
            return 0.0f;
        }
    }

    /* Use standard powf function */
    errno = 0;
    float result = powf(base, exponent);

    /* Check for errors */
    if (errno != 0 || !isfinite(result)) {
        return 0.0f;  /* Return safe default on error */
    }

    return result;
}

