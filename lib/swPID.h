/*
 * Author: Augustin.B
 * Date  : 28.07.2021
 *
 * Simple, General purpose, C PID library using fixed-point maths.
 * _compute should be called with a known and stable time interval, such as using a timer.
 *
 * ToDo:
 * - Use band-limited differentiator (LP filter) to limit derivative noise (Usually HF).
 * - Add optional Proportiona-On-Measurement.
 * - Add feedforward ability.
 */

#ifndef SWPID_H_
#define SWPID_H_

#include <stdint.h>
#include <stdbool.h>

typedef struct{
    // PID coefficients Settings
    uint16_t kp, ki, kd;
    uint8_t kpShift, kiShift, kdShift;  // Q-Shift, default is Q8.8.

    // Settings for protection.
    int32_t outMax, outMin;              // Saturation block bounds.

    // Flags
    bool isAntiwindupActive;             // Default is on.
    bool isActive;                       // Is PID regulation active.

    // Working variables
    int32_t measure, output, target;
    int32_t integral;
    int32_t lastMeasure;
} PID_State;


// Initialize swPID object. Uses default Q-Shift format of 8.8.
PID_State swpid_initStruct(uint16_t kp, uint16_t ki, uint16_t kd, int32_t outMax, int32_t outMin);

// Change controller tuning.
void swpid_setQFactors(PID_State* pidState, uint8_t kpShift, uint8_t kiShift, uint8_t kdShift);
void swpid_setCoefs(PID_State* pidState, uint16_t kp, uint16_t ki, uint16_t kd);

// PID Settings.
void swpid_enablePID(PID_State* pidState, bool enable);
void swpid_enableAntiwindup(PID_State* pidState, bool enable);
void swpid_setOutputLimits(PID_State* pidState, int32_t outMax, int32_t outMin);

// Working functions.
void swpid_setTarget(PID_State* pidState, int32_t target);
void swpid_setMeasure(PID_State* pidState, int32_t measure);
int32_t swpid_getOutput(PID_State* pidState);

// Compute PID value, call at known, stable intervals.
void swpid_compute(PID_State* pidState);

#endif /* SWPID_H_ */
