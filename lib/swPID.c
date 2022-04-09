/*
 * Author: Augustin.B
 * Date  : 28.07.2021
 */

#include "swPID.h"

PID_State swpid_initStruct(uint16_t kp, uint16_t ki, uint16_t kd, int32_t outMax, int32_t outMin)
{
    PID_State pidState;

    // Set user values.
    pidState.kp = kp;
    pidState.ki = ki;
    pidState.kd = kd;
    pidState.outMax = outMax;
    pidState.outMin = outMin;

	// Shift coefficients by 8 by default.
	pidState.kpShift = 8;
	pidState.kiShift = 8;
	pidState.kdShift = 8;

    // antiwindup true by default, with PID disabled.
    pidState.isAntiwindupActive = true;
    pidState.isActive = false;

    // Initialize working variables to 0;
    pidState.measure = 0;
    pidState.output = 0;
    pidState.target = 0;
    pidState.integral = 0;
    pidState.lastMeasure = 0;

    return pidState;
}

void swpid_setQFactors(PID_State* pidState, uint8_t kpShift, uint8_t kiShift, uint8_t kdShift)
{
	pidState->kpShift = kpShift;
	pidState->kiShift = kiShift;
	pidState->kdShift = kdShift;
}

void swpid_setCoefs(PID_State* pidState, uint16_t kp, uint16_t ki, uint16_t kd)
{
	pidState->kp = kp;
	pidState->ki = ki;
	pidState->kd = kd;
}

void swpid_enablePID(PID_State* pidState, bool enable)
{
	if (enable && !pidState->isActive)
    {
		// Initialize if PID was disabled
		pidState->integral = pidState->output;
	    pidState->lastMeasure = pidState->measure;
    }

    pidState->isActive = enable;
}

void swpid_enableAntiwindup(PID_State* pidState, bool enable)
{
    pidState->isAntiwindupActive = enable;
}

void swpid_setOutputLimits(PID_State* pidState, int32_t outMax, int32_t outMin)
{
    // Set new saturation values.
	pidState->outMin = outMin;
	pidState->outMax = outMax;

    // Update internal variables accordingly.
	if (pidState->output > pidState->outMax) {
		pidState->output = pidState->outMax;
	}
	else if (pidState->output < pidState->outMin) {
		pidState->output = pidState->outMin;
	}
}

void swpid_setTarget(PID_State* pidState, int32_t target)
{
    pidState->target = target;
}

void swpid_setMeasure(PID_State* pidState, int32_t measure)
{
    pidState->measure = measure;
}

int32_t swpid_getOutput(PID_State* pidState)
{
    return pidState->output;
}

void swpid_compute(PID_State* pidState)
{
	if (pidState->isActive){
		// Compute the error and working variables:
		int32_t error = pidState->target - pidState->measure;
		pidState->integral += (pidState->ki * error) >> pidState->kpShift;
		int32_t d_measure = (pidState->measure - pidState->lastMeasure);

		// Compute PID output
		pidState->output = ((pidState->kp * error) >> pidState->kpShift) + pidState->integral - ((pidState->kd * d_measure) >> pidState->kpShift);

		if (pidState->output > pidState->outMax)
		{
			// Saturate output
			pidState->output = pidState->outMax;
			if (pidState->isAntiwindupActive)
			{
				// Reset integral term
				pidState->integral -= (pidState->ki * error) >> pidState->kpShift;
			}
		}
		else if (pidState->output < pidState->outMin)
		{
			// Saturate output.
			pidState->output = pidState->outMin;
			if (pidState->isAntiwindupActive)
			{
				// Reset integral term
				pidState->integral -= (pidState->ki * error) >> pidState->kpShift;
			}
		}

		// Remember some variables for next time
		pidState->lastMeasure = pidState->measure;
	}
}