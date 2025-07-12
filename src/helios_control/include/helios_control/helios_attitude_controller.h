#ifndef HELIOS_ATTITUDE_CONTROLLER_H
#define HELIOS_ATTITUDE_CONTROLLER_H

#include <stdbool.h>
#include "helios_types.h"

/**
 * Initialize attitude PID and rate PID controllers.
 * @param updateDt Update timestep in seconds (float).
 */
void attitudeControllerInit(const float updateDt);

/**
 * Returns true if controller has been initialized.
 */
bool attitudeControllerTest(void);

/**
 * Runs attitude PID to generate desired angular rates.
 * @param eulerRollActual  Current roll angle (degrees)
 * @param eulerPitchActual Current pitch angle (degrees)
 * @param eulerYawActual   Current yaw angle (degrees)
 * @param eulerRollDesired Desired roll angle (degrees)
 * @param eulerPitchDesired Desired pitch angle (degrees)
 * @param eulerYawDesired   Desired yaw angle (degrees)
 * @param rollRateDesired  Output: roll rate command (deg/s)
 * @param pitchRateDesired Output: pitch rate command (deg/s)
 * @param yawRateDesired   Output: yaw rate command (deg/s)
 */
void attitudeCorrecterPID(
    float eulerRollActual, float eulerPitchActual, float eulerYawActual,
    float eulerRollDesired, float eulerPitchDesired, float eulerYawDesired,
    float* rollRateDesired, float* pitchRateDesired, float* yawRateDesired);

/**
 * Runs rate PID to produce actuator torque commands.
 * @param rollRateActual  Measured roll rate (deg/s)
 * @param pitchRateActual Measured pitch rate (deg/s)
 * @param yawRateActual   Measured yaw rate (deg/s)
 * @param rollRateDesired Desired roll rate (deg/s)
 * @param pitchRateDesired Desired pitch rate (deg/s)
 * @param yawRateDesired   Desired yaw rate (deg/s)
 */
void attitudeRateCorrecterPID(
    float rollRateActual, float pitchRateActual, float yawRateActual,
    float rollRateDesired, float pitchRateDesired, float yawRateDesired);

/**
 * Resets only the roll attitude PID.
 */
void resetRollAttitudePID(void);

/**
 * Resets only the pitch attitude PID.
 */
void resetPitchAttitudePID(void);

/**
 * Resets all attitude and rate PID controllers.
 */
void attitudeControllerResetAllPID(void);

/**
 * Gets actuator torque outputs from the rate controllers.
 * @param roll Pointer to roll torque output
 * @param pitch Pointer to pitch torque output
 * @param yaw Pointer to yaw torque output
 */
void getActuatorOutput(
    int16_t* roll, int16_t* pitch, int16_t* yaw);

#endif