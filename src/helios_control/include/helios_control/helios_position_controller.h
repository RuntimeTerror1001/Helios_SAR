#ifndef HELIOS_POSITION_CONTROLLER_H
#define HELIOS_POSITION_CONTROLLER_H

#include "helios_types.h"

void positionControllerInit();
void positionControllerResetAllPID();
void positionControllerResetAllFilters();

void positionController(float* thrust, HeliosAttitude *attitude, HeliosSetpoint *setpoint, const HeliosState *state);

void velocityController(float* thrust, HeliosAttitude *attitude, HeliosSetpoint *setpoint, const HeliosState *state);

#endif