#ifndef HELIOS_PID_H
#define HELIOS_PID_H

#include "helios_types.h"
#include <chrono>

void HeliosPIDInit(void);
bool HeliosPIDTest(void);

void HeliosPIDUpdate(
    HeliosControlOutput *output,
    HeliosSetpoint *setpoint,
    const HeliosIMUData *imu,
    const HeliosState *state,
    std::chrono::steady_clock::duration simTime
);

#endif