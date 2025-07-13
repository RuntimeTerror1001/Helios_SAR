#ifndef HELIOS_UTILS_H
#define HELIOS_UTILS_H

#include "helios_types.h"
#include <ignition/msgs/imu.pb.h>
#include <ignition/msgs/odometry.pb.h>

void FillHeliosState(const ignition::msgs::Odometry &msg, HeliosState *state);

void FillIMUData(const ignition::msgs::IMU &msg, HeliosIMUData *imu);

void GenerateTestSetpoint(HeliosSetpoint *setpoint);

static inline float helios_radians(float degrees){
    return (float)(M_PI / 180) * degrees;
}

static inline float helios_constrain(float value, float minVal, float maxVal){
    return fminf(maxVal, fmaxf(minVal, value));
}

#endif