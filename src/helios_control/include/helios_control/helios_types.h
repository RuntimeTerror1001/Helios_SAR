#ifndef HELIOS_TYPES_H
#define HELIOS_TYPES_H

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

struct HeliosVec3{
    float x;
    float y;
    float z;
};
    
struct HeliosAttitude{
    float roll; // degrees
    float yaw; // degrees
    float pitch; // degrees
};

struct HeliosQuat{
    float x;
    float y;
    float z;
    float w;
};

enum HeliosControlMode{
    HELIOS_MODE_DISABLED = 0,
    HELIOS_MODE_ABSOLUTE,
    HELIOS_MODE_VELOCITY
};

struct SetpointMode{
        HeliosControlMode x;
        HeliosControlMode y;
        HeliosControlMode z;
        HeliosControlMode roll;
        HeliosControlMode pitch;
        HeliosControlMode yaw;
        HeliosControlMode quat;
};

struct HeliosSetpoint{
    HeliosAttitude attitude; // degrees
    HeliosAttitude attitudeRate; // degrees/sec
    HeliosQuat attitudeQuat; 
    float thrust; // N
    HeliosVec3 position; // meters
    HeliosVec3 velocity; // m/s
    HeliosVec3 acceleration; // m/s^2
    bool velocityInBodyFrame;
    SetpointMode mode;
};

struct HeliosState{
    HeliosAttitude attitude;
    HeliosQuat orientationQuat;
    HeliosVec3 position;
    HeliosVec3 velocity;
};

struct HeliosControlOutput{
    int16_t rollTorque;
    int16_t pitchTorque;
    int16_t yawTorque;
    float thrust;
};

struct HeliosAirPress{
    float air_press;
    float temp;
    float sea_level_alt;
};

struct HeliosIMUData{
    HeliosVec3 gyro; // deg/s
    HeliosAirPress airPress;
};

#endif