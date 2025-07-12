#include "helios_types.h"
#include "helios_attitude_controller.h"
#include "helios_position_controller.h"
#include "helios_pid.h"

#include <chrono> 

static std::chrono::steady_clock::duration lastAttitudeUpdateTime{};
static std::chrono::steady_clock::duration lastPositionUpdateTime{};

// You can also define rates if needed
constexpr double ATTITUDE_RATE = 500.0;  // Hz
constexpr double POSITION_RATE = 100.0;  // Hz

constexpr auto ATTITUDE_PERIOD = std::chrono::duration<double>(1.0 / ATTITUDE_RATE);
constexpr auto POSITION_PERIOD = std::chrono::duration<double>(1.0 / POSITION_RATE);

static HeliosAttitude attitudeDesired;
static HeliosAttitude rateDesired;
static float actuatorThrust = 0.0f;

static float capAngle(float angle)
{
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

void HeliosPIDInit(void)
{
    //attitudeControllerInit(ATTITUDE_UPDATE_DT);
    //positionControllerInit();
}

void HeliosPIDUpdate(HeliosControlOutput *output,
                     HeliosSetpoint *setpoint,
                     const HeliosIMUData *imu,
                     const HeliosState *state,
                     std::chrono::steady_clock::duration simTime)
{
    if ((simTime - lastAttitudeUpdateTime) >= ATTITUDE_PERIOD)
    {
        if (setpoint->mode.yaw == HELIOS_MODE_VELOCITY)
        {
            attitudeDesired.yaw = capAngle(attitudeDesired.yaw +
                                           setpoint->attitudeRate.yaw * (1.0f/ATTITUDE_RATE));
        }
        else
        {
            attitudeDesired.yaw = capAngle(setpoint->attitude.yaw);
        }

        if (setpoint->mode.z == HELIOS_MODE_DISABLED)
            actuatorThrust = setpoint->thrust;

        if (setpoint->mode.x == HELIOS_MODE_DISABLED || setpoint->mode.y == HELIOS_MODE_DISABLED)
        {
            attitudeDesired.roll = setpoint->attitude.roll;
            attitudeDesired.pitch = setpoint->attitude.pitch;
        }

        attitudeCorrecterPID(
            state->attitude.roll, state->attitude.pitch, state->attitude.yaw,
            attitudeDesired.roll, attitudeDesired.pitch, attitudeDesired.yaw,
            &rateDesired.roll, &rateDesired.pitch, &rateDesired.yaw);

        if (setpoint->mode.roll == HELIOS_MODE_VELOCITY)
        {
            rateDesired.roll = setpoint->attitudeRate.roll;
            resetRollAttitudePID();
        }

        if (setpoint->mode.pitch == HELIOS_MODE_VELOCITY)
        {
            rateDesired.pitch = setpoint->attitudeRate.pitch;
            resetPitchAttitudePID();
        }

        attitudeRateCorrecterPID(
            imu->gyro.x, -imu->gyro.y, imu->gyro.z,
            rateDesired.roll, rateDesired.pitch, rateDesired.yaw);

        getActuatorOutput(&output->rollTorque,
                                            &output->pitchTorque,
                                            &output->yawTorque);

        output->yawTorque = -output->yawTorque;  

        lastAttitudeUpdateTime = simTime;
    }

    if ((simTime - lastPositionUpdateTime) >= POSITION_PERIOD)
    {
        positionController(&actuatorThrust, &attitudeDesired, setpoint, state);
        lastPositionUpdateTime = simTime;
    }

    output->thrust = actuatorThrust;

    if (output->thrust == 0.0f)
    {
        output->rollTorque = 0;
        output->pitchTorque = 0;
        output->yawTorque = 0;

        attitudeControllerResetAllPID();
        attitudeDesired.yaw = state->attitude.yaw;
    }
}