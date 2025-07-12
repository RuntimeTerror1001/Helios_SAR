#include "pid_main.h"
#include "helios_utils.h"

void PIDInit(PIDObject* pid, float desired, float kp, float ki, float kd, float dt,
             float samplingRate, float cutoffFreq, bool enableDFilter)
{
  pid->error         = 0;
  pid->prevError     = 0;
  pid->integ         = 0;
  pid->deriv         = 0;
  pid->desired       = desired;
  pid->kp            = kp;
  pid->ki            = ki;
  pid->kd            = kd;
  pid->iLimit        = DEFAULT_PID_INTEGRATION_LIMIT;
  pid->outputLimit   = DEFAULT_PID_OUTPUT_LIMIT;
  pid->dt            = dt;
  pid->enableDFilter = enableDFilter;
}

float PIDUpdate(PIDObject* pid, float measured, bool updateError)
{
    float output = 0.0f;

    if (updateError)
    {
        pid->error = pid->desired - measured;
    }

    pid->outP = pid->kp * pid->error;
    output += pid->outP;

    float deriv = (pid->error - pid->prevError) / pid->dt;
    pid->deriv = deriv;

    if (std::isnan(pid->deriv)) {
        pid->deriv = 0;
    }

    pid->outD = pid->kd * pid->deriv;
    output += pid->outD;

    pid->integ += pid->error * pid->dt;

    if (pid->iLimit != 0)
    {
        pid->integ = helios_constrain(pid->integ, -pid->iLimit, pid->iLimit);
    }

    pid->outI = pid->ki * pid->integ;
    output += pid->outI;

    if (pid->outputLimit != 0)
    {
        output = helios_constrain(output, -pid->outputLimit, pid->outputLimit);
    }

    pid->prevError = pid->error;

    return output;
}

void PIDSetIntegralLimit(PIDObject* pid, float limit) {
    pid->iLimit = limit;
}

void PIDReset(PIDObject* pid)
{
    pid->error     = 0;
    pid->prevError = 0;
    pid->integ     = 0;
    pid->deriv     = 0;
}

void PIDSetError(PIDObject* pid, float error)
{
    pid->error = error;
}

void PIDSetDesired(PIDObject* pid, float desired)
{
    pid->desired = desired;
}

float PIDGetDesired(PIDObject* pid)
{
    return pid->desired;
}

bool PIDIsActive(PIDObject* pid)
{
    return !(pid->kp < 0.0001f && pid->ki < 0.0001f && pid->kd < 0.0001f);
}

void PIDSetKp(PIDObject* pid, float kp)
{
    pid->kp = kp;
}

void PIDSetKi(PIDObject* pid, float ki)
{
    pid->ki = ki;
}

void PIDSetKd(PIDObject* pid, float kd)
{
    pid->kd = kd;
}

void PIDSetDt(PIDObject* pid, float dt) {
    pid->dt = dt;
}

void filterReset(PIDObject* pid, float samplingRate, float cutoffFreq, bool enableDFilter) {
    pid->enableDFilter = enableDFilter;
}
