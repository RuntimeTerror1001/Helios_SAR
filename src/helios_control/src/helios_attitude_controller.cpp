#include "helios_attitude_controller.h"
#include "pid_main.h"
#include <cstdint>
#include <cmath>

#define ATTITUDE_LPF_CUTOFF_FREQ 15.0f
#define ATTITUDE_LPF_ENABLE false
#define ATTITUDE_RATE_LPF_CUTOFF_FREQ 30.0f
#define ATTITUDE_RATE_LPF_ENABLE false 
#define ATTITUDE_RATE 500

namespace {

    inline int16_t saturateSignedInt16(float in){
        if (in > INT16_MAX)
            return INT16_MAX;
        else if (in < -INT16_MAX)
            return -INT16_MAX;
        else
            return static_cast<int16_t>(in);
    }

    PIDObject PIDRollRate;
    PIDObject PIDPitchRate;
    PIDObject PIDYawRate;
    PIDObject PIDRoll;
    PIDObject PIDPitch;
    PIDObject PIDYaw;

    int16_t rollOP = 0;
    int16_t pitchOP = 0;
    int16_t yawOP = 0;
    bool initialized = false;
}

void attitudeControllerInit(const float updateDt)
{
  if (initialized)
    return;

  PIDInit(&PIDRollRate,  0, PID_ROLL_RATE_KP,  PID_ROLL_RATE_KI,  PID_ROLL_RATE_KD,
          updateDt, ATTITUDE_RATE, ATTITUDE_RATE_LPF_CUTOFF_FREQ, ATTITUDE_RATE_LPF_ENABLE);
  PIDInit(&PIDPitchRate, 0, PID_PITCH_RATE_KP, PID_PITCH_RATE_KI, PID_PITCH_RATE_KD,
          updateDt, ATTITUDE_RATE, ATTITUDE_RATE_LPF_CUTOFF_FREQ, ATTITUDE_RATE_LPF_ENABLE);
  PIDInit(&PIDYawRate,   0, PID_YAW_RATE_KP,   PID_YAW_RATE_KI,   PID_YAW_RATE_KD,
          updateDt, ATTITUDE_RATE, ATTITUDE_RATE_LPF_CUTOFF_FREQ, ATTITUDE_RATE_LPF_ENABLE);

  PIDSetIntegralLimit(&PIDRollRate,  PID_ROLL_RATE_INTEGRATION_LIMIT);
  PIDSetIntegralLimit(&PIDPitchRate, PID_PITCH_RATE_INTEGRATION_LIMIT);
  PIDSetIntegralLimit(&PIDYawRate,   PID_YAW_RATE_INTEGRATION_LIMIT);

  PIDInit(&PIDRoll,  0, PID_ROLL_KP,  PID_ROLL_KI,  PID_ROLL_KD,
          updateDt, ATTITUDE_RATE, ATTITUDE_LPF_CUTOFF_FREQ, ATTITUDE_LPF_ENABLE);
  PIDInit(&PIDPitch, 0, PID_PITCH_KP, PID_PITCH_KI, PID_PITCH_KD,
          updateDt, ATTITUDE_RATE, ATTITUDE_LPF_CUTOFF_FREQ, ATTITUDE_LPF_ENABLE);
  PIDInit(&PIDYaw,   0, PID_YAW_KP,   PID_YAW_KI,   PID_YAW_KD,
          updateDt, ATTITUDE_RATE, ATTITUDE_LPF_CUTOFF_FREQ, ATTITUDE_LPF_ENABLE);

  PIDSetIntegralLimit(&PIDRoll,  PID_ROLL_INTEGRATION_LIMIT);
  PIDSetIntegralLimit(&PIDPitch, PID_PITCH_INTEGRATION_LIMIT);
  PIDSetIntegralLimit(&PIDYaw,   PID_YAW_INTEGRATION_LIMIT);

  initialized = true;
}

bool attitudeControllerTest()
{
  return initialized;
}

void attitudeRateCorrecterPID(
    float rollRateActual, float pitchRateActual, float yawRateActual,
    float rollRateDesired, float pitchRateDesired, float yawRateDesired)
{
  PIDSetDesired(&PIDRollRate, rollRateDesired);
  rollOP = saturateSignedInt16(PIDUpdate(&PIDRollRate, rollRateActual, true));

  PIDSetDesired(&PIDPitchRate, pitchRateDesired);
  pitchOP = saturateSignedInt16(PIDUpdate(&PIDPitchRate, pitchRateActual, true));

  PIDSetDesired(&PIDYawRate, yawRateDesired);
  yawOP = saturateSignedInt16(PIDUpdate(&PIDYawRate, yawRateActual, true));
}

void attitudeCorrecterPID(
    float eulerRollActual, float eulerPitchActual, float eulerYawActual,
    float eulerRollDesired, float eulerPitchDesired, float eulerYawDesired,
    float* rollRateDesired, float* pitchRateDesired, float* yawRateDesired)
{
  PIDSetDesired(&PIDRoll, eulerRollDesired);
  *rollRateDesired = PIDUpdate(&PIDRoll, eulerRollActual, true);

  PIDSetDesired(&PIDPitch, eulerPitchDesired);
  *pitchRateDesired = PIDUpdate(&PIDPitch, eulerPitchActual, true);

  float yawError = eulerYawDesired - eulerYawActual;
  if (yawError > 180.0f)
    yawError -= 360.0f;
  else if (yawError < -180.0f)
    yawError += 360.0f;

  PIDSetError(&PIDYaw, yawError);
  *yawRateDesired = PIDUpdate(&PIDYaw, eulerYawActual, false);
}

void resetRollAttitudePID()
{
  PIDReset(&PIDRoll);
}

void resetPitchAttitudePID()
{
  PIDReset(&PIDPitch);
}

void attitudeControllerResetAllPID()
{
  PIDReset(&PIDRoll);
  PIDReset(&PIDPitch);
  PIDReset(&PIDYaw);
  PIDReset(&PIDRollRate);
  PIDReset(&PIDPitchRate);
  PIDReset(&PIDYawRate);
}

void getActuatorOutput(int16_t* roll, int16_t* pitch, int16_t* yaw)
{
  *roll = rollOP;
  *pitch = pitchOP;
  *yaw = yawOP;
}