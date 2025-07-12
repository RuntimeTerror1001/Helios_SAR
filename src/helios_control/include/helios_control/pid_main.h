#ifndef PID_MAIN_H
#define PID_MAIN_H

#include <cmath>
#include <cfloat>
#include <cstdint>
#include <stdbool.h>

#define PID_ROLL_RATE_KP  250.0f
#define PID_ROLL_RATE_KI  500.0f
#define PID_ROLL_RATE_KD  2.5f
#define PID_ROLL_RATE_INTEGRATION_LIMIT    33.3f

#define PID_PITCH_RATE_KP  250.0f
#define PID_PITCH_RATE_KI  500.0f
#define PID_PITCH_RATE_KD  2.5f
#define PID_PITCH_RATE_INTEGRATION_LIMIT   33.3f

#define PID_YAW_RATE_KP  120.0f
#define PID_YAW_RATE_KI  16.7f
#define PID_YAW_RATE_KD  0.0f
#define PID_YAW_RATE_INTEGRATION_LIMIT     166.7f

#define PID_ROLL_KP  6.0f
#define PID_ROLL_KI  3.0f
#define PID_ROLL_KD  0.0f
#define PID_ROLL_INTEGRATION_LIMIT    20.0f

#define PID_PITCH_KP  6.0f
#define PID_PITCH_KI  3.0f
#define PID_PITCH_KD  0.0f
#define PID_PITCH_INTEGRATION_LIMIT   20.0f

#define PID_YAW_KP  6.0f
#define PID_YAW_KI  1.0f
#define PID_YAW_KD  0.35f
#define PID_YAW_INTEGRATION_LIMIT     360.0f

#define DEFAULT_PID_INTEGRATION_LIMIT 5000.0f
#define DEFAULT_PID_OUTPUT_LIMIT      0.0f

typedef struct {
    float desired;
    float error;
    float prevError;
    float integ;
    float deriv;
    float kp;
    float ki;
    float kd;
    float outP;
    float outI;
    float outD;
    float iLimit;
    float outputLimit;
    float dt;
    bool enableDFilter;
} PIDObject;

void PIDInit(PIDObject* pid, float desired, float kp, float ki, float kd, float dt,
             float samplingRate, float cutoffFreq, bool enableDFilter);

void PIDSetIntegralLimit(PIDObject* pid, float limit);
void PIDReset(PIDObject* pid);
float PIDUpdate(PIDObject* pid, float measured, bool updateError);
void PIDSetDesired(PIDObject* pid, float desired);
float PIDGetDesired(PIDObject* pid);
bool PIDIsActive(PIDObject* pid);
void PIDSetError(PIDObject* pid, float error);
void PIDSetKp(PIDObject* pid, float kp);
void PIDSetKi(PIDObject* pid, float ki);
void PIDSetKd(PIDObject* pid, float kd);
void PIDSetDt(PIDObject* pid, float dt);
void filterReset(PIDObject* pid, float samplingRate, float cutoffFreq, bool enableDFilter);

#endif /* PID_MAIN_H */
  