#include "pid_main.h"
#include "helios_position_controller.h"
#include "helios_types.h"
#include "helios_utils.h"

#include <cmath>
#include <cstdint>

namespace {
  constexpr float rLimit = 20.0f;
  constexpr float pLimit = 20.0f;
  constexpr float rpLimitOverhead = 1.10f;
  constexpr float xVelMax = 1.0f;
  constexpr float yVelMax = 1.0f;
  constexpr float zVelMax = 1.0f;
  constexpr float velMaxOverhead = 1.10f;
  constexpr float thrustScale = 1000.0f;
  constexpr float kFFx = 0.0f;
  constexpr float kFFy = 0.0f;

  constexpr float posFiltCutoff = 20.0f;
  constexpr float velFiltCutoff = 20.0f;
  constexpr float posZFiltCutoff = 20.0f;
  constexpr float velZFiltCutoff = 20.0f;

  #define POSITION_RATE 100

  constexpr float DT = 1.0f / POSITION_RATE; 

  struct PIDAxis {
    PIDObject pid;
    float setpoint = 0.0f;
  };

  struct ControllerState {
    PIDAxis pidX, pidY, pidZ;
    PIDAxis pidVX, pidVY, pidVZ;
    uint16_t thrustBase = 36000;
    uint16_t thrustMin = 20000;
  };

  ControllerState ctrl;

  float state_body_x = 0, state_body_y = 0;
  float state_body_vx = 0, state_body_vy = 0;

  float runPid(float input, PIDAxis &axis, float setpoint, float dt) {
    axis.setpoint = setpoint;
    PIDSetDesired(&axis.pid, setpoint);
    return PIDUpdate(&axis.pid, input, true);
  }
}

void positionControllerInit() {
  PIDInit(&ctrl.pidX.pid, 0, 2.0f, 0.0f, 0.0f, DT, POSITION_RATE, posFiltCutoff, true);
  PIDInit(&ctrl.pidY.pid, 0, 2.0f, 0.0f, 0.0f, DT, POSITION_RATE, posFiltCutoff, true);
  PIDInit(&ctrl.pidZ.pid, 0, 2.0f, 0.5f, 0.0f, DT, POSITION_RATE, posZFiltCutoff, true);

  PIDInit(&ctrl.pidVX.pid, 0, 25.0f, 1.0f, 0.0f, DT, POSITION_RATE, velFiltCutoff, true);
  PIDInit(&ctrl.pidVY.pid, 0, 25.0f, 1.0f, 0.0f, DT, POSITION_RATE, velFiltCutoff, true);
  PIDInit(&ctrl.pidVZ.pid, 0, 25.0f, 15.0f, 0.0f, DT, POSITION_RATE, velZFiltCutoff, true);
}

void positionController(float* thrust, HeliosAttitude* attitude, HeliosSetpoint* setpoint, const HeliosState* state) {
  ctrl.pidX.pid.outputLimit = xVelMax * velMaxOverhead;
  ctrl.pidY.pid.outputLimit = yVelMax * velMaxOverhead;
  ctrl.pidZ.pid.outputLimit = std::fmax(zVelMax, 0.5f) * velMaxOverhead;

  float cosyaw = std::cos(state->attitude.yaw * M_PI / 180.0f);
  float sinyaw = std::sin(state->attitude.yaw * M_PI / 180.0f);

  float setp_body_x = setpoint->position.x * cosyaw + setpoint->position.y * sinyaw;
  float setp_body_y = -setpoint->position.x * sinyaw + setpoint->position.y * cosyaw;

  state_body_x = state->position.x * cosyaw + state->position.y * sinyaw;
  state_body_y = -state->position.x * sinyaw + state->position.y * cosyaw;

  float globalvx = setpoint->velocity.x;
  float globalvy = setpoint->velocity.y;

  if (setpoint->mode.x == HELIOS_MODE_ABSOLUTE) {
    setpoint->velocity.x = runPid(state_body_x, ctrl.pidX, setp_body_x, DT);
  } else if (!setpoint->velocityInBodyFrame) {
    setpoint->velocity.x = globalvx * cosyaw + globalvy * sinyaw;
  }

  if (setpoint->mode.y == HELIOS_MODE_ABSOLUTE) {
    setpoint->velocity.y = runPid(state_body_y, ctrl.pidY, setp_body_y, DT);
  } else if (!setpoint->velocityInBodyFrame) {
    setpoint->velocity.y = globalvy * cosyaw - globalvx * sinyaw;
  }

  if (setpoint->mode.z == HELIOS_MODE_ABSOLUTE) {
    setpoint->velocity.z = runPid(state->position.z, ctrl.pidZ, setpoint->position.z, DT);
  }

  velocityController(thrust, attitude, setpoint, state);
}

void velocityController(float* thrust, HeliosAttitude* attitude, HeliosSetpoint* setpoint, const HeliosState* state) {
  ctrl.pidVX.pid.outputLimit = pLimit * rpLimitOverhead;
  ctrl.pidVY.pid.outputLimit = rLimit * rpLimitOverhead;
  ctrl.pidVZ.pid.outputLimit = (UINT16_MAX / 2 / thrustScale);

  float cosyaw = std::cos(state->attitude.yaw * M_PI / 180.0f);
  float sinyaw = std::sin(state->attitude.yaw * M_PI / 180.0f);

  state_body_vx = state->velocity.x * cosyaw + state->velocity.y * sinyaw;
  state_body_vy = -state->velocity.x * sinyaw + state->velocity.y * cosyaw;

  attitude->pitch = -runPid(state_body_vx, ctrl.pidVX, setpoint->velocity.x, DT) - kFFx * setpoint->velocity.x;
  attitude->roll  = -runPid(state_body_vy, ctrl.pidVY, setpoint->velocity.y, DT) - kFFy * setpoint->velocity.y;

  attitude->roll  = helios_constrain(attitude->roll, -rLimit, rLimit);
  attitude->pitch = helios_constrain(attitude->pitch, -pLimit, pLimit);

  float thrustRaw = runPid(state->velocity.z, ctrl.pidVZ, setpoint->velocity.z, DT);
  *thrust = thrustRaw * thrustScale + ctrl.thrustBase;
  if (*thrust < ctrl.thrustMin) *thrust = ctrl.thrustMin;
  *thrust = helios_constrain(*thrust, 0, UINT16_MAX);
}

void positionControllerResetAllPID() {
  PIDReset(&ctrl.pidX.pid);
  PIDReset(&ctrl.pidY.pid);
  PIDReset(&ctrl.pidZ.pid);
  PIDReset(&ctrl.pidVX.pid);
  PIDReset(&ctrl.pidVY.pid);
  PIDReset(&ctrl.pidVZ.pid);
}

void positionControllerResetAllFilters() {
  filterReset(&ctrl.pidX.pid, POSITION_RATE, posFiltCutoff, true);
  filterReset(&ctrl.pidY.pid, POSITION_RATE, posFiltCutoff, true);
  filterReset(&ctrl.pidZ.pid, POSITION_RATE, posZFiltCutoff, true);
  filterReset(&ctrl.pidVX.pid, POSITION_RATE, velFiltCutoff, true);
  filterReset(&ctrl.pidVY.pid, POSITION_RATE, velFiltCutoff, true);
  filterReset(&ctrl.pidVZ.pid, POSITION_RATE, velZFiltCutoff, true);
}
