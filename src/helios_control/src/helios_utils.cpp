#include "helios_utils.h"
#include <ignition/msgs/imu.pb.h> 

void FillHeliosState(void *poseIn, void *velIn, HeliosState *state){
    double *pose = (double *)poseIn;
    double *vel = (double *)velIn;

    state->position.x = (float)pose[0];
    state->position.y = (float)pose[1];
    state->position.z = (float)pose[2];

    state->orientationQuat.x = (float)pose[3];
    state->orientationQuat.y = (float)pose[4];
    state->orientationQuat.z = (float)pose[5];
    state->orientationQuat.w = (float)pose[6];

    state->attitude.roll = (float)pose[7];
    state->attitude.pitch = (float)pose[8];
    state->attitude.yaw = (float)pose[9];

    state->velocity.x = (float)vel[0];
    state->velocity.y = (float)vel[1];
    state->velocity.z = (float)vel[2];

    state->acceleration.x = 0.0f;
    state->acceleration.y = 0.0f;
    state->acceleration.z = 0.0f;
}

void FillIMUData(const ignition::msgs::IMU &msg, HeliosIMUData *imu){
    imu->gyro.x = static_cast<float>(msg.angular_velocity().x());
    imu->gyro.y = static_cast<float>(msg.angular_velocity().y());
    imu->gyro.z = static_cast<float>(msg.angular_velocity().z());

    imu->accel.x = static_cast<float>(msg.linear_acceleration().x());
    imu->accel.y = static_cast<float>(msg.linear_acceleration().y());
    imu->accel.z = static_cast<float>(msg.linear_acceleration().z());

    imu->airPress.air_press = 1013.25f;
    imu->airPress.temp = 25.0f;
    imu->airPress.sea_level_alt = 0.0f;
}

void GenerateTestSetpoint(HeliosSetpoint *setpoint){
    // setpoint->attitude.roll = 0.0f;
    // setpoint->attitude.pitch = 0.0f;
    // setpoint->attitude.yaw = 0.0f;
    // setpoint->thrust = 5.0f;

    // setpoint->mode.x = HELIOS_MODE_DISABLED;
    // setpoint->mode.y = HELIOS_MODE_DISABLED;
    // setpoint->mode.z = HELIOS_MODE_DISABLED;
    // setpoint->mode.roll = HELIOS_MODE_ABSOLUTE;
    // setpoint->mode.pitch = HELIOS_MODE_ABSOLUTE;
    // setpoint->mode.yaw = HELIOS_MODE_ABSOLUTE;

    // Desired position
    setpoint->position.x = 0.0f;
    setpoint->position.y = 0.0f;
    setpoint->position.z = 5.0f;

    setpoint->attitude.roll = 0.0f;
    setpoint->attitude.pitch = 0.0f;
    setpoint->attitude.yaw = 0.0f;

    setpoint->velocityInBodyFrame = false;
    setpoint->thrust = 0.0f;  // ignored in position mode

    setpoint->mode.x = HELIOS_MODE_ABSOLUTE;
    setpoint->mode.y = HELIOS_MODE_ABSOLUTE;
    setpoint->mode.z = HELIOS_MODE_ABSOLUTE;

    setpoint->mode.roll = HELIOS_MODE_ABSOLUTE;
    setpoint->mode.pitch = HELIOS_MODE_ABSOLUTE;
    setpoint->mode.yaw = HELIOS_MODE_ABSOLUTE;

    setpoint->mode.quat = HELIOS_MODE_DISABLED;

}