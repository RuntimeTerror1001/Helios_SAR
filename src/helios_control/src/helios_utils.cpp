#include "helios_utils.h"
#include <ignition/msgs/imu.pb.h> 
#include <ignition/msgs/odometry.pb.h>
#include <ignition/math/Quaternion.hh>

void FillHeliosState(const ignition::msgs::Odometry &msg, HeliosState *state){
    state->position.x = static_cast<float>(msg.pose().position().x());
    state->position.y = static_cast<float>(msg.pose().position().y());
    state->position.z = static_cast<float>(msg.pose().position().z());
    state->orientationQuat.x = static_cast<float>(msg.pose().orientation().x());
    state->orientationQuat.y = static_cast<float>(msg.pose().orientation().y());
    state->orientationQuat.z = static_cast<float>(msg.pose().orientation().z());
    state->orientationQuat.w = static_cast<float>(msg.pose().orientation().w());

    auto rotEuler = ignition::math::Quaterniond(
        msg.pose().orientation().w(),
        msg.pose().orientation().x(),
        msg.pose().orientation().y(),
        msg.pose().orientation().z()
    ).Euler();
    state->attitude.roll = static_cast<float>(rotEuler.X() * 180.0/M_PI);
    state->attitude.pitch = static_cast<float>(rotEuler.Y() * 180.0/M_PI);
    state->attitude.yaw = static_cast<float>(rotEuler.Z() * 180.0/M_PI);
    
    if(msg.twist().has_linear()){
        state->velocity.x = static_cast<float>(msg.twist().linear().x());
        state->velocity.y = static_cast<float>(msg.twist().linear().y());
        state->velocity.z = static_cast<float>(msg.twist().linear().z());
    } else{
        state->velocity.x = state->velocity.y = state->velocity.z = 0;
    }
    
}

void FillIMUData(const ignition::msgs::IMU &msg, HeliosIMUData *imu){
    imu->gyro.x = static_cast<float>(msg.angular_velocity().x());
    imu->gyro.y = static_cast<float>(msg.angular_velocity().y());
    imu->gyro.z = static_cast<float>(msg.angular_velocity().z());

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