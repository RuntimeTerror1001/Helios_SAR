#include "HeliosController.hh"
#include <ignition/plugin/Register.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/LinearVelocity.hh>
#include <ignition/gazebo/components/AngularVelocity.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/msgs/actuators.pb.h>

#include "helios_pid.h"
#include "helios_utils.h"


IGNITION_ADD_PLUGIN(
    helios_controller::HeliosController,
    ignition::gazebo::System,
    ignition::gazebo::ISystemConfigure,
    ignition::gazebo::ISystemPreUpdate
)

using namespace helios_controller;
using namespace ignition::gazebo;
using namespace ignition::math;

HeliosController::HeliosController(){}

HeliosController::~HeliosController(){}

void HeliosController::Configure(
    const ignition::gazebo::Entity &entity,
    const std::shared_ptr<const sdf::Element> &sdf,
    ignition::gazebo::EntityComponentManager &ecm,
    ignition::gazebo::EventManager &eventMgr)
{
    // Optional: store the entity or do setup
    this->heliosEntity = entity;
    std::cout << "[HeliosController] Configure called for entity: " << entity << std::endl;
}

void HeliosController::ImuCallback(const ignition::msgs::IMU &_msg)
{
    std::lock_guard<std::mutex> lock(this->imuMutex);
    this->latestIMUMsg = _msg;
    this->imuReceived = true;
}

void HeliosController::PressureCallback(const ignition::msgs::FluidPressure &_msg)
{
    std::lock_guard<std::mutex> lock(this->pressureMutex);
    this->latestPressureMsg = _msg;
    this->pressureReceived = true;
}

void HeliosController::PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                        ignition::gazebo::EntityComponentManager &_ecm)
{
    static bool initialized = false;
    if (!initialized)
    {
        HeliosPIDInit();
        initialized = true;
    }

    const auto simTime = _info.simTime;

    if (this->heliosEntity == ignition::gazebo::kNullEntity)
    {
        _ecm.Each<ignition::gazebo::components::Name>(
            [&](const ignition::gazebo::Entity &entity,
                const ignition::gazebo::components::Name *name) -> bool
            {
                if (name->Data() == "helios")  
                {
                    this->heliosEntity = entity;
                    ignmsg << "[HeliosController] Found Helios entity: " << entity << std::endl;
                    return false; // stop iterating
                }
                return true;
            });
    }

    if (this->heliosEntity == ignition::gazebo::kNullEntity)
        return;

    if(!this->motorPubInitialized){
        this->motorPub = this->node.Advertise<ignition::msgs::Actuators>("/gazebo/command/motor_speed");
        this->motorPubInitialized = true;
    }

    if (!this->imuSubInitialized)
    {
        this->node.Subscribe("/world/collapsed_industrial/model/helios/link/base_link/sensor/imu_sensor/imu",
                            &HeliosController::ImuCallback,
                            this);
        this->imuSubInitialized = true;
    }

    if (!this->pressureSubInitialized)
    {
        this->node.Subscribe("/world/collapsed_industrial/model/helios/link/base_link/sensor/air_pressure/air_pressure",
                            &HeliosController::PressureCallback,
                            this);
        this->pressureSubInitialized = true;
    }

    auto poseComp = _ecm.Component<components::Pose>(this->heliosEntity);
    auto linVelComp = _ecm.Component<components::LinearVelocity>(this->heliosEntity);
    auto angVelComp = _ecm.Component<components::AngularVelocity>(this->heliosEntity);

    const auto attitudePeriod = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(1.0 / 500.0)); //ATTITUDE_RATE=500
    const auto positionPeriod = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(1.0 / 100.0));//POSITION_RATE=100

    bool runAttitude = (simTime - lastAttitudeUpdateTime) >= attitudePeriod;
    bool runPosition = (simTime - lastPositionUpdateTime) >= positionPeriod;

    if (runAttitude || runPosition)
    {
        HeliosControlOutput control{};
        HeliosSetpoint setpoint{};
        HeliosIMUData imuData{};
        HeliosState state{};

        // Convert data into C format
        double poseData[10]; // x, y, z, qx, qy, qz, qw, roll, pitch, yaw
        double velData[3];
        double gyroData[3];

        auto pose = poseComp->Data();
        auto rot = pose.Rot().Euler();

        poseData[0] = pose.Pos().X();
        poseData[1] = pose.Pos().Y();
        poseData[2] = pose.Pos().Z();

        poseData[3] = pose.Rot().X();
        poseData[4] = pose.Rot().Y();
        poseData[5] = pose.Rot().Z();
        poseData[6] = pose.Rot().W();

        poseData[7] = rot.X() * 180.0 / M_PI;
        poseData[8] = rot.Y() * 180.0 / M_PI;
        poseData[9] = rot.Z() * 180.0 / M_PI;

        auto vel = linVelComp->Data();
        velData[0] = vel.X();
        velData[1] = vel.Y();
        velData[2] = vel.Z();

        auto angVel = angVelComp->Data();
        gyroData[0] = angVel.X();
        gyroData[1] = angVel.Y();
        gyroData[2] = angVel.Z();

        // Fill C structs using helper
        FillHeliosState(poseData, velData, &state);
        if (this->imuReceived)
        {
            std::lock_guard<std::mutex> lock(this->imuMutex);
            FillIMUData(this->latestIMUMsg, &imuData);
        }
        if (this->pressureReceived)
        {
            std::lock_guard<std::mutex> lock(this->pressureMutex);
            imuData.airPress.air_press = static_cast<float>(this->latestPressureMsg.pressure());
        }
        GenerateTestSetpoint(&setpoint); // TODO : Fill from a ROS2 topic

        HeliosPIDUpdate(&control, &setpoint, &imuData, &state, simTime);

        //--------------------Motor Command Mixer--------------------
        std::vector<double> motorSpeeds(4,0.0);

        // Mixer constants
        const double k_f = 8.54858e-06; // motorConstant (N/(rad/s)^2)
        const double k_m = 0.016; // momentConstant (Nm/N)
        const double T = control.thrust;
        const double tau_x = control.rollTorque;
        const double tau_y = control.pitchTorque;
        const double tau_z = control.yawTorque;

        // Compute individual motor thrusts
        double f0 = 0.25 * (T + tau_x + tau_y - tau_z / k_m); // Front Right
        double f1 = 0.25 * (T - tau_x + tau_y + tau_z / k_m); // Back Left
        double f2 = 0.25 * (T - tau_x - tau_y - tau_z / k_m); // Front Left
        double f3 = 0.25 * (T + tau_x - tau_y + tau_z / k_m); // Back Right

        std::vector<double> forces = {f0,f1,f2,f3};

        // Convert forces to motor speeds
        for(size_t i = 0; i<4;i++){
            double force = std::max(0.0, forces[i]); // Clamp negative to 0
            motorSpeeds[i] = std::sqrt(force / k_f);
        }

        // Publish motor speeds
        if(this->motorPubInitialized){
            ignition::msgs::Actuators msg;
            msg.mutable_velocity()->Resize(4, 0.0f);
            for(int i = 0; i<4; i++)
                msg.mutable_velocity()->Set(i, static_cast<float>(motorSpeeds[i]));
            
            this->motorPub.Publish(msg);
        }

        if (runAttitude)
            lastAttitudeUpdateTime = simTime;

        if (runPosition)
            lastPositionUpdateTime = simTime;
    }
} 