#include <ignition/gazebo/System.hh>
#include <ignition/transport/Node.hh>
#include <ignition/msgs/imu.pb.h>
#include <ignition/msgs/fluid_pressure.pb.h>
#include <ignition/msgs/odometry.pb.h>
#include <mutex>
#include <chrono>

namespace helios_controller{
    class HeliosController:
        public ignition::gazebo::System,
        public ignition::gazebo::ISystemConfigure,
        public ignition::gazebo::ISystemPreUpdate
        {
            public: HeliosController();
            public: ~HeliosController() override;
            public: void PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                                ignition::gazebo::EntityComponentManager &_ecm) override;
            public:
                void Configure(
                    const ignition::gazebo::Entity &entity,
                    const std::shared_ptr<const sdf::Element> &sdf,
                    ignition::gazebo::EntityComponentManager &ecm,
                    ignition::gazebo::EventManager &eventMgr) override;

            private: 
                ignition::gazebo::Entity heliosEntity{ignition::gazebo::kNullEntity};
                
                std::chrono::steady_clock::duration lastAttitudeUpdateTime{};
                std::chrono::steady_clock::duration lastPositionUpdateTime{};
                ignition::transport::Node node;

                // Odometry Subscription
                ignition::msgs::Odometry latestOdomMsg;
                std::mutex odomMutex;
                bool odomSubInitialized = false;  
                bool odomReceived = false;
                
                // IMU Subscription
                ignition::msgs::IMU latestIMUMsg;
                std::mutex imuMutex;
                bool imuSubInitialized = false;
                bool imuReceived = false;
                
                // Fluid Pressure (Air) Subscription
                ignition::msgs::FluidPressure latestPressureMsg;
                std::mutex pressureMutex;
                bool pressureSubInitialized = false;
                bool pressureReceived = false;
                
                // Motor Command Publisher 
                ignition::transport::Node::Publisher motorPub;
                bool motorPubInitialized = false;
                
                void OdomCallback(const ignition::msgs::Odometry &msg);
                void ImuCallback(const ignition::msgs::IMU &_msg);
                void PressureCallback(const ignition::msgs::FluidPressure &_msg);
        };
}