#ifndef CONTROL_H
#define CONTROL_H
// author: MacManhHai
// email: hai372002@gmail.com

// Include headers
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/manual_control_setpoint.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>
#include <fstream>
#include "planning.h"

// Define constants
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

// Control class definition
class Control : public rclcpp::Node
{
public:
    // Constructor with path parameter
    Control(const std::vector<Point3D>& path); 

    // Control methods
    void arm();
    void disarm();
    void mission_mode();
    void land();
    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
    void publish_control_setpoint(float roll, float pitch, float yaw, float thrust);
    void pid_controller(float_t x_d, float_t y_d, float_t z_d, float_t yaw_d, bool hover);

private:
    // Publishers and Subscribers
    rclcpp::Publisher<VehicleCommand>::SharedPtr            vehicle_command_pub_;
    rclcpp::Publisher<ManualControlSetpoint>::SharedPtr     control_setpoint_pub_;
    rclcpp::Subscription<VehicleAttitude>::SharedPtr        vehicle_attitude_sub_;
    rclcpp::Subscription<VehicleLocalPosition>::SharedPtr   vehicle_local_position_sub_;

    // State variables
    float_t attitude_msg_[4];
    float_t local_position_msg_[3];

    // Timer and counter
    rclcpp::TimerBase::SharedPtr timer_;
    uint64_t counter_;

    // Control commands: [roll, pitch, yaw, throttle]
    float_t u[4]; 

    // Path planning
    std::vector<Point3D> path_;
    size_t path_index_ = 0;

    // Logging
    std::ofstream log_file_;
};

#endif // CONTROL_H
