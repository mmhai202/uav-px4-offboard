#include "control.h"
#include "planning.h"

Control::Control(const std::vector<Point3D>& path) : Node("mission")
{

    log_file_.open("/home/hai/DevPX4/uav_simulation/src/uav_log.csv", std::ios::out | std::ios::trunc);
    log_file_ << "x_desired,y_desired,z_desired,x_actual,y_actual,z_actual\n";

    // Initialize publishers
    vehicle_command_pub_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);
    control_setpoint_pub_ = this->create_publisher<ManualControlSetpoint>("/fmu/in/manual_control_input", 10);

    // qos profile for subscribers
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
	auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    // Initialize subscribers
    vehicle_attitude_sub_ = this->create_subscription<VehicleAttitude>(
        "/fmu/out/vehicle_attitude", qos, [this](const VehicleAttitude::UniquePtr msg) {
            float w = msg->q[0];
            float x = msg->q[1];
            float y = msg->q[2];
            float z = msg->q[3];

            // Compute roll, pitch, yaw from quaternion
            float roll  = std::atan2(2.0f * (w * x + y * z), 1.0f - 2.0f * (x * x + y * y));
            float pitch = std::asin(2.0f * (w * y - z * x));
            float yaw   = std::atan2(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z));

            // Convert to degrees
            attitude_msg_[0] = roll  * 180.0f / M_PI;
            attitude_msg_[1] = pitch * 180.0f / M_PI;
            attitude_msg_[2] = yaw   * 180.0f / M_PI;
        });

    vehicle_local_position_sub_ = this->create_subscription<VehicleLocalPosition>(
        "/fmu/out/vehicle_local_position", qos, [this](const VehicleLocalPosition::UniquePtr msg) {
            local_position_msg_[0] = msg->y;    // (x_enu = y_ned)
            local_position_msg_[1] = msg->x;    // (y_enu = x_ned)
            local_position_msg_[2] = -(msg->z); // (z_enu = -z_ned)
        });
    
    // Initialize variables
    counter_ = 0;
    path_ = path;
    path_index_ = 0;

    // Timer callback function
    auto timer_callback = [this]() -> void {

        if (counter_ < 10) this->publish_control_setpoint(0.0, 0.0, 0.0, -1.0);

        // Switch to mission mode after 10 iterations
        if (counter_ == 10) {this->mission_mode(); this->arm();}

        // Arm
        if (counter_ > 10 && counter_ <= 100) this->publish_control_setpoint(0.0, 0.0, 0.0, 0.45);

        // Take off
        if (counter_ > 100 && counter_ <= 200) {
            Point3D p = path_[0];
            pid_controller(p.x, p.y, p.z, 90.0, true);
            this->publish_control_setpoint(u[0], u[1], u[2], u[3]);

            log_file_ << p.x << "," << p.y << "," << p.z << ","
                          << local_position_msg_[0] << "," << local_position_msg_[1] << "," << local_position_msg_[2] << "\n";
        }

        // Stop counter
        if (counter_ < 201) counter_++;

        // Follow trajectory
        if (counter_ > 200) {
            if (path_index_ < path_.size()) {
                Point3D p = path_[path_index_];
                pid_controller(p.x, p.y, p.z, 90.0, false);
                this->publish_control_setpoint(u[0], u[1], u[2], u[3]);
                path_index_++;

                log_file_ << p.x << "," << p.y << "," << p.z << ","
                          << local_position_msg_[0] << "," << local_position_msg_[1] << "," << local_position_msg_[2] << "\n";
            } else if (path_index_ == path_.size() && counter_ < 350) {
                counter_++;
                Point3D p = path_[path_.size() - 1];
                pid_controller(p.x, p.y, p.z, 90.0, true);
                this->publish_control_setpoint(u[0], u[1], u[2], u[3]);

                log_file_ << p.x << "," << p.y << "," << p.z << ","
                          << local_position_msg_[0] << "," << local_position_msg_[1] << "," << local_position_msg_[2] << "\n";
            } else if (counter_ == 350) {
                // Land
                this->land();
                counter_++;
                // Log the current state
                RCLCPP_INFO(this->get_logger(), "x = %.2f, y = %.2f, z = %.2f, yaw = %.2f", 
                    local_position_msg_[0], local_position_msg_[1], local_position_msg_[2], attitude_msg_[2]);
                RCLCPP_INFO(this->get_logger(), "Mission completed, landing.");
                log_file_.close();
            }
        }
    };
    // Create a timer that calls the callback every 50 milliseconds
    timer_ = this->create_wall_timer(50ms, timer_callback);
}

void Control::pid_controller(float_t x_d, float_t y_d, float_t z_d, float_t yaw_d, bool hover)
{
    // Compute position errors
    float_t p_eX =   x_d - local_position_msg_[0];
    float_t p_eY = -(y_d - local_position_msg_[1]);
    float_t p_eZ =   z_d - local_position_msg_[2];
    float_t p_eYaw = yaw_d - attitude_msg_[2];

    // Initialize static variables for PID
    static float_t i_eX = 0.0, last_eX = 0.0;
    static float_t i_eY = 0.0, last_eY = 0.0;
    static float_t i_eZ = 0.0, last_eZ = 0.0;
    static float_t i_eYaw = 0.0, last_eYaw = 0.0;
    static float_t time_step = 0.05;   // Time step for control loop

    // Compute derivative of errors (D)
    float_t d_eX = (p_eX - last_eX) / time_step;
    float_t d_eY = (p_eY - last_eY) / time_step;
    float_t d_eZ = (p_eZ - last_eZ) / time_step;
    float_t d_eYaw = (p_eYaw - last_eYaw) / time_step;

    if (hover) {
        // Compute integral errors (I)
        i_eX += p_eX * time_step;
        i_eY += p_eY * time_step;
        i_eZ += p_eZ * time_step;
        i_eYaw += p_eYaw * time_step;

        // Limit integral errors
        if (abs(p_eX) > 0.2) i_eX = 0.0;
        if (abs(p_eY) > 0.2) i_eY = 0.0;
        if (abs(p_eZ) > 0.2) i_eZ = 0.0;
        if (abs(p_eYaw) > 0.2) i_eYaw = 0.0;

    } else {
        // Reset integral errors when not hovering
        i_eX = 0.0;
        i_eY = 0.0;
        i_eZ = 0.0;
        i_eYaw = 0.0;
    }

    // Save last error values for derivative calculation
    last_eX = p_eX;
    last_eY = p_eY;
    last_eZ = p_eZ;
    last_eYaw = p_eYaw;

    // Compute control inputs using PID
    u[0] = p_eX * 0.3 + i_eX * 0.1 + d_eX * 0.6;                 // PID cho Roll
    u[1] = p_eY * 0.3 + i_eY * 0.1 + d_eY * 0.6;                 // PID cho Pitch
    u[2] = p_eYaw * 0.005 + i_eYaw * 0.001 + d_eYaw * 0.005;     // PID cho Yaw
    u[3] = 0.4 + (p_eZ * 0.35 + i_eZ * 0.1 + d_eZ * 0.2);        // PID cho Throttle

    // Clamp control inputs to valid ranges
    for (int i = 0; i < 3; i++) u[i] = std::clamp(u[i], -0.3f, 0.3f);
    u[3] = std::clamp(u[3], -0.5f, 0.8f);

    // Special case: if yaw error is too large, set roll and pitch to zero
    if (p_eYaw > 10) 
    {
        u[0] = 0.0;
        u[1] = 0.0;
    }
}

void Control::arm()
{
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1);
    RCLCPP_INFO(this->get_logger(), "Arm command sent");
}

void Control::disarm()
{
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0);
    RCLCPP_INFO(this->get_logger(), "Disarm command sent");
}

void Control::land()
{
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_LAND, 0, 0);
    RCLCPP_INFO(this->get_logger(), "Land command sent");
}

void Control::mission_mode()
{
    this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 1);
    RCLCPP_INFO(this->get_logger(), "Mission mode command sent");
}

void Control::publish_vehicle_command(uint16_t command, float param1, float param2)
{
    VehicleCommand msg{};
    msg.param1 = param1;
    msg.param2 = param2;
    msg.command = command;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    vehicle_command_pub_->publish(msg);
}

void Control::publish_control_setpoint(float roll, float pitch, float yaw, float throttle)
{
    ManualControlSetpoint msg{};
    msg.roll = pitch;  // ned to enu conversion
    msg.pitch = roll;  // ned to enu conversion
    msg.yaw = yaw;
    msg.throttle = throttle;
    msg.valid = true;
    msg.data_source = px4_msgs::msg::ManualControlSetpoint::SOURCE_MAVLINK_0;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    msg.timestamp_sample = msg.timestamp;
    control_setpoint_pub_->publish(msg);
}

int main(int argc, char *argv[])
{
    std::cout << "Starting mission node..." << std::endl;
    
    // Load obstacles from the world file
    std::vector<Obstacle> obstacles = Planning::loadObstaclesFromWorld(
        "/home/hai/DevPX4/uav_px4/my_world/my_world.world", 0.5);

    // Create path planning object
    Planning planner(obstacles);

    Point3D start(0.0, 0.0, 2.0);
    Point3D goal(-8.0, -8.0, 2.0);

    // Find path using Bi-RRT
    std::vector<Point3D> path = planner.biRRT(start, goal, 10000);
    std::vector<Point3D> simple_path = planner.simplifyPath(path);
    std::vector<Point3D> dense_path = planner.densifyPath(simple_path, 1000);
    std::cout << "Path size: " << dense_path.size() << std::endl;

    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Control>(dense_path));
    rclcpp::shutdown();
    return 0;
}