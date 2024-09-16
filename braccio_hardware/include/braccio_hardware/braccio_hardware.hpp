#ifndef BRACCIO_HARDWARE_HPP_
#define BRACCIO_HARDWARE_HPP_

#include <termios.h>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "braccio_hardware/visibility_control.hpp"
#include "braccio_hardware/arduino_comms.hpp"
#include "braccio_hardware/hardware.hpp"


namespace braccio_hardware
{
class BraccioHardware : public hardware_interface::SystemInterface
{
    struct Config
    {
        //float loop_rate = 0.0;
        std::string device_port = "";
        int baud_rate = 0;
        int timeout_ms = 0;
        bool hw_test = false;
   };
    
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(BraccioHardware)

        BRACCIO_HARDWARE_PUBLIC
        hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

        BRACCIO_HARDWARE_PUBLIC
        hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

        BRACCIO_HARDWARE_PUBLIC
        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

        BRACCIO_HARDWARE_PUBLIC
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        BRACCIO_HARDWARE_PUBLIC
        hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

        BRACCIO_HARDWARE_PUBLIC
        hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

        BRACCIO_HARDWARE_PUBLIC
        hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

        BRACCIO_HARDWARE_PUBLIC
        hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

    private:

        std::vector<double> hw_commands_;
        std::vector<double> hw_states_position_;
        std::vector<double> hw_states_velocity_;

        ArduinoComms comms_;
        Config cfg_;

        Joint joint0_;
        Joint joint1_;
        Joint joint2_;
        Joint joint3_;
        Joint joint4_;
        GripperJoint gripper_;
};
}

#endif
