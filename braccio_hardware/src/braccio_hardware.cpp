#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <chrono>

#include "braccio_hardware/braccio_hardware.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace braccio_hardware
{
    // ================================
    // Initialization function state
    // ================================
    hardware_interface::CallbackReturn BraccioHardware::on_init(const hardware_interface::HardwareInfo & info)
    {
        if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }

        hw_states_position_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        hw_states_velocity_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

        for (const hardware_interface::ComponentInfo & joint : info_.joints)
        {
            if (joint.command_interfaces.size() != 1)
            {
                RCLCPP_FATAL(rclcpp::get_logger("BraccioInterface"),
                "joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(), joint.command_interfaces.size());
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
            {
                RCLCPP_FATAL(rclcpp::get_logger("BraccioInterface"),
                "Joint '%s' has %s command interfaces found. '%s' expected.", joint.name.c_str(), joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces.size() != 2)
            {
                RCLCPP_FATAL(rclcpp::get_logger("BraccioInterface"),
                "joint '%s' has %zu state interfaces found. 2 expected.", joint.name.c_str(), joint.state_interfaces.size());
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
            {
                RCLCPP_FATAL(rclcpp::get_logger("BraccioInterface"),
                "Joint '%s' has %s state interfaces found. '%s' expected.", joint.name.c_str(), joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
            {
                RCLCPP_FATAL(rclcpp::get_logger("BraccioInterface"),
                "Joint '%s' has %s state interfaces found. '%s' expected.", joint.name.c_str(), joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
                return hardware_interface::CallbackReturn::ERROR;
            }
        }

        cfg_.device_port = info_.hardware_parameters["device_port"];
        cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);;
        cfg_.timeout_ms = std::stoi(info_.hardware_parameters["timeout_ms"]);;
        if (info_.hardware_parameters["hw_test"] == "True")
        {
            cfg_.hw_test = true;
        }
        else
        {
            cfg_.hw_test = false;
        }
        
        RCLCPP_INFO(rclcpp::get_logger("BraccioInterface"), "Verbosity status: %i", cfg_.hw_test);

        return hardware_interface::CallbackReturn::SUCCESS;

    }

    // ================================
    // Configuration function state
    // ================================
    hardware_interface::CallbackReturn BraccioHardware::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
    {
            
        if (!comms_.connect(cfg_.device_port, cfg_.baud_rate, cfg_.timeout_ms))
        {
            RCLCPP_FATAL(rclcpp::get_logger("BraccioInterface"), "Arduino connection NOT stablished");
            return hardware_interface::CallbackReturn::ERROR;
        }

        RCLCPP_INFO(rclcpp::get_logger("BraccioInterface"), "Successfully configured!");

        return hardware_interface::CallbackReturn::SUCCESS;

    }

    // ================================
    // Activation function state
    // ================================
    hardware_interface::CallbackReturn BraccioHardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
    {

        if (!comms_.connected(cfg_.hw_test))
        {
            RCLCPP_FATAL(rclcpp::get_logger("BraccioInterface"), "Arduino NOT connected");
            return hardware_interface::CallbackReturn::ERROR;
        }

        hw_commands_[0] = 1.5708;
        hw_commands_[1] = 0.7854;
        hw_commands_[2] = 3.1416;
        hw_commands_[3] = 3.1416;
        hw_commands_[4] = 1.5708;
        hw_commands_[5] = 0.1745;
        
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    // ================================
    // Deactivation function state
    // ================================
    hardware_interface::CallbackReturn BraccioHardware::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
    {

        comms_.disconnect();
        
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    // ================================
    // Export state interf. function
    // ================================
    std::vector<hardware_interface::StateInterface> BraccioHardware::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        for (uint i = 0; i < info_.joints.size(); i++)
        {
            state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_position_[i]));

            state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocity_[i]));
        }
        return state_interfaces;
    }

    // ================================
    // Export command interf. function
    // ================================
    std::vector<hardware_interface::CommandInterface> BraccioHardware::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        for (uint i = 0; i < info_.joints.size(); i++)
        {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
        }
        return command_interfaces;
    }

    // ================================
    // Read function state
    // ================================
    hardware_interface::return_type BraccioHardware::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
    {

        comms_.read_joints_values(cfg_.hw_test, joint0_.pos, joint1_.pos, joint2_.pos, joint3_.pos, joint4_.pos, gripper_.pos);

        // Store joint values for later velocity computation
        //float pos_prev[] = {joint0_.pos, joint1_.pos, joint2_.pos, joint3_.pos, joint4_.pos, gripper_.pos};
        double pos_prev[] = {hw_states_position_[0], hw_states_position_[1], hw_states_position_[2], hw_states_position_[3], hw_states_position_[4], hw_states_position_[5]};

        // JointX_.pos (float) in deg to HW_states_position_ (vector<double>) in rads.
        hw_states_position_[0] = joint0_.pos * 3.1416 / 180;
        hw_states_position_[1] = joint1_.pos * 3.1416 / 180;
        hw_states_position_[2] = joint2_.pos * 3.1416 / 180;
        hw_states_position_[3] = joint3_.pos * 3.1416 / 180;
        hw_states_position_[4] = joint4_.pos * 3.1416 / 180;
        hw_states_position_[5] = gripper_.pos * 3.1416 / 180;

        // Compute velocity state
        float delta_seconds = period.seconds();

        hw_states_velocity_[0] = (hw_states_position_[0] - pos_prev[0]) / delta_seconds;
        hw_states_velocity_[1] = (hw_states_position_[1] - pos_prev[1]) / delta_seconds;
        hw_states_velocity_[2] = (hw_states_position_[2] - pos_prev[2]) / delta_seconds;
        hw_states_velocity_[3] = (hw_states_position_[3] - pos_prev[3]) / delta_seconds;
        hw_states_velocity_[4] = (hw_states_position_[4] - pos_prev[4]) / delta_seconds;
        hw_states_velocity_[5] = (hw_states_position_[5] - pos_prev[5]) / delta_seconds;


        // Report
        if (cfg_.hw_test)
        {
            RCLCPP_INFO(rclcpp::get_logger("BraccioInterface"), "Delta time (sec): %f", delta_seconds);

            RCLCPP_INFO(rclcpp::get_logger("BraccioInterface"), "Received (deg): %i, %i, %i, %i, %i, %i", joint0_.pos, joint1_.pos, joint2_.pos, joint3_.pos, joint4_.pos, gripper_.pos);

            RCLCPP_INFO(rclcpp::get_logger("BraccioInterface"), "Received (rads): %f, %f, %f, %f, %f, %f", hw_states_position_[0], hw_states_position_[1], hw_states_position_[2], hw_states_position_[3], hw_states_position_[4], hw_states_position_[5]);
            RCLCPP_INFO(rclcpp::get_logger("BraccioInterface"), "Received (rads/s): %f, %f, %f, %f, %f, %f", hw_states_velocity_[0], hw_states_velocity_[1], hw_states_velocity_[2], hw_states_velocity_[3], hw_states_velocity_[4], hw_states_velocity_[5]);
        }

        return hardware_interface::return_type::OK;
    }

    // ================================
    // Write function state
    // ================================
    hardware_interface::return_type BraccioHardware::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        if (cfg_.hw_test)
        {
            RCLCPP_INFO(rclcpp::get_logger("BraccioInterface"), "Sending (rads): %f, %f, %f, %f, %f, %f", hw_commands_[0], hw_commands_[1], hw_commands_[2], hw_commands_[3], hw_commands_[4], hw_commands_[5]);
        }

        // HW_COOMANDS_ (vector<double>) in rads to JOINTX_.CMD (float) in deg.
        joint0_.cmd = hw_commands_[0] * 180 / 3.1416;
        joint1_.cmd = hw_commands_[1] * 180 / 3.1416;
        joint2_.cmd = hw_commands_[2] * 180 / 3.1416;
        joint3_.cmd = hw_commands_[3] * 180 / 3.1416;
        joint4_.cmd = hw_commands_[4] * 180 / 3.1416;
        gripper_.cmd = hw_commands_[5] * 180 / 3.1416;

        comms_.set_joints_values(cfg_.hw_test, joint0_.cmd, joint1_.cmd, joint2_.cmd, joint3_.cmd, joint4_.cmd, gripper_.cmd);

        return hardware_interface::return_type::OK;
    }
}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(braccio_hardware::BraccioHardware, hardware_interface::SystemInterface)