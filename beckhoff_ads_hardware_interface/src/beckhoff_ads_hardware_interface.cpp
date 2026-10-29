// Copyright (c) 2025, b-robotized
// All rights reserved.
//
// Proprietary License
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential.
//
// Author: Nikola Banovic
// Contributor: Hajar Bartakh

#include <limits>
#include <vector>
#include <cstdint>
#include <algorithm> // std::transform
#include <utility>

#include "beckhoff_ads_hardware_interface/beckhoff_ads_hardware_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace beckhoff_ads_hardware_interface
{
    hardware_interface::CallbackReturn BeckhoffADSHardwareInterface::on_init(
        const hardware_interface::HardwareComponentInterfaceParams &params)
    {
        if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS)
        {
            return CallbackReturn::ERROR;
        }

        logging_throttle_clock_ = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);

        return CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn BeckhoffADSHardwareInterface::on_configure(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        // Before anything else: configure_ads_device() below REPLACES ads_device_, which destroys
        // the previous one. A handle left over from an earlier configure would then be released
        // against a freed device -- the same fault as on_shutdown. The layout vectors are cleared
        // again further down as part of building them; this is about WHEN, not whether.
        release_ads_handles();

        // Configure ADS Client Device
        if (!configure_ads_device())
        {
            RCLCPP_FATAL(getLogger(), "Failed to configure ADS device from URDF parameters.");
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Fill the ADSDataLayout vectors for read and write operations
        ads_read_layout_configure();
        ads_write_layout_configure();

        // Request handles for all symbolic PLC variable names
        RCLCPP_INFO(getLogger(), "Fetching ADS handles for configured PLC variables...");
        for (auto &layout : ads_item_layouts_read_)
        {
            try
            {
                layout.ads_handle_ref.emplace(ads_device_->GetHandle(layout.plc_name_symbolic));
                layout.ads_handle = *layout.ads_handle_ref.value();
            }
            catch (const std::exception &ex)
            {
                RCLCPP_ERROR(getLogger(), "\tADS Exception getting handle for '%s': %s. Read operations for this variable will fail.", layout.plc_name_symbolic.c_str(), ex.what());
            }
        }
        for (auto &layout : ads_item_layouts_write_)
        {
            try
            {
                layout.ads_handle_ref.emplace(ads_device_->GetHandle(layout.plc_name_symbolic));
                layout.ads_handle = *layout.ads_handle_ref.value();
            }
            catch (const std::exception &ex)
            {
                RCLCPP_ERROR(getLogger(), "\tADS Exception getting handle for '%s': %s. Write operations for this variable will fail.", layout.plc_name_symbolic.c_str(), ex.what());
            }
        }
        RCLCPP_INFO(getLogger(), "\tHandles acquired");

        // Pre-pack what we can for SUM read/write commands
        if (!build_sum_read_buffers())
        {
            RCLCPP_FATAL(getLogger(), "\tFailed to build ADS sum read buffer.");
            return hardware_interface::CallbackReturn::ERROR;
        }
        if (!build_sum_write_buffers())
        {
            RCLCPP_FATAL(getLogger(), "\tFailed to build ADS sum write buffer.");
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Link command interfaces to their corresponding state interfaces
        RCLCPP_INFO(getLogger(), "Linking command interfaces to state interfaces...");
        for (auto &command_layout : ads_item_layouts_write_)
        {
            for (const auto &state_layout : ads_item_layouts_read_)
            {
                if (command_layout.plc_name_symbolic != state_layout.plc_name_symbolic)
                {
                    continue;
                }
                // A command member and a state member at the same place in the same symbol are the
                // same PLC value
                for (const auto &command_member : command_layout.members)
                {
                    for (const auto &state_member : state_layout.members)
                    {
                        if (command_member.byte_offset == state_member.byte_offset)
                        {
                            // The pair is made of (command_interface_name,
                            // corresponding_state_interface_name)
                            command_layout.state_command_interfaces_map_.emplace(
                                command_member.interface_name, state_member.interface_name);
                        }
                    }
                }
            }
        }

        return CallbackReturn::SUCCESS;
    }

    bool BeckhoffADSHardwareInterface::build_sum_read_buffers()
    {
        num_items_read_ = ads_item_layouts_read_.size();
        if (num_items_read_ == 0)
        {
            RCLCPP_INFO(getLogger(), "No items to configure for ADS Sum READ.");
            return true;
        }
        RCLCPP_INFO(getLogger(), "Building ADS sum READ buffer...");

        size_t total_error_block_size = num_items_read_ * sizeof(uint32_t);
        size_t total_data_block_size = 0;
        for (const auto &layout : ads_item_layouts_read_)
        {
            total_data_block_size += layout.byte_size;
        }

        ads_buffer_sum_read_response_.resize(total_error_block_size + total_data_block_size);
        ads_buffer_sum_read_request_.clear();
        ads_read_instructions_.clear();  // a reconfigure rebuilds them

        size_t current_data_offset = 0;
        size_t current_error_offset = 0;

        for (auto &layout : ads_item_layouts_read_)
        {
            layout.offset_in_read_response_data = total_error_block_size + current_data_offset;
            layout.offset_in_read_response_error = current_error_offset;

            ADS_ITEM_REQ_HEADER header;
            header.indexGroup = ADSIGRP_SYM_VALBYHND;
            header.indexOffset = layout.ads_handle;
            header.NumBytesData = layout.byte_size;
            const uint8_t *ptr = reinterpret_cast<const uint8_t *>(&header);
            ads_buffer_sum_read_request_.insert(ads_buffer_sum_read_request_.end(), ptr, ptr + sizeof(ADS_ITEM_REQ_HEADER));

            // For interfaces targeting the same PLC symbol
            for (const auto &member : layout.members)
            {
                // Fill the read instruction vector
                ReadInstruction read_instruction;
                read_instruction.read_buffer_offset_error_code = layout.offset_in_read_response_error;
                read_instruction.read_buffer_offset_data =
                    layout.offset_in_read_response_data + member.byte_offset;
                read_instruction.plc_type = member.plc_type;
                read_instruction.state_interface_name = member.interface_name;

                // Ordered by ascending offset inside the PLC symbol, as layout.members is
                ads_read_instructions_.push_back(read_instruction);
            }

            current_data_offset += header.NumBytesData;
            current_error_offset += sizeof(uint32_t);
        }
        RCLCPP_INFO(getLogger(), "ADS Sum READ configured for %zu items. Request: %zu bytes, Response: %zu bytes.",
                    num_items_read_, ads_buffer_sum_read_request_.size(), ads_buffer_sum_read_response_.size());
        return true;
    }

    bool BeckhoffADSHardwareInterface::build_sum_write_buffers()
    {
        RCLCPP_INFO(getLogger(), "Building ADS sum WRITE buffer...");
        num_items_write_ = ads_item_layouts_write_.size();
        if (num_items_write_ == 0)
        {
            RCLCPP_INFO(getLogger(), "No items to configure for ADS Sum WRITE.");
            return true;
        }

        size_t total_data_size = 0;
        size_t total_header_size = num_items_write_ * sizeof(ADS_ITEM_REQ_HEADER);
        for (const auto &layout : ads_item_layouts_write_)
        {
            total_data_size += layout.byte_size;
        }

        // Zero-filled: bytes of a struct that no interface maps are written as 0.
        ads_buffer_sum_write_request_.assign(total_header_size + total_data_size, 0);
        ads_write_instructions_.clear();  // a reconfigure rebuilds them
        ads_buffer_sum_write_response_.resize(num_items_write_ * sizeof(uint32_t));

        auto *header_block_ptr = reinterpret_cast<ADS_ITEM_REQ_HEADER *>(ads_buffer_sum_write_request_.data());
        size_t current_data_offset = 0;
        size_t i = 0; // Index for the header block pointer

        for (auto &layout : ads_item_layouts_write_)
        {
            header_block_ptr[i].indexGroup = ADSIGRP_SYM_VALBYHND;
            header_block_ptr[i].indexOffset = layout.ads_handle;
            header_block_ptr[i].NumBytesData = layout.byte_size;

            layout.offset_in_write_request_data = total_header_size + current_data_offset;

            // For interfaces targeting the same PLC symbol
            for (const auto &member : layout.members)
            {
                // Fill the write instruction vector
                WriteInstruction write_instruction;
                write_instruction.write_buffer_offset_data =
                    layout.offset_in_write_request_data + member.byte_offset;
                write_instruction.plc_type = member.plc_type;
                write_instruction.command_interface_name = member.interface_name;
                write_instruction.fallback_state_interface_name = "";
                write_instruction.auto_increment = member.auto_increment;
                write_instruction.pulse_on_rising_edge = member.pulse_on_rising_edge;

                // There exists a state interface for the same PLC value
                const auto fallback =
                    layout.state_command_interfaces_map_.find(member.interface_name);
                if (fallback != layout.state_command_interfaces_map_.end())
                {
                    write_instruction.fallback_state_interface_name = fallback->second;
                }

                // Ordered by ascending offset inside the PLC symbol, as layout.members is
                ads_write_instructions_.push_back(write_instruction);
            }

            current_data_offset += header_block_ptr[i].NumBytesData;
            i++;
        }

        RCLCPP_INFO(getLogger(), "ADS Sum WRITE configured for %zu items. Request: %zu bytes, Response: %zu bytes.",
                    num_items_write_, ads_buffer_sum_write_request_.size(), ads_buffer_sum_write_response_.size());
        return true;
    }

    void BeckhoffADSHardwareInterface::ads_read_layout_configure()
    {
        std::vector<InterfaceSpec> specs;
        for (const auto *interfaces :
             {&joint_state_interfaces_, &gpio_state_interfaces_, &sensor_state_interfaces_})
        {
            const auto type_specs = interface_specs(*interfaces, false);
            specs.insert(specs.end(), type_specs.begin(), type_specs.end());
        }
        layouts_from_specs(specs, ads_item_layouts_read_);
    }

    std::vector<InterfaceSpec> BeckhoffADSHardwareInterface::interface_specs(
        const std::unordered_map<std::string, hardware_interface::InterfaceDescription> &interfaces,
        bool command)
    {
        std::vector<InterfaceSpec> specs;
        for (const auto &[name, descr] : interfaces)
        {
            const auto &parameters = descr.interface_info.parameters;
            InterfaceSpec spec;
            spec.name = name;
            try
            {
                spec.plc_symbol = parameters.at("PLC_symbol");
                spec.plc_type = plc_type_from_string(parameters.at("PLC_type"));
                if (parameters.count("n_elements"))
                {
                    spec.n_elements = std::stoul(parameters.at("n_elements"));
                }
                if (parameters.count("index"))
                {
                    spec.index = std::stoul(parameters.at("index"));
                }
                if (parameters.count("byte_offset"))
                {
                    spec.byte_offset = std::stoul(parameters.at("byte_offset"));
                }
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(getLogger(),
                             "Error parsing PLC parameters for %s interface '%s': %s. Check URDF.",
                             command ? "command" : "state", name.c_str(), e.what());
                continue;
            }
            // Command-only boolean flags.
            bool valid = true;
            for (const auto &[flag_name, field] :
                 {std::pair<const char *, bool *>{"auto_increment", &spec.auto_increment},
                  std::pair<const char *, bool *>{"pulse_on_rising_edge",
                                                  &spec.pulse_on_rising_edge}})
            {
                if (!parameters.count(flag_name))
                {
                    continue;
                }
                std::string flag = parameters.at(flag_name);
                std::transform(flag.begin(), flag.end(), flag.begin(), ::tolower);
                if (!command)
                {
                    RCLCPP_WARN(getLogger(),
                                "Ignoring '%s' on state interface '%s': only a command is written.",
                                flag_name, name.c_str());
                }
                else if (flag == "true" || flag == "1")
                {
                    *field = true;
                }
                else if (flag != "false" && flag != "0")
                {
                    RCLCPP_ERROR(getLogger(),
                                 "Invalid '%s' ('%s') for command interface '%s'; expected true or "
                                 "false. Skipping it.",
                                 flag_name, parameters.at(flag_name).c_str(), name.c_str());
                    valid = false;
                }
            }
            if (valid)
            {
                specs.push_back(spec);
            }
        }
        return specs;
    }

    void BeckhoffADSHardwareInterface::layouts_from_specs(const std::vector<InterfaceSpec> &specs,
                                                          std::vector<ADSDataLayout> &layouts)
    {
        std::vector<std::string> errors;
        const auto items = group_items(specs, errors);
        for (const auto &error : errors)
        {
            RCLCPP_ERROR(getLogger(), "Skipping an interface: %s", error.c_str());
        }

        layouts.clear();
        layouts.reserve(items.size());
        for (const auto &item : items)
        {
            ADSDataLayout layout;
            layout.plc_name_symbolic = item.plc_symbol;
            layout.byte_size = item.byte_size;
            layout.members = item.members;
            layouts.push_back(std::move(layout));
        }
    }

    void BeckhoffADSHardwareInterface::ads_write_layout_configure()
    {
        // Rebuilt from the URDF below; a stale entry would drive a value for an interface this
        // configure no longer declares.
        ads_shutdown_values_.clear();

        auto init_ads_write_layout =
            [&](const auto &type_command_interfaces_)
        {
            for (const auto &[name, descr] : type_command_interfaces_)
            {
                [[maybe_unused]] double initial_value = std::numeric_limits<double>::quiet_NaN();

                if (descr.interface_info.parameters.count("initial_value"))
                {
                    try
                    {
                        initial_value = std::stod(descr.interface_info.parameters.at("initial_value"));
                    }
                    catch (const std::exception &ex)
                    { // Catch conversion errors
                        RCLCPP_WARN(
                            getLogger(),
                            "Invalid 'initial_value' ('%s') for command interface '%s'. Using NaN. Error: %s",
                            descr.interface_info.parameters.at("initial_value").c_str(),
                            name.c_str(),
                            ex.what());
                    }
                }

                // Optional: the value to drive onto the PLC when this component deactivates.
                // Parsed here, beside initial_value, because this is the one place that sees each
                // command interface's parameters. Applied in on_deactivate(), not here.
                if (descr.interface_info.parameters.count("shutdown_value"))
                {
                    try
                    {
                        ads_shutdown_values_[name] = std::stod(descr.interface_info.parameters.at("shutdown_value"));
                    }
                    catch (const std::exception &ex)
                    {
                        RCLCPP_WARN(
                            getLogger(),
                            "Invalid 'shutdown_value' ('%s') for command interface '%s'. Ignoring it, so this "
                            "interface will NOT be driven on deactivate. Error: %s",
                            descr.interface_info.parameters.at("shutdown_value").c_str(),
                            name.c_str(),
                            ex.what());
                    }
                }
            }
        };

        init_ads_write_layout(joint_command_interfaces_);
        init_ads_write_layout(gpio_command_interfaces_);

        std::vector<InterfaceSpec> specs = interface_specs(joint_command_interfaces_, true);
        const auto gpio_specs = interface_specs(gpio_command_interfaces_, true);
        specs.insert(specs.end(), gpio_specs.begin(), gpio_specs.end());
        layouts_from_specs(specs, ads_item_layouts_write_);
    }

    hardware_interface::CallbackReturn BeckhoffADSHardwareInterface::on_activate(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        return CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn BeckhoffADSHardwareInterface::on_deactivate(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        // Drive every declared shutdown_value onto the PLC, in ONE final sum-write.
        //
        // This has to happen here rather than being published by a cooperating node on its way out.
        // Controllers deactivate only a few hundred microseconds before the hardware does, which at
        // any realistic update rate is less than one cycle -- so a command value set that late has
        // no write() left to carry it. Doing it inside on_deactivate() is ordered by construction:
        // the device and its handles are still alive (they are released in on_shutdown, which runs
        // after this), and the write() below is the very last one.
        //
        // Reuses write() rather than repeating the packing: setting the commands first means write()
        // picks them up exactly as it would a live command. Interfaces with no shutdown_value are
        // left alone and keep their normal fallback behaviour.
        if (!ads_shutdown_values_.empty() && ads_device_ && num_items_write_ > 0)
        {
            for (const auto &[interface_name, value] : ads_shutdown_values_)
            {
                set_command(interface_name, value);
                RCLCPP_INFO(getLogger(), "Shutdown value: driving '%s' to %g", interface_name.c_str(), value);
            }

            if (write(rclcpp::Time{}, rclcpp::Duration{0, 0}) != hardware_interface::return_type::OK)
            {
                // Not fatal: deactivation must still complete, and refusing to deactivate would leave
                // the component worse off than a PLC that missed one write. Loud, because a stop
                // request that did not land is exactly what someone needs to know about.
                RCLCPP_ERROR(getLogger(), "Failed to write shutdown values to the PLC; they may NOT have taken effect.");
            }
        }

        return CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type BeckhoffADSHardwareInterface::read(
        const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        if (num_items_read_ == 0)
        {
            return hardware_interface::return_type::OK;
        }

        uint32_t bytes_read_from_plc = 0;
        long ads_sum_read_error = ads_device_->ReadWriteReqEx2(
            ADSIGRP_SUMUP_READ,
            num_items_read_,
            ads_buffer_sum_read_response_.size(),
            ads_buffer_sum_read_response_.data(),
            ads_buffer_sum_read_request_.size(),
            ads_buffer_sum_read_request_.data(),
            &bytes_read_from_plc);

        if (ads_sum_read_error != ADSERR_NOERR)
        {
            RCLCPP_ERROR_THROTTLE(getLogger(), *logging_throttle_clock_, 1000,
                                  "Overall ADS Sum Read Error: 0x%lX.", ads_sum_read_error);
            // TODO: See if we need to do something on read error. Maybe assign NaN to all interfaces?
            return hardware_interface::return_type::ERROR;
        }

        if (bytes_read_from_plc != ads_buffer_sum_read_response_.size())
        {
            RCLCPP_ERROR_THROTTLE(getLogger(), *logging_throttle_clock_, 1000,
                                  "ADS Sum Read size mismatch. Expected %zu, Got %u.",
                                  ads_buffer_sum_read_response_.size(), bytes_read_from_plc);
            return hardware_interface::return_type::ERROR;
        }

        bool any_item_read_failed = false;
        for (const auto &read_instruction : ads_read_instructions_)
        {
            uint32_t item_error_code;
            memcpy(&item_error_code,
                   ads_buffer_sum_read_response_.data() + read_instruction.read_buffer_offset_error_code,
                   sizeof(uint32_t));

            if (item_error_code != ADSERR_NOERR)
            {
                RCLCPP_WARN_THROTTLE(getLogger(), *logging_throttle_clock_, 1000,
                                     "ADS Sum Read operation corresponding to the state interface '%s' failed: 0x%X.",
                                     read_instruction.state_interface_name.c_str(), item_error_code);

                // TODO: See if we need to do something on read error. Maybe assign NaN to the interface?
                any_item_read_failed = true;
                continue;
            }

            // Each state interface has its corresponding read_instruction
            const uint8_t *ptr_plc_element_current = ads_buffer_sum_read_response_.data() + read_instruction.read_buffer_offset_data;

            double val;
            if (!decode_plc_value(read_instruction.plc_type, ptr_plc_element_current, val))
            {
                RCLCPP_ERROR_THROTTLE(getLogger(), *logging_throttle_clock_, 1000,
                                      "Unhandled or UNKNOWN PLC type (%d) for the interface '%s' during read.",
                                      static_cast<int>(read_instruction.plc_type), read_instruction.state_interface_name.c_str());
                set_state(read_instruction.state_interface_name, std::numeric_limits<double>::quiet_NaN());
                any_item_read_failed = true;
                continue;
            }
            set_state(read_instruction.state_interface_name, val);
        }
        return any_item_read_failed ? hardware_interface::return_type::ERROR : hardware_interface::return_type::OK;
    }

    hardware_interface::return_type BeckhoffADSHardwareInterface::write(
        const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        if (num_items_write_ == 0)
        {
            return hardware_interface::return_type::OK;
        }

        for (auto &write_instruction : ads_write_instructions_)
        {
            uint8_t *ptr_write_buffer_destination_current = ads_buffer_sum_write_request_.data() + write_instruction.write_buffer_offset_data;

            if (write_instruction.auto_increment)
            {
                // A counter the component owns: one more every write, whatever the command says,
                // so the PLC sees each sample of the item as new.
                write_instruction.counter_value =
                    next_counter_value(write_instruction.counter_value, write_instruction.plc_type);
                encode_plc_value(write_instruction.plc_type, write_instruction.counter_value,
                                 ptr_write_buffer_destination_current);
                continue;
            }

            if (write_instruction.pulse_on_rising_edge)
            {
                // Only a CHANGE of the command reaches the PLC as a 1, for this one write: a held
                // command (the gpio controller rewrites its last one every cycle) is no proof that
                // its sender is still alive, an edge is.
                const double command = get_command(write_instruction.command_interface_name);
                set_command(write_instruction.command_interface_name,
                            std::numeric_limits<double>::quiet_NaN());
                const bool pulse = rising_edge(write_instruction.last_command, command);
                if (!std::isnan(command))
                {
                    write_instruction.last_command = command;
                }
                encode_plc_value(write_instruction.plc_type, pulse ? 1.0 : 0.0,
                                 ptr_write_buffer_destination_current);
                continue;
            }

            // store the current val and reset the ros-side command value
            double val = get_command(write_instruction.command_interface_name);
            set_command(write_instruction.command_interface_name, std::numeric_limits<double>::quiet_NaN());

            if (std::isnan(val))
            {
                // if the original value was NaN and there exist a state interface of the same name, write corresponding state interface
                if (!write_instruction.fallback_state_interface_name.empty())
                {
                    val = get_state(write_instruction.fallback_state_interface_name);
                }

                // if we STILL don't have a fallback value on, don't update the write buffer.
                // the last valid command is written
                if (std::isnan(val))
                {
                    continue;
                }
            }

            if (!encode_plc_value(write_instruction.plc_type, val,
                                  ptr_write_buffer_destination_current))
            {
                RCLCPP_FATAL(getLogger(),
                             "UNKNOWN PLC type (%d) for the interface '%s' during write.",
                             static_cast<int>(write_instruction.plc_type),
                             write_instruction.command_interface_name.c_str());
                return hardware_interface::return_type::ERROR;
            }
        }

        uint32_t bytes_response_buffer_from_plc = 0;
        long ads_sum_write_error = ads_device_->ReadWriteReqEx2(
            ADSIGRP_SUMUP_WRITE,
            num_items_write_,
            ads_buffer_sum_write_response_.size(),
            ads_buffer_sum_write_response_.data(),
            ads_buffer_sum_write_request_.size(),
            ads_buffer_sum_write_request_.data(),
            &bytes_response_buffer_from_plc);

        if (ads_sum_write_error != ADSERR_NOERR)
        {
            RCLCPP_ERROR_THROTTLE(getLogger(), *logging_throttle_clock_, 1000,
                                  "Overall ADS Sum Write Error: 0x%lX.", ads_sum_write_error);
            return hardware_interface::return_type::ERROR;
        }

        if (bytes_response_buffer_from_plc != ads_buffer_sum_write_response_.size())
        {
            RCLCPP_ERROR_THROTTLE(getLogger(), *logging_throttle_clock_, 1000,
                                  "ADS Sum Write response size mismatch (error codes). Expected %zu, Got %u.",
                                  ads_buffer_sum_write_response_.size(), bytes_response_buffer_from_plc);
        }

        bool any_item_write_failed = false;
        for (size_t i = 0; i < num_items_write_; ++i)
        {
            uint32_t item_error_code;
            memcpy(&item_error_code, ads_buffer_sum_write_response_.data() + i * sizeof(uint32_t), sizeof(uint32_t));
            if (item_error_code != ADSERR_NOERR)
            {
                RCLCPP_WARN_THROTTLE(getLogger(), *logging_throttle_clock_, 1000,
                                     "ADS Sum Write sub-op for '%s' (handle 0x%X) failed: 0x%X",
                                     ads_item_layouts_write_[i].plc_name_symbolic.c_str(), ads_item_layouts_write_[i].ads_handle, item_error_code);
                any_item_write_failed = true;
            }
        }

        return any_item_write_failed ? hardware_interface::return_type::ERROR : hardware_interface::return_type::OK;
    }

    hardware_interface::CallbackReturn BeckhoffADSHardwareInterface::on_shutdown(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(getLogger(), "Releasing ADS resources...");
        // Handles first, device second. The other order leaves every optional<AdsHandle> in the
        // layout vectors holding a deleter bound to a freed AdsDevice; they then run at component
        // destruction, inside ~ResourceManager, and segfault in DeleteSymbolHandle -> WriteReqEx
        // -> GetLocalPort() on a dangling this.
        release_ads_handles();
        if (ads_device_)
        {
            ads_device_.reset();
        }
        RCLCPP_INFO(getLogger(), "ADS resources released.");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    void BeckhoffADSHardwareInterface::release_ads_handles()
    {
        // Clearing the layouts is what actually frees the handles: destroying each
        // optional<AdsHandle> runs the deleter that issues DeleteSymbolHandle on the PLC.
        ads_item_layouts_read_.clear();
        ads_item_layouts_write_.clear();
    }

    bool BeckhoffADSHardwareInterface::configure_ads_device()
    {
        RCLCPP_INFO(getLogger(), "Configuring ADS device...");
        const auto &params = info_.hardware_parameters;
        try
        {
            std::string plc_ip = params.at("plc_ip_address");
            std::string plc_ams_net_id_str = params.at("plc_ams_net_id");
            std::string local_ams_net_id_str = params.at("local_ams_net_id");
            uint16_t plc_ams_port = std::stoul(params.at("plc_ams_port"));

            AmsNetId remote_net_id;
            if (sscanf(plc_ams_net_id_str.c_str(), "%hhu.%hhu.%hhu.%hhu.%hhu.%hhu",
                       &remote_net_id.b[0], &remote_net_id.b[1], &remote_net_id.b[2],
                       &remote_net_id.b[3], &remote_net_id.b[4], &remote_net_id.b[5]) != 6)
            {
                RCLCPP_FATAL(getLogger(), "\tInvalid format for 'plc_ams_net_id'. Expected 'x.x.x.x.x.x'.");
                return false;
            }

            AmsNetId local_net_id;
            if (sscanf(local_ams_net_id_str.c_str(), "%hhu.%hhu.%hhu.%hhu.%hhu.%hhu",
                       &local_net_id.b[0], &local_net_id.b[1], &local_net_id.b[2],
                       &local_net_id.b[3], &local_net_id.b[4], &local_net_id.b[5]) != 6)
            {
                RCLCPP_FATAL(getLogger(), "\tInvalid format for 'local_ams_net_id'. Expected 'x.x.x.x.x.x'.");
                return false;
            }

            bhf::ads::SetLocalAddress(local_net_id);
            ads_device_ = std::make_unique<AdsDevice>(plc_ip, remote_net_id, plc_ams_port);
            RCLCPP_INFO(getLogger(), "\tTimeout is: %u", ads_device_->GetTimeout());

            RCLCPP_INFO(getLogger(), "\tADS Device configured for PLC: %s, Port: %u", plc_ip.c_str(), plc_ams_port);
            RCLCPP_INFO(getLogger(), "\tPLC AMS NetID: %s", plc_ams_net_id_str.c_str());

            RCLCPP_INFO(getLogger(), "Requesting Device state...");
            AdsDeviceState deviceState = ads_device_->GetState();
            RCLCPP_INFO(getLogger(), "\tCommunication successful! ADS State: %d, DeviceState: %d", deviceState.ads, deviceState.device);
        }
        catch (const std::out_of_range &ex)
        {
            RCLCPP_FATAL(getLogger(), "\tMissing required URDF <hardware> parameter: %s", ex.what());
            return false;
        }
        catch (const AdsException &ex)
        {
            RCLCPP_FATAL(getLogger(), "\tADS Exception during connection: %s (Error Code: 0x%lX)", ex.what(), ex.errorCode);
            return false;
        }
        catch (const std::exception &ex)
        {
            RCLCPP_FATAL(getLogger(), "\tError during ADS connection config: %s", ex.what());
            return false;
        }

        return true;
    }
} // namespace beckhoff_ads_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    beckhoff_ads_hardware_interface::BeckhoffADSHardwareInterface, hardware_interface::SystemInterface)
