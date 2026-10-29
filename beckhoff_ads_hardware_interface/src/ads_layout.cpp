// Copyright (c) 2025, b-robotized
// All rights reserved.
//
// Proprietary License
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential.

#include "beckhoff_ads_hardware_interface/ads_layout.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstring>

namespace beckhoff_ads_hardware_interface
{
    namespace
    {
        bool is_unsigned_integer(PLCType plc_type)
        {
            return plc_type == PLCType::USINT || plc_type == PLCType::BYTE ||
                   plc_type == PLCType::UINT || plc_type == PLCType::UDINT;
        }

        // An item while it is being built: the kind is fixed by the first interface on the symbol.
        struct PendingItem
        {
            ItemLayoutSpec item;
            bool is_struct = false;
            PLCType array_type = PLCType::UNKNOWN;
            size_t n_elements = 1;
        };

        std::string quoted(const std::string &name)
        {
            return "'" + name + "'";
        }

        // Empty when `spec` fits `pending`, else why not. `offset` is where its value then starts.
        std::string place(const PendingItem &pending, const InterfaceSpec &spec, size_t &offset)
        {
            const size_t size = plc_type_byte_size(spec.plc_type);
            if (pending.is_struct)
            {
                if (!spec.byte_offset)
                {
                    return "has no byte_offset, but " + quoted(pending.item.plc_symbol) +
                           " is addressed by byte_offset (a struct); index and byte_offset cannot "
                           "be mixed on one symbol";
                }
                offset = *spec.byte_offset;
                for (const auto &member : pending.item.members)
                {
                    const size_t member_end =
                        member.byte_offset + plc_type_byte_size(member.plc_type);
                    if (offset < member_end && member.byte_offset < offset + size)
                    {
                        return "overlaps " + quoted(member.interface_name) + " in " +
                               quoted(pending.item.plc_symbol);
                    }
                }
                return "";
            }

            if (spec.byte_offset)
            {
                return "has a byte_offset, but " + quoted(pending.item.plc_symbol) +
                       " is addressed by index (an array); index and byte_offset cannot be mixed "
                       "on one symbol";
            }
            if (spec.plc_type != pending.array_type)
            {
                return "has another PLC_type than the other elements of the array " +
                       quoted(pending.item.plc_symbol);
            }
            const size_t index = spec.index.value_or(0);
            if (index >= pending.n_elements)
            {
                return "index " + std::to_string(index) + " is outside the " +
                       std::to_string(pending.n_elements) + " elements of " +
                       quoted(pending.item.plc_symbol);
            }
            offset = index * size;
            for (const auto &member : pending.item.members)
            {
                if (member.byte_offset == offset)
                {
                    return "uses index " + std::to_string(index) + " of " +
                           quoted(pending.item.plc_symbol) + ", already taken by " +
                           quoted(member.interface_name);
                }
            }
            return "";
        }
    }  // namespace

    PLCType plc_type_from_string(const std::string &type_str)
    {
        std::string upper = type_str;
        std::transform(upper.begin(), upper.end(), upper.begin(),
                       [](unsigned char c)
                       {
                           return static_cast<char>(std::toupper(c));
                       });

        if (upper == "LREAL")
            return PLCType::LREAL;
        if (upper == "REAL")
            return PLCType::REAL;
        if (upper == "BOOL")
            return PLCType::BOOL;
        if (upper == "UDINT")
            return PLCType::UDINT;
        if (upper == "DINT")
            return PLCType::DINT;
        if (upper == "UINT")
            return PLCType::UINT;
        if (upper == "INT")
            return PLCType::INT;
        if (upper == "USINT")
            return PLCType::USINT;
        if (upper == "SINT")
            return PLCType::SINT;
        if (upper == "BYTE")
            return PLCType::BYTE;
        if (upper == "STRING")
            return PLCType::STRING;
        return PLCType::UNKNOWN;
    }

    size_t plc_type_byte_size(PLCType plc_type)
    {
        switch (plc_type)
        {
        case PLCType::LREAL:
            return 8;
        case PLCType::REAL:
        case PLCType::UDINT:
        case PLCType::DINT:
            return 4;
        case PLCType::UINT:
        case PLCType::INT:
            return 2;
        case PLCType::BOOL:
        case PLCType::USINT:
        case PLCType::SINT:
        case PLCType::BYTE:
            return 1;
        // case PLCType::STRING: not currently supported
        case PLCType::UNKNOWN:
        default:
            return 0;
        }
    }

    std::vector<ItemLayoutSpec> group_items(const std::vector<InterfaceSpec> &specs,
                                            std::vector<std::string> &errors)
    {
        std::vector<PendingItem> pending_items;

        for (const auto &spec : specs)
        {
            if (plc_type_byte_size(spec.plc_type) == 0)
            {
                errors.push_back("Interface " + quoted(spec.name) + " on " +
                                 quoted(spec.plc_symbol) + ": unsupported or unknown PLC_type.");
                continue;
            }
            if (spec.pulse_on_rising_edge &&
                (spec.plc_type != PLCType::BOOL || spec.auto_increment))
            {
                errors.push_back(
                    "Interface " + quoted(spec.name) +
                    ": pulse_on_rising_edge needs PLC_type BOOL and no auto_increment.");
                continue;
            }
            if (spec.auto_increment && !is_unsigned_integer(spec.plc_type))
            {
                errors.push_back("Interface " + quoted(spec.name) +
                                 ": auto_increment needs an unsigned integer PLC_type (USINT, "
                                 "BYTE, UINT, UDINT).");
                continue;
            }

            auto it = std::find_if(pending_items.begin(), pending_items.end(),
                                   [&spec](const PendingItem &pending)
                                   {
                                       return pending.item.plc_symbol == spec.plc_symbol;
                                   });
            PendingItem fresh;
            if (it == pending_items.end())
            {
                fresh.item.plc_symbol = spec.plc_symbol;
                fresh.is_struct = spec.byte_offset.has_value();
                fresh.array_type = spec.plc_type;
                fresh.n_elements = spec.n_elements;
            }
            const PendingItem &target = (it == pending_items.end()) ? fresh : *it;

            size_t offset = 0;
            const std::string why_not = place(target, spec, offset);
            if (!why_not.empty())
            {
                errors.push_back("Interface " + quoted(spec.name) + " " + why_not + ".");
                continue;
            }

            ItemMember member;
            member.interface_name = spec.name;
            member.byte_offset = offset;
            member.plc_type = spec.plc_type;
            member.auto_increment = spec.auto_increment;
            member.pulse_on_rising_edge = spec.pulse_on_rising_edge;
            if (it == pending_items.end())
            {
                fresh.item.members.push_back(member);
                pending_items.push_back(std::move(fresh));
            }
            else
            {
                it->item.members.push_back(member);
            }
        }

        std::vector<ItemLayoutSpec> items;
        items.reserve(pending_items.size());
        for (auto &pending : pending_items)
        {
            auto &members = pending.item.members;
            std::sort(members.begin(), members.end(),
                      [](const ItemMember &a, const ItemMember &b)
                      {
                          return a.byte_offset < b.byte_offset;
                      });
            if (pending.is_struct)
            {
                for (const auto &member : members)
                {
                    pending.item.byte_size =
                        std::max(pending.item.byte_size,
                                 member.byte_offset + plc_type_byte_size(member.plc_type));
                }
            }
            else
            {
                pending.item.byte_size =
                    pending.n_elements * plc_type_byte_size(pending.array_type);
            }
            items.push_back(std::move(pending.item));
        }
        return items;
    }

    double next_counter_value(double previous, PLCType plc_type)
    {
        const uint64_t modulus = uint64_t{1} << (8 * plc_type_byte_size(plc_type));
        const uint64_t current = std::isnan(previous) ? 0 : static_cast<uint64_t>(previous);
        return static_cast<double>((current + 1) % modulus);
    }

    bool rising_edge(double previous, double command)
    {
        const bool was_high = !std::isnan(previous) && previous > 0.5;
        const bool is_high = !std::isnan(command) && command > 0.5;
        return is_high && !was_high;
    }

    namespace
    {
        template <typename T> void store(T plc_value, uint8_t *destination)
        {
            std::memcpy(destination, &plc_value, sizeof(T));
        }

        template <typename T> double load(const uint8_t *source)
        {
            T plc_value;
            std::memcpy(&plc_value, source, sizeof(T));
            return static_cast<double>(plc_value);
        }
    }  // namespace

    bool encode_plc_value(PLCType plc_type, double value, uint8_t *destination)
    {
        switch (plc_type)
        {
        case PLCType::LREAL:
            store<double>(value, destination);
            return true;
        case PLCType::REAL:
            store<float>(static_cast<float>(value), destination);
            return true;
        case PLCType::BOOL:
            // bool is size of byte in PLC
            store<uint8_t>((value != 0.0) ? 1 : 0, destination);
            return true;
        case PLCType::SINT:
            store<int8_t>(static_cast<int8_t>(std::round(value)), destination);
            return true;
        case PLCType::USINT:
        case PLCType::BYTE:
            store<uint8_t>(static_cast<uint8_t>(std::round(value)), destination);
            return true;
        case PLCType::INT:
            store<int16_t>(static_cast<int16_t>(std::round(value)), destination);
            return true;
        case PLCType::UINT:
            store<uint16_t>(static_cast<uint16_t>(std::round(value)), destination);
            return true;
        case PLCType::DINT:
            store<int32_t>(static_cast<int32_t>(std::round(value)), destination);
            return true;
        case PLCType::UDINT:
            store<uint32_t>(static_cast<uint32_t>(std::round(value)), destination);
            return true;
        /* String not supported for now
        case PLCType::STRING: break;
        */
        case PLCType::UNKNOWN:
        default:
            return false;
        }
    }

    bool decode_plc_value(PLCType plc_type, const uint8_t *source, double &value)
    {
        switch (plc_type)
        {
        case PLCType::LREAL:
            value = load<double>(source);
            return true;
        case PLCType::REAL:
            value = load<float>(source);
            return true;
        case PLCType::BOOL:
            value = (load<uint8_t>(source) != 0.0) ? 1.0 : 0.0;
            return true;
        case PLCType::SINT:
            value = load<int8_t>(source);
            return true;
        case PLCType::USINT:
        case PLCType::BYTE:
            value = load<uint8_t>(source);
            return true;
        case PLCType::INT:
            value = load<int16_t>(source);
            return true;
        case PLCType::UINT:
            value = load<uint16_t>(source);
            return true;
        case PLCType::DINT:
            value = load<int32_t>(source);
            return true;
        case PLCType::UDINT:
            value = load<uint32_t>(source);
            return true;
        /* Not supported for now, guarded against in on_configure()
        case PLCType::STRING:
            break;
        */
        case PLCType::UNKNOWN:
        default:
            return false;
        }
    }

}  // namespace beckhoff_ads_hardware_interface
