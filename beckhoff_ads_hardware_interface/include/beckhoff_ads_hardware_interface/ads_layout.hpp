// Copyright (c) 2025, b-robotized
// All rights reserved.
//
// Proprietary License
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential.

#ifndef BECKHOFF_ADS_HARDWARE_INTERFACE__ADS_LAYOUT_HPP_
#define BECKHOFF_ADS_HARDWARE_INTERFACE__ADS_LAYOUT_HPP_

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

// How ros2_control interfaces map onto ADS sum read/write items. Free of ADS and rclcpp so it can
// be unit tested without a PLC.
namespace beckhoff_ads_hardware_interface
{

    enum class PLCType
    {
        // Using enum instead of raw strings for faster iterations in read/write
        UNKNOWN,
        BOOL,
        LREAL,
        REAL,
        UDINT,
        DINT,
        INT,
        UINT,
        SINT,
        USINT,
        BYTE,
        STRING,
    };

    /// UNKNOWN for a name that is not a PLC type. Case-insensitive.
    PLCType plc_type_from_string(const std::string &type_str);

    /// Bytes of ONE value of this type on the PLC; 0 for UNKNOWN and STRING (not supported).
    size_t plc_type_byte_size(PLCType plc_type);

    /// One interface as the URDF declares it.
    struct InterfaceSpec
    {
        std::string name;        // ros2_control interface name, e.g. "turntable/feed_position"
        std::string plc_symbol;  // e.g. "MAIN.FeedIf"
        PLCType plc_type = PLCType::UNKNOWN;
        size_t n_elements = 1;              // array length; arrays only
        std::optional<size_t> index;        // array element; defaults to 0
        std::optional<size_t> byte_offset;  // struct member: offset inside the symbol, in bytes
        bool auto_increment = false;        // command only: the component counts it up every write
    };

    /// Where one interface's value sits inside its item.
    struct ItemMember
    {
        std::string interface_name;
        size_t byte_offset = 0;
        PLCType plc_type = PLCType::UNKNOWN;
        bool auto_increment = false;
    };

    /// One ADS item: one PLC symbol, read or written as one block in the sum command, so a PLC
    /// cycle never sees some of its members updated and others not.
    struct ItemLayoutSpec
    {
        std::string plc_symbol;
        size_t byte_size = 0;
        std::vector<ItemMember> members;  // ordered by byte_offset
    };

    /// Groups interfaces into one item per PLC symbol, in the order the symbols are first seen.
    ///
    /// A symbol is either an ARRAY (members by `index`, all of the array's type, item size
    /// n_elements x type size) or a STRUCT (members by `byte_offset`, each of its own type, item
    /// size up to the end of the last member; bytes between members are written as 0). An interface
    /// that does not fit -- unsupported type, mixed index/byte_offset, overlap, duplicate or
    /// out-of-range index, auto_increment on a type that cannot count -- is left out and described
    /// in `errors`.
    std::vector<ItemLayoutSpec> group_items(const std::vector<InterfaceSpec> &specs,
                                            std::vector<std::string> &errors);

    /// The value an auto_increment member carries after `previous`: one more, wrapping to 0 past
    /// the type's maximum. NaN (nothing sent yet) counts as 0, so the first value written is 1.
    double next_counter_value(double previous, PLCType plc_type);

    /// Writes `value` into `destination` as the PLC type (integers rounded, BOOL is value != 0).
    /// False for a type that is not supported.
    bool encode_plc_value(PLCType plc_type, double value, uint8_t *destination);

    /// Reads a PLC value of this type from `source` into `value`. False for an unsupported type.
    bool decode_plc_value(PLCType plc_type, const uint8_t *source, double &value);

}  // namespace beckhoff_ads_hardware_interface

#endif  // BECKHOFF_ADS_HARDWARE_INTERFACE__ADS_LAYOUT_HPP_
