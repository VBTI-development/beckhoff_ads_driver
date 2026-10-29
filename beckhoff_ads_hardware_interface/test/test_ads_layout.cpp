// Copyright (c) 2025, b-robotized
// All rights reserved.
//
// Proprietary License
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential.

#include <gtest/gtest.h>

#include <cstdint>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

#include "beckhoff_ads_hardware_interface/ads_layout.hpp"

using beckhoff_ads_hardware_interface::decode_plc_value;
using beckhoff_ads_hardware_interface::encode_plc_value;
using beckhoff_ads_hardware_interface::group_items;
using beckhoff_ads_hardware_interface::InterfaceSpec;
using beckhoff_ads_hardware_interface::ItemLayoutSpec;
using beckhoff_ads_hardware_interface::next_counter_value;
using beckhoff_ads_hardware_interface::PLCType;
using beckhoff_ads_hardware_interface::rising_edge;

namespace
{
    InterfaceSpec array_member(const std::string &name, const std::string &symbol, PLCType type,
                               size_t n_elements, size_t index)
    {
        InterfaceSpec spec;
        spec.name = name;
        spec.plc_symbol = symbol;
        spec.plc_type = type;
        spec.n_elements = n_elements;
        spec.index = index;
        return spec;
    }

    InterfaceSpec struct_member(const std::string &name, const std::string &symbol, PLCType type,
                                size_t byte_offset)
    {
        InterfaceSpec spec;
        spec.name = name;
        spec.plc_symbol = symbol;
        spec.plc_type = type;
        spec.byte_offset = byte_offset;
        return spec;
    }

    // Brom's ST_AxisStream, pack_mode 1: the cyclic setpoint the PLC must see whole.
    std::vector<InterfaceSpec> axis_stream()
    {
        return {
            struct_member("feed/counter", "MAIN.FeedIf", PLCType::UDINT, 0),
            struct_member("feed/position", "MAIN.FeedIf", PLCType::LREAL, 4),
            struct_member("feed/velocity", "MAIN.FeedIf", PLCType::LREAL, 12),
            struct_member("feed/acceleration", "MAIN.FeedIf", PLCType::LREAL, 20),
        };
    }
}  // namespace

TEST(GroupItems, AScalarIsOneItemOfItsOwnSize)
{
    InterfaceSpec spec;
    spec.name = "machine/stop";
    spec.plc_symbol = "Main.Machine.bStop";
    spec.plc_type = PLCType::BOOL;
    std::vector<std::string> errors;

    const auto items = group_items({spec}, errors);

    EXPECT_TRUE(errors.empty());
    ASSERT_EQ(items.size(), 1u);
    EXPECT_EQ(items[0].plc_symbol, "Main.Machine.bStop");
    EXPECT_EQ(items[0].byte_size, 1u);
    ASSERT_EQ(items[0].members.size(), 1u);
    EXPECT_EQ(items[0].members[0].interface_name, "machine/stop");
    EXPECT_EQ(items[0].members[0].byte_offset, 0u);
    EXPECT_EQ(items[0].members[0].plc_type, PLCType::BOOL);
}

TEST(GroupItems, ArrayElementsShareOneItemAtIndexTimesElementSize)
{
    std::vector<std::string> errors;

    const auto items = group_items({array_member("j1", "MAIN.Pos", PLCType::LREAL, 3, 0),
                                    array_member("j3", "MAIN.Pos", PLCType::LREAL, 3, 2),
                                    array_member("j2", "MAIN.Pos", PLCType::LREAL, 3, 1)},
                                   errors);

    EXPECT_TRUE(errors.empty());
    ASSERT_EQ(items.size(), 1u);
    EXPECT_EQ(items[0].byte_size, 24u);
    ASSERT_EQ(items[0].members.size(), 3u);
    EXPECT_EQ(items[0].members[0].interface_name, "j1");
    EXPECT_EQ(items[0].members[0].byte_offset, 0u);
    EXPECT_EQ(items[0].members[1].interface_name, "j2");
    EXPECT_EQ(items[0].members[1].byte_offset, 8u);
    EXPECT_EQ(items[0].members[2].interface_name, "j3");
    EXPECT_EQ(items[0].members[2].byte_offset, 16u);
}

TEST(GroupItems, SymbolsKeepTheOrderTheyWereFirstSeenIn)
{
    std::vector<std::string> errors;

    const auto items = group_items({array_member("b", "MAIN.B", PLCType::INT, 1, 0),
                                    array_member("a", "MAIN.A", PLCType::INT, 1, 0)},
                                   errors);

    ASSERT_EQ(items.size(), 2u);
    EXPECT_EQ(items[0].plc_symbol, "MAIN.B");
    EXPECT_EQ(items[1].plc_symbol, "MAIN.A");
}

TEST(GroupItems, StructMembersOfMixedTypesAreOneItemSizedToTheLastMemberEnd)
{
    std::vector<std::string> errors;

    const auto items = group_items(axis_stream(), errors);

    EXPECT_TRUE(errors.empty());
    ASSERT_EQ(items.size(), 1u);
    EXPECT_EQ(items[0].plc_symbol, "MAIN.FeedIf");
    EXPECT_EQ(items[0].byte_size, 28u);
    ASSERT_EQ(items[0].members.size(), 4u);
    EXPECT_EQ(items[0].members[0].plc_type, PLCType::UDINT);
    EXPECT_EQ(items[0].members[0].byte_offset, 0u);
    EXPECT_EQ(items[0].members[1].plc_type, PLCType::LREAL);
    EXPECT_EQ(items[0].members[1].byte_offset, 4u);
    EXPECT_EQ(items[0].members[3].interface_name, "feed/acceleration");
    EXPECT_EQ(items[0].members[3].byte_offset, 20u);
}

TEST(GroupItems, StructMembersAreOrderedByOffsetWhateverTheDeclarationOrder)
{
    auto specs = axis_stream();
    std::swap(specs[0], specs[3]);
    std::vector<std::string> errors;

    const auto items = group_items(specs, errors);

    ASSERT_EQ(items.size(), 1u);
    ASSERT_EQ(items[0].members.size(), 4u);
    EXPECT_EQ(items[0].members[0].interface_name, "feed/counter");
    EXPECT_EQ(items[0].members[3].interface_name, "feed/acceleration");
    EXPECT_EQ(items[0].byte_size, 28u);
}

TEST(GroupItems, OverlappingStructMembersAreRejected)
{
    auto specs = axis_stream();
    specs.push_back(
        struct_member("feed/bad", "MAIN.FeedIf", PLCType::LREAL, 8));  // straddles position
    std::vector<std::string> errors;

    const auto items = group_items(specs, errors);

    ASSERT_EQ(errors.size(), 1u);
    EXPECT_NE(errors[0].find("feed/bad"), std::string::npos);
    ASSERT_EQ(items.size(), 1u);
    EXPECT_EQ(items[0].members.size(), 4u);
    EXPECT_EQ(items[0].byte_size, 28u);
}

TEST(GroupItems, ByteOffsetAndIndexCannotBeMixedOnOneSymbol)
{
    std::vector<std::string> errors;

    const auto items = group_items({struct_member("a", "MAIN.X", PLCType::LREAL, 0),
                                    array_member("b", "MAIN.X", PLCType::LREAL, 2, 1)},
                                   errors);

    ASSERT_EQ(errors.size(), 1u);
    EXPECT_NE(errors[0].find("'b'"), std::string::npos);
    ASSERT_EQ(items.size(), 1u);
    EXPECT_EQ(items[0].members.size(), 1u);
}

TEST(GroupItems, TwoInterfacesOnTheSameArrayIndexAreRejected)
{
    std::vector<std::string> errors;

    const auto items = group_items({array_member("a", "MAIN.X", PLCType::LREAL, 2, 1),
                                    array_member("b", "MAIN.X", PLCType::LREAL, 2, 1)},
                                   errors);

    ASSERT_EQ(errors.size(), 1u);
    EXPECT_NE(errors[0].find("'b'"), std::string::npos);
    ASSERT_EQ(items.size(), 1u);
    EXPECT_EQ(items[0].members.size(), 1u);
}

TEST(GroupItems, AnArrayElementOfAnotherTypeIsRejected)
{
    std::vector<std::string> errors;

    const auto items = group_items({array_member("a", "MAIN.X", PLCType::LREAL, 2, 0),
                                    array_member("b", "MAIN.X", PLCType::REAL, 2, 1)},
                                   errors);

    ASSERT_EQ(errors.size(), 1u);
    ASSERT_EQ(items.size(), 1u);
    EXPECT_EQ(items[0].members.size(), 1u);
}

TEST(GroupItems, AnIndexOutsideTheArrayIsRejected)
{
    std::vector<std::string> errors;

    const auto items = group_items({array_member("a", "MAIN.X", PLCType::LREAL, 2, 2)}, errors);

    ASSERT_EQ(errors.size(), 1u);
    EXPECT_TRUE(items.empty());
}

TEST(GroupItems, UnsupportedTypesAreRejected)
{
    std::vector<std::string> errors;

    const auto items = group_items({struct_member("s", "MAIN.S", PLCType::STRING, 0),
                                    struct_member("u", "MAIN.U", PLCType::UNKNOWN, 0)},
                                   errors);

    EXPECT_EQ(errors.size(), 2u);
    EXPECT_TRUE(items.empty());
}

TEST(GroupItems, AutoIncrementIsKeptOnTheMember)
{
    auto specs = axis_stream();
    specs[0].auto_increment = true;
    std::vector<std::string> errors;

    const auto items = group_items(specs, errors);

    EXPECT_TRUE(errors.empty());
    ASSERT_EQ(items.size(), 1u);
    EXPECT_TRUE(items[0].members[0].auto_increment);
    EXPECT_FALSE(items[0].members[1].auto_increment);
}

TEST(GroupItems, AutoIncrementNeedsAnUnsignedIntegerType)
{
    auto specs = axis_stream();
    specs[1].auto_increment = true;  // LREAL position
    std::vector<std::string> errors;

    const auto items = group_items(specs, errors);

    ASSERT_EQ(errors.size(), 1u);
    EXPECT_NE(errors[0].find("feed/position"), std::string::npos);
    ASSERT_EQ(items.size(), 1u);
    EXPECT_EQ(items[0].members.size(), 3u);
}

TEST(NextCounterValue, CountsUpByOne)
{
    EXPECT_EQ(next_counter_value(0.0, PLCType::UDINT), 1.0);
    EXPECT_EQ(next_counter_value(41.0, PLCType::UINT), 42.0);
}

TEST(NextCounterValue, WrapsAtTheTypeMaximum)
{
    EXPECT_EQ(next_counter_value(4294967295.0, PLCType::UDINT), 0.0);
    EXPECT_EQ(next_counter_value(65535.0, PLCType::UINT), 0.0);
    EXPECT_EQ(next_counter_value(255.0, PLCType::USINT), 0.0);
    EXPECT_EQ(next_counter_value(255.0, PLCType::BYTE), 0.0);
}

TEST(NextCounterValue, StartsFromZeroWhenThePreviousValueIsNotANumber)
{
    EXPECT_EQ(next_counter_value(std::numeric_limits<double>::quiet_NaN(), PLCType::UDINT), 1.0);
}

TEST(PlcValueCodec, AnAxisStreamPacksToBromsTwentyEightBytes)
{
    std::vector<uint8_t> buffer(28, 0xAA);

    ASSERT_TRUE(encode_plc_value(PLCType::UDINT, 7.0, buffer.data() + 0));
    ASSERT_TRUE(encode_plc_value(PLCType::LREAL, 1.25, buffer.data() + 4));
    ASSERT_TRUE(encode_plc_value(PLCType::LREAL, -0.5, buffer.data() + 12));
    ASSERT_TRUE(encode_plc_value(PLCType::LREAL, 8.0, buffer.data() + 20));

    // What Python's struct.pack("<Iddd", 7, 1.25, -0.5, 8.0) sends, which the PLC accepted on
    // 2026-10-28.
    uint32_t counter;
    double position, velocity, acceleration;
    std::memcpy(&counter, buffer.data() + 0, 4);
    std::memcpy(&position, buffer.data() + 4, 8);
    std::memcpy(&velocity, buffer.data() + 12, 8);
    std::memcpy(&acceleration, buffer.data() + 20, 8);
    EXPECT_EQ(counter, 7u);
    EXPECT_EQ(position, 1.25);
    EXPECT_EQ(velocity, -0.5);
    EXPECT_EQ(acceleration, 8.0);
}

TEST(PlcValueCodec, EverySupportedTypeRoundTrips)
{
    const std::vector<std::pair<PLCType, double>> cases = {
        {PLCType::LREAL, -3.25},       {PLCType::REAL, 2.5},     {PLCType::BOOL, 1.0},
        {PLCType::SINT, -12.0},        {PLCType::USINT, 200.0},  {PLCType::BYTE, 17.0},
        {PLCType::INT, -300.0},        {PLCType::UINT, 60000.0}, {PLCType::DINT, -70000.0},
        {PLCType::UDINT, 4000000000.0}};
    for (const auto &[type, value] : cases)
    {
        uint8_t buffer[8] = {};
        double decoded = 0.0;
        ASSERT_TRUE(encode_plc_value(type, value, buffer)) << static_cast<int>(type);
        ASSERT_TRUE(decode_plc_value(type, buffer, decoded)) << static_cast<int>(type);
        EXPECT_EQ(decoded, value) << static_cast<int>(type);
    }
}

TEST(PlcValueCodec, UnsupportedTypesAreRefused)
{
    uint8_t buffer[8] = {};
    double decoded = 0.0;
    EXPECT_FALSE(encode_plc_value(PLCType::UNKNOWN, 1.0, buffer));
    EXPECT_FALSE(decode_plc_value(PLCType::STRING, buffer, decoded));
}

TEST(GroupItems, PulseOnRisingEdgeIsKeptOnTheMember)
{
    InterfaceSpec spec;
    spec.name = "machine/alive";
    spec.plc_symbol = "MAIN.Machine.Alive";
    spec.plc_type = PLCType::BOOL;
    spec.pulse_on_rising_edge = true;
    std::vector<std::string> errors;

    const auto items = group_items({spec}, errors);

    EXPECT_TRUE(errors.empty());
    ASSERT_EQ(items.size(), 1u);
    EXPECT_TRUE(items[0].members[0].pulse_on_rising_edge);
}

TEST(GroupItems, PulseOnRisingEdgeNeedsABool)
{
    InterfaceSpec spec;
    spec.name = "x";
    spec.plc_symbol = "MAIN.X";
    spec.plc_type = PLCType::UDINT;
    spec.pulse_on_rising_edge = true;
    std::vector<std::string> errors;

    const auto items = group_items({spec}, errors);

    ASSERT_EQ(errors.size(), 1u);
    EXPECT_NE(errors[0].find("pulse_on_rising_edge"), std::string::npos);
    EXPECT_TRUE(items.empty());
}

// A heartbeat a dead node cannot fake: the gpio controller keeps writing the last command it got,
// so a level written as-is would stay 1 forever after its sender died. Only a 0 -> 1 change of the
// command produces a 1 on the PLC, for one write.
TEST(RisingEdge, OnlyATransitionToOneIsAnEdge)
{
    EXPECT_TRUE(rising_edge(0.0, 1.0));
    EXPECT_FALSE(rising_edge(1.0, 1.0));
    EXPECT_FALSE(rising_edge(1.0, 0.0));
    EXPECT_FALSE(rising_edge(0.0, 0.0));
}

TEST(RisingEdge, NothingCommandedYetCountsAsZero)
{
    const double nan = std::numeric_limits<double>::quiet_NaN();
    EXPECT_TRUE(rising_edge(nan, 1.0));
    EXPECT_FALSE(rising_edge(0.0, nan));
    EXPECT_FALSE(rising_edge(nan, nan));
}
