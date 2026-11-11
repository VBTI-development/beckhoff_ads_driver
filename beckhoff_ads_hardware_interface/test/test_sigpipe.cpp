// Copyright (c) 2025, b-robotized
// All rights reserved.
//
// Proprietary License
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential.

#include <gtest/gtest.h>

#include <csignal>
#include <tuple>

#include "beckhoff_ads_hardware_interface/beckhoff_ads_hardware_interface.hpp"

// 2026-11-11: the PLC connection was reset, the ADS library sent on the dead socket and SIGPIPE
// killed ros2_control_node (exit code -13) 49 ms after an ADS read timeout, with every other
// hardware component and controller in it. The library is a vendor binary whose sends cannot be
// given MSG_NOSIGNAL, so the component ignores the signal for the process it is loaded into.
TEST(Sigpipe, InitialisingTheComponentIgnoresSigpipe)
{
    std::signal(SIGPIPE, SIG_DFL);

    beckhoff_ads_hardware_interface::BeckhoffADSHardwareInterface hardware;
    // Whether an empty description initialises is beside the point: the signal is ignored first.
    std::ignore = hardware.on_init(hardware_interface::HardwareComponentInterfaceParams{});

    const auto disposition = std::signal(SIGPIPE, SIG_DFL);
    EXPECT_EQ(SIG_IGN, disposition) << "a send to a PLC that went away would kill the process";
}
