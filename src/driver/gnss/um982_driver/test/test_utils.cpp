#include <array>
#include <cstdint>

#include <gtest/gtest.h>

#include "um982_driver/utils.hpp"

namespace
{

// UNIHEADINGB message ID 972 captured from the actual UM982.  The frame
// layout and CRC cover Header (24) + Data (44), followed by little-endian
// CRC32, as specified by the N4 Reference Commands Manual R1.4.
constexpr std::array<uint8_t, 72> kCapturedUniheadingFrame = {
    0xaa, 0x44, 0xb5, 0x62, 0xcc, 0x03, 0x2c, 0x00,
    0x00, 0xc9, 0x01, 0x00, 0x50, 0x69, 0x0f, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x12, 0x02, 0x00,
    0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x40, 0xf8, 0x22, 0x10,
};

TEST(UnicoreBinaryProtocol, CapturedUniheadingFrameHasValidCrc)
{
    constexpr std::size_t kCrcSize = 4;
    const auto calculated = um982_driver::utils::calculate_unicore_crc32(
        kCapturedUniheadingFrame.data(), kCapturedUniheadingFrame.size() - kCrcSize);
    const auto received = um982_driver::utils::read_u32_le(
        kCapturedUniheadingFrame.data() + kCapturedUniheadingFrame.size() - kCrcSize);

    EXPECT_EQ(um982_driver::utils::read_u16_le(kCapturedUniheadingFrame.data() + 4), 972U);
    EXPECT_EQ(um982_driver::utils::read_u16_le(kCapturedUniheadingFrame.data() + 6), 44U);
    EXPECT_EQ(calculated, 0x1022f840U);
    EXPECT_EQ(calculated, received);
}

}  // namespace
