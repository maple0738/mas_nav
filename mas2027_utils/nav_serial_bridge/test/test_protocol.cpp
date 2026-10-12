#include "nav_serial_bridge/protocol.hpp"

#include <array>
#include <gtest/gtest.h>

TEST(NavSerialProtocol, MatchesControllerZeroFrames)
{
  using nav_serial_bridge::make_command;
  const std::array<uint8_t, 12> valid_zero = {
    0xb5, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0xa4, 0xcd};
  const std::array<uint8_t, 12> invalid_zero = {
    0xb5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x2d, 0xdc};
  EXPECT_EQ(make_command(0.0f, 0.0f, 1), valid_zero);
  EXPECT_EQ(make_command(0.0f, 0.0f, 0), invalid_zero);
}

TEST(NavSerialProtocol, MatchesObservedControllerReply)
{
  const std::array<uint8_t, 11> reply = {
    0x5b, 0, 0, 0, 0, 0, 0, 0, 0, 0x22, 0xfe};
  EXPECT_TRUE(nav_serial_bridge::has_valid_crc(reply.data(), reply.size()));
  auto corrupted = reply;
  corrupted[2] = 1;
  EXPECT_FALSE(nav_serial_bridge::has_valid_crc(corrupted.data(), corrupted.size()));
}

TEST(NavSerialProtocol, EncodesBothVelocitiesInLittleEndian)
{
  const auto frame = nav_serial_bridge::make_command(1.0f, -0.5f, 1);
  EXPECT_EQ(frame[0], 0xb5);
  EXPECT_EQ((std::array<uint8_t, 4>{frame[1], frame[2], frame[3], frame[4]}),
            (std::array<uint8_t, 4>{0, 0, 0x80, 0x3f}));
  EXPECT_EQ((std::array<uint8_t, 4>{frame[5], frame[6], frame[7], frame[8]}),
            (std::array<uint8_t, 4>{0, 0, 0, 0xbf}));
  EXPECT_TRUE(nav_serial_bridge::has_valid_crc(frame.data(), frame.size()));
}
