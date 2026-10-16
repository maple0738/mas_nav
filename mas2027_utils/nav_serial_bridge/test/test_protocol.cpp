#include "nav_serial_bridge/protocol.hpp"

#include <array>
#include <gtest/gtest.h>

TEST(NavSerialProtocol, EncodesFixedYawWithoutTranslation)
{
  using nav_serial_bridge::make_command;
  const std::array<uint8_t, 16> expected = {
    0xb5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x80, 0x40, 1, 0x73, 0x75};
  EXPECT_EQ(make_command(0.0f, 0.0f), expected);
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

TEST(NavSerialProtocol, EncodesTranslationAndFixedYawInLittleEndian)
{
  const std::array<uint8_t, 16> expected = {
    0xb5, 0, 0, 0x80, 0x3f, 0, 0, 0, 0xbf,
    0, 0, 0x80, 0x40, 1, 0xa8, 0xcb};
  const auto frame = nav_serial_bridge::make_command(1.0f, -0.5f);
  EXPECT_EQ(frame, expected);
  EXPECT_TRUE(nav_serial_bridge::has_valid_crc(frame.data(), frame.size()));
}
