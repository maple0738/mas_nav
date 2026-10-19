#include "nav_serial_bridge/protocol.hpp"

#include <array>
#include <gtest/gtest.h>

TEST(NavSerialProtocol, EncodesStopWithNoFollowRequest)
{
  const std::array<uint8_t, 16> expected = {
    0xb5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x70, 0x2e};
  EXPECT_EQ(nav_serial_bridge::make_command(0.0f, 0.0f, 0.0f, false), expected);
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

TEST(NavSerialProtocol, EncodesTranslationAndSignedDegreeYawRefInLittleEndian)
{
  const std::array<uint8_t, 16> positive = {
    0xb5, 0x00, 0x00, 0x80, 0x3f, 0x00, 0x00, 0x00, 0xbf, 0x00, 0x00, 0xf0, 0x41, 0x01, 0xa8, 0x52};
  const std::array<uint8_t, 16> negative = {
    0xb5, 0x00, 0x00, 0x80, 0x3f, 0x00, 0x00, 0x00, 0xbf, 0x00, 0x00, 0xf0, 0xc1, 0x01, 0x64, 0xde};
  EXPECT_EQ(nav_serial_bridge::make_command(1.0f, -0.5f, 30.0f, true), positive);
  const auto frame = nav_serial_bridge::make_command(1.0f, -0.5f, -30.0f, true);
  EXPECT_EQ(frame, negative);
  EXPECT_TRUE(nav_serial_bridge::has_valid_crc(frame.data(), frame.size()));
  auto corrupted = frame;
  corrupted[13] = 0;
  EXPECT_FALSE(nav_serial_bridge::has_valid_crc(corrupted.data(), corrupted.size()));
}

TEST(NavSerialProtocol, NotFollowingClearsReference)
{
  EXPECT_EQ(nav_serial_bridge::make_command(0.0f, 0.0f, 1.0f, false),
    nav_serial_bridge::make_command(0.0f, 0.0f, 0.0f, false));
}
