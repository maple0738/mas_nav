#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>

namespace nav_serial_bridge {

constexpr size_t kRxFrameSize = 11;  // 0x5B + NavSendPacket(8) + CRC16
constexpr size_t kTxFrameSize = 16;  // 0xB5 + vx/vy/w/nav_state(13) + CRC16
constexpr float kFixedW = 4.0f;

// crc_rm: CRC16 init 0xFFFF, reflected polynomial 0x8408, little-endian result.
inline uint16_t crc16(const uint8_t *data, size_t size)
{
  uint16_t crc = 0xffff;
  for (size_t i = 0; i < size; ++i) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc >> 1) ^ ((crc & 1) ? 0x8408 : 0);
    }
  }
  return crc;
}

inline bool has_valid_crc(const uint8_t *frame, size_t size)
{
  if (size < 3) return false;
  const uint16_t expected = crc16(frame, size - 2);
  return frame[size - 2] == static_cast<uint8_t>(expected) &&
         frame[size - 1] == static_cast<uint8_t>(expected >> 8);
}

inline std::array<uint8_t, kTxFrameSize> make_command(float vx, float vy)
{
  static_assert(sizeof(float) == 4 && std::numeric_limits<float>::is_iec559,
                "NavReceivePacket requires IEEE-754 binary32");
  std::array<uint8_t, kTxFrameSize> frame{};
  frame[0] = 0xb5;
  uint32_t x_bits;
  uint32_t y_bits;
  uint32_t w_bits;
  std::memcpy(&x_bits, &vx, sizeof(vx));
  std::memcpy(&y_bits, &vy, sizeof(vy));
  std::memcpy(&w_bits, &kFixedW, sizeof(kFixedW));
  for (unsigned i = 0; i < 4; ++i) {
    frame[1 + i] = static_cast<uint8_t>(x_bits >> (8 * i));
    frame[5 + i] = static_cast<uint8_t>(y_bits >> (8 * i));
    frame[9 + i] = static_cast<uint8_t>(w_bits >> (8 * i));
  }
  frame[13] = 1U;
  const uint16_t crc = crc16(frame.data(), frame.size() - 2);
  frame[14] = static_cast<uint8_t>(crc);
  frame[15] = static_cast<uint8_t>(crc >> 8);
  return frame;
}

}  // namespace nav_serial_bridge
