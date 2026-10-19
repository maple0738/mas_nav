#include "nav_serial_bridge/protocol.hpp"
#include "nav_serial_bridge/interface_port.hpp"

#include <interfaces/msg/chassis_command.hpp>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>

#include <cerrno>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

namespace {
constexpr auto kSendPeriod = std::chrono::milliseconds(20);
constexpr auto kCommandTimeout = std::chrono::milliseconds(300);
constexpr auto kReconnectPeriod = std::chrono::seconds(1);
constexpr auto kDiagnosticsPeriod = std::chrono::seconds(5);
}  // namespace

class NavSerialBridge : public rclcpp::Node
{
public:
  NavSerialBridge() : Node("nav_serial_bridge")
  {
    device_ = declare_parameter<std::string>("device", "");
    command_sub_ = create_subscription<interfaces::msg::ChassisCommand>(
      "/nav_executor/chassis_cmd", rclcpp::QoS(1),
      [this](interfaces::msg::ChassisCommand::ConstSharedPtr msg) {
        if (!std::isfinite(msg->vx) || !std::isfinite(msg->vy)) {
          vx_ = 0.0f;
          vy_ = 0.0f;
          has_command_ = false;
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                               "Non-finite chassis command; zeroing translation");
          return;
        }
        vx_ = msg->vx;
        vy_ = msg->vy;
        follow_gimbal_ = msg->follow_gimbal && std::isfinite(msg->small_yaw_ref);
        small_yaw_ref_ = follow_gimbal_ ?
          std::remainder(msg->small_yaw_ref, 360.0f) : 0.0f;
        if (msg->follow_gimbal && !follow_gimbal_) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                               "Non-finite small-yaw reference; clearing follow_gimbal request");
        }
        last_command_ = std::chrono::steady_clock::now();
        has_command_ = true;
      });
    timer_ = create_wall_timer(kSendPeriod, [this]() { tick(); });
    RCLCPP_INFO(get_logger(), "Direct navigation bridge: %s, 115200 8N1, 50 Hz",
                device_.empty() ? "USB iInterface=PCCOMM-NAV" : device_.c_str());
  }

  ~NavSerialBridge() override
  {
    if (fd_ >= 0) {
      const auto stop = nav_serial_bridge::make_command(0.0f, 0.0f, 0.0f, false);
      const ssize_t written = ::write(fd_, stop.data(), stop.size());
      if (written != static_cast<ssize_t>(stop.size())) {
        RCLCPP_WARN(get_logger(), "Could not send final zero-translation frame");
      }
      ::close(fd_);
    }
  }

private:
  void disconnect()
  {
    if (fd_ >= 0) ::close(fd_);
    fd_ = -1;
    has_command_ = false;
    rx_size_ = 0;
    last_tx_time_ = {};
  }

  bool connect()
  {
    std::string port = device_;
    if (port.empty()) {
      const auto matches = nav_serial_bridge::find_interface_ports("PCCOMM-NAV");
      if (matches.size() != 1) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 3000,
                              "Expected one USB interface PCCOMM-NAV, found %zu", matches.size());
        return false;
      }
      port = matches.front();
    }
    const int fd = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 3000,
                            "Cannot open %s: %s", port.c_str(), std::strerror(errno));
      return false;
    }
    if (::ioctl(fd, TIOCEXCL) < 0) {
      RCLCPP_ERROR(get_logger(), "Cannot reserve navigation serial port: %s", std::strerror(errno));
      ::close(fd);
      return false;
    }
    termios settings{};
    if (::tcgetattr(fd, &settings) < 0) {
      RCLCPP_ERROR(get_logger(), "Cannot read serial settings: %s", std::strerror(errno));
      ::close(fd);
      return false;
    }
    ::cfmakeraw(&settings);
    settings.c_cflag |= CLOCAL | CREAD;
    settings.c_cflag &= ~CRTSCTS;
    settings.c_cc[VMIN] = 0;
    settings.c_cc[VTIME] = 0;
    ::cfsetispeed(&settings, B115200);
    ::cfsetospeed(&settings, B115200);
    if (::tcsetattr(fd, TCSANOW, &settings) < 0) {
      RCLCPP_ERROR(get_logger(), "Cannot configure serial port: %s", std::strerror(errno));
      ::close(fd);
      return false;
    }
    fd_ = fd;
    has_command_ = false;  // Reconnection needs a new ROS command before motion.
    rx_size_ = 0;
    last_tx_time_ = {};
    RCLCPP_INFO(get_logger(), "Navigation CDC1 connected: %s", port.c_str());
    return true;
  }

  void accept_rx_byte(uint8_t byte)
  {
    if (rx_size_ == 0 && byte != 0x5b) {
      ++discarded_rx_bytes_;
      return;
    }
    rx_[rx_size_++] = byte;
    if (rx_size_ < rx_.size()) return;

    if (nav_serial_bridge::has_valid_crc(rx_.data(), rx_.size())) {
      ++valid_rx_frames_;
      rx_size_ = 0;
      return;
    }
    ++invalid_rx_frames_;
    const auto next = std::find(rx_.begin() + 1, rx_.end(), 0x5b);
    discarded_rx_bytes_ += static_cast<uint64_t>(next - rx_.begin());
    rx_size_ = static_cast<size_t>(rx_.end() - next);
    std::move(next, rx_.end(), rx_.begin());
  }

  bool drain_rx()
  {
    uint8_t bytes[256];
    for (;;) {
      const ssize_t count = ::read(fd_, bytes, sizeof(bytes));
      if (count > 0) {
        rx_bytes_ += static_cast<uint64_t>(count);
        for (ssize_t i = 0; i < count; ++i) accept_rx_byte(bytes[i]);
        continue;
      }
      if (count == 0 || errno == EAGAIN || errno == EWOULDBLOCK) return true;
      if (errno == EINTR) continue;
      RCLCPP_ERROR(get_logger(), "Navigation CDC1 read failed: %s", std::strerror(errno));
      return false;
    }
  }

  void tick()
  {
    const auto now = std::chrono::steady_clock::now();
    if (fd_ < 0) {
      if (now - last_connect_attempt_ < kReconnectPeriod) return;
      last_connect_attempt_ = now;
      if (!connect()) return;
    }
    if (!drain_rx()) {
      disconnect();
      return;
    }

    const bool fresh = has_command_ && now - last_command_ <= kCommandTimeout;
    const float vx = fresh ? vx_ : 0.0f;
    const float vy = fresh ? vy_ : 0.0f;
    const bool follow = fresh && follow_gimbal_;
    const float ref = fresh ? small_yaw_ref_ : 0.0f;
    const auto frame = nav_serial_bridge::make_command(vx, vy, ref, follow);
    const ssize_t written = ::write(fd_, frame.data(), frame.size());
    if (written != static_cast<ssize_t>(frame.size())) {
      ++failed_tx_frames_;
      RCLCPP_ERROR(get_logger(), "Navigation CDC1 short/failed write: %zd/%zu (%s)",
                   written, frame.size(), written < 0 ? std::strerror(errno) : "short write");
      disconnect();
      return;
    }
    ++successful_tx_frames_;
    if (last_tx_time_ != std::chrono::steady_clock::time_point{}) {
      const double gap_ms = std::chrono::duration<double, std::milli>(now - last_tx_time_).count();
      max_tx_gap_ms_ = std::max(max_tx_gap_ms_, gap_ms);
    }
    last_tx_time_ = now;
    if (has_command_ && !fresh) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                           "Chassis command timed out; zeroing translation");
    }
    if (now - last_diag_time_ >= kDiagnosticsPeriod) {
      const double seconds = std::chrono::duration<double>(now - last_diag_time_).count();
      RCLCPP_INFO(get_logger(),
                  "CDC1 %.2fs: tx_ok=%llu (%.2f Hz), tx_failed=%llu, max_tx_gap=%.2f ms; "
                  "rx_valid=%llu (%.2f Hz), rx_crc_bad=%llu, rx_bytes=%llu, rx_discarded=%llu",
                  seconds,
                  static_cast<unsigned long long>(successful_tx_frames_ - reported_tx_frames_),
                  (successful_tx_frames_ - reported_tx_frames_) / seconds,
                  static_cast<unsigned long long>(failed_tx_frames_ - reported_failed_tx_frames_),
                  max_tx_gap_ms_,
                  static_cast<unsigned long long>(valid_rx_frames_ - reported_rx_frames_),
                  (valid_rx_frames_ - reported_rx_frames_) / seconds,
                  static_cast<unsigned long long>(invalid_rx_frames_ - reported_invalid_rx_frames_),
                  static_cast<unsigned long long>(rx_bytes_ - reported_rx_bytes_),
                  static_cast<unsigned long long>(discarded_rx_bytes_ - reported_discarded_rx_bytes_));
      reported_tx_frames_ = successful_tx_frames_;
      reported_failed_tx_frames_ = failed_tx_frames_;
      reported_rx_frames_ = valid_rx_frames_;
      reported_invalid_rx_frames_ = invalid_rx_frames_;
      reported_rx_bytes_ = rx_bytes_;
      reported_discarded_rx_bytes_ = discarded_rx_bytes_;
      max_tx_gap_ms_ = 0.0;
      last_diag_time_ = now;
    }
  }

  std::string device_;
  int fd_{-1};
  float vx_{0.0f};
  float vy_{0.0f};
  float small_yaw_ref_{0.0f};
  bool follow_gimbal_{false};
  bool has_command_{false};
  std::chrono::steady_clock::time_point last_command_{};
  std::chrono::steady_clock::time_point last_connect_attempt_{};
  std::chrono::steady_clock::time_point last_diag_time_{std::chrono::steady_clock::now()};
  std::chrono::steady_clock::time_point last_tx_time_{};
  double max_tx_gap_ms_{0.0};
  std::array<uint8_t, nav_serial_bridge::kRxFrameSize> rx_{};
  size_t rx_size_{0};
  uint64_t successful_tx_frames_{0};
  uint64_t failed_tx_frames_{0};
  uint64_t rx_bytes_{0};
  uint64_t valid_rx_frames_{0};
  uint64_t invalid_rx_frames_{0};
  uint64_t discarded_rx_bytes_{0};
  uint64_t reported_tx_frames_{0};
  uint64_t reported_failed_tx_frames_{0};
  uint64_t reported_rx_frames_{0};
  uint64_t reported_invalid_rx_frames_{0};
  uint64_t reported_rx_bytes_{0};
  uint64_t reported_discarded_rx_bytes_{0};
  rclcpp::Subscription<interfaces::msg::ChassisCommand>::SharedPtr command_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NavSerialBridge>());
  rclcpp::shutdown();
  return 0;
}
