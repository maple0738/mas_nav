#pragma once

#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

namespace nav_serial_bridge {

// Read the USB interface's own string descriptor exposed by sysfs. The USB
// device's product string is deliberately never consulted.
inline std::vector<std::string> find_interface_ports(
  const std::string & interface_name,
  const std::filesystem::path & sys_tty = "/sys/class/tty",
  const std::filesystem::path & dev_root = "/dev")
{
  std::vector<std::string> ports;
  std::error_code ec;
  const std::filesystem::directory_iterator end;
  for (std::filesystem::directory_iterator it(sys_tty, ec); !ec && it != end; it.increment(ec)) {
    const auto name = it->path().filename().string();
    if (name.compare(0, 6, "ttyACM") != 0) continue;

    auto path = std::filesystem::canonical(it->path() / "device", ec);
    if (ec) {
      ec.clear();
      continue;
    }
    for (;;) {
      std::ifstream descriptor(path / "interface");
      std::string value;
      if (descriptor && std::getline(descriptor, value)) {
        if (value == interface_name) ports.push_back((dev_root / name).string());
        break;
      }
      const auto parent = path.parent_path();
      if (parent == path || parent.empty()) break;
      path = parent;
    }
  }
  return ports;
}

}  // namespace nav_serial_bridge
