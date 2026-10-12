#include "nav_serial_bridge/interface_port.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <string>
#include <unistd.h>

namespace fs = std::filesystem;

TEST(InterfacePort, MatchesInterfaceStringInsteadOfProductOrAcmNumber)
{
  const auto root = fs::temp_directory_path() /
    ("nav_interface_port_test_" + std::to_string(::getpid()));
  const auto tty = root / "sys/class/tty";
  const auto usb = root / "sys/devices/usb";
  fs::create_directories(tty);
  fs::create_directories(usb / "board:1.0");
  fs::create_directories(usb / "board:1.2");
  fs::create_directories(usb / "other:1.0");
  std::ofstream(usb / "board:1.0/interface") << "PCCOMM-AIM\n";
  std::ofstream(usb / "board:1.2/interface") << "PCCOMM-NAV\n";
  std::ofstream(usb / "other:1.0/interface") << "Other\n";
  std::ofstream(usb / "product") << "PCCOMM-NAV\n";
  for (const auto & [name, target] : {
         std::pair{"ttyACM8", "board:1.0"},
         std::pair{"ttyACM3", "board:1.2"},
         std::pair{"ttyACM0", "other:1.0"}}) {
    fs::create_directories(tty / name);
    fs::create_directory_symlink(usb / target, tty / name / "device");
  }

  EXPECT_EQ(nav_serial_bridge::find_interface_ports("PCCOMM-NAV", tty, root / "dev"),
            std::vector<std::string>{(root / "dev/ttyACM3").string()});
  EXPECT_TRUE(nav_serial_bridge::find_interface_ports("missing", tty, root / "dev").empty());

  std::ofstream(usb / "other:1.0/interface") << "PCCOMM-NAV\n";
  EXPECT_EQ(nav_serial_bridge::find_interface_ports("PCCOMM-NAV", tty, root / "dev").size(), 2u);
  fs::remove_all(root);
}
