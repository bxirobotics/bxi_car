#include "motor/yiyou/communication.h"

#include <array>
#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <ifaddrs.h>
#include <iostream>
#include <net/if.h>
#include <netinet/in.h>
#include <stdexcept>
#include <string>
#include <sys/socket.h>

namespace
{
char interface_name[] = "testec0";
struct InterfaceFixture
{
  bool present{true};
  bool up{true};
  bool enumeration_fails{false};
  bool ipv4{false};
  bool ipv6{false};
  int open_attempts{0};
  int freed_lists{0};
  std::array<ifaddrs, 3> records{};
  sockaddr_in v4{};
  sockaddr_in6 v6{};
} fixture;

void require(bool condition, const char * message)
{
  if (!condition) {throw std::runtime_error(message);}
}

std::string open_error()
{
  try {
    chassis::ethercat::Master master({interface_name, {1, 2, 3}});
  } catch (const std::exception & error) {
    return error.what();
  }
  throw std::runtime_error("test must never open a real EtherCAT master");
}
}  // namespace

extern "C" unsigned int __wrap_if_nametoindex(const char * name)
{
  return fixture.present && std::strcmp(name, interface_name) == 0 ? 1 : 0;
}

extern "C" int __wrap_getifaddrs(ifaddrs ** interfaces)
{
  *interfaces = nullptr;
  if (fixture.enumeration_fails) {errno = EIO; return -1;}
  // Include the link record with no IP, as well as any configured IP records.
  std::size_t count = 1;
  if (fixture.ipv4) {
    fixture.v4.sin_family = AF_INET;
    fixture.v4.sin_addr.s_addr = htonl(0xc0000201);  // 192.0.2.1, test data only.
    fixture.records[count++].ifa_addr = reinterpret_cast<sockaddr *>(&fixture.v4);
  }
  if (fixture.ipv6) {
    fixture.v6.sin6_family = AF_INET6;
    fixture.v6.sin6_addr.s6_addr[0] = 0x20;  // 2001:db8::1, not link-local.
    fixture.v6.sin6_addr.s6_addr[1] = 0x01;
    fixture.v6.sin6_addr.s6_addr[2] = 0x0d;
    fixture.v6.sin6_addr.s6_addr[3] = 0xb8;
    fixture.v6.sin6_addr.s6_addr[15] = 1;
    fixture.records[count++].ifa_addr = reinterpret_cast<sockaddr *>(&fixture.v6);
  }
  for (std::size_t i = 0; i < count; ++i) {
    fixture.records[i].ifa_name = interface_name;
    fixture.records[i].ifa_flags = fixture.up ? IFF_UP : 0;
    fixture.records[i].ifa_next = i + 1 < count ? &fixture.records[i + 1] : nullptr;
  }
  *interfaces = fixture.records.data();
  return 0;
}

extern "C" void __wrap_freeifaddrs(ifaddrs * interfaces)
{
  if (interfaces != fixture.records.data()) {std::abort();}
  ++fixture.freed_lists;
}

extern "C" int __wrap_ec_init(const char * name)
{
  if (std::strcmp(name, interface_name) != 0) {std::abort();}
  ++fixture.open_attempts;
  return 0;  // Stop after validation: no sockets, slave discovery, or cyclic I/O.
}

extern "C" int __wrap_socket(int, int, int)
{
  std::cerr << "FAIL: offline interface test attempted a real socket\n";
  std::abort();
}

int main()
{
  try {
    for (const bool ipv4 : {false, true}) {
      for (const bool ipv6 : {false, true}) {
        fixture = {};
        fixture.ipv4 = ipv4;
        fixture.ipv6 = ipv6;
        const auto error = open_error();
        require(fixture.open_attempts == 1,
          "UP interfaces must reach transport initialization with or without IPv4/IPv6");
        require(error.find("cannot open EtherCAT raw socket") != std::string::npos,
          "validation must finish before the fake transport rejects initialization");
        require(fixture.freed_lists == 1, "interface records must be released");
      }
    }
    fixture = {};
    fixture.present = false;
    require(open_error().find("does not exist") != std::string::npos,
      "missing interfaces must be rejected");
    require(fixture.open_attempts == 0, "missing interface must not open the transport");

    fixture = {};
    fixture.up = false;
    fixture.ipv4 = fixture.ipv6 = true;
    require(open_error().find("interface is not UP") != std::string::npos,
      "DOWN interfaces must still be rejected even when they have IP addresses");
    require(fixture.open_attempts == 0 && fixture.freed_lists == 1,
      "DOWN interface must release records without opening the transport");

    fixture = {};
    fixture.enumeration_fails = true;
    require(open_error().find("cannot inspect network interfaces") != std::string::npos,
      "interface enumeration failures must remain fatal");
    require(fixture.open_attempts == 0, "failed enumeration must not open the transport");
    std::cout << "PASS: shared IPv4/IPv6 interfaces accepted; missing/DOWN/errors rejected offline\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
