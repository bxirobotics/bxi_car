// Link this file directly into the dedicated lifecycle test executable.
// It satisfies every BXI vendor reference used by board.cpp; no vendor archive
// object, CAN device, power switch or vendor polling thread is used.
#include "bxi_pci_drv.h"

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <mutex>
#include <tuple>

namespace
{
constexpr char identity[] = "CAR2_SHARED_DAEMON_FAKE_VENDOR_V1";

struct TestOnlyIdentity
{
  TestOnlyIdentity()
  {
    const char * enabled = std::getenv("CAR2_FAKE_VENDOR_TEST");
    if (!enabled || std::strcmp(enabled, "1") != 0) {
      std::fputs("FAKE_BXI_REFUSED: run only through test_shared_daemon_recycle.py\n", stderr);
      std::_Exit(90);
    }
    std::printf("FAKE_BXI_STUB_READY %s\n", identity);
    std::fflush(stdout);
  }
} test_only_identity;

struct ObjectValue
{
  std::array<std::uint8_t, 4> bytes{};
  unsigned size{4};
};

using ObjectKey = std::tuple<unsigned, unsigned, std::uint16_t, std::uint8_t>;
std::mutex vendor_mutex;
canfd_rx_call receiver = nullptr;
void * receiver_context = nullptr;
unsigned motor_power = 0;
std::map<ObjectKey, ObjectValue> objects;

unsigned default_size(std::uint16_t index)
{
  if (index == 0x6040 || index == 0x6041) {return 2;}
  if (index == 0x6060 || index == 0x6061) {return 1;}
  return 4;
}
}  // namespace

extern "C" int bxi_pci_init(canfd_rx_call callback, void * context, int)
{
  std::lock_guard<std::mutex> lock(vendor_mutex);
  receiver = callback;
  receiver_context = context;
  std::puts("FAKE_BXI_INIT");
  std::fflush(stdout);
  return 0;
}

extern "C" int bxi_pci_exit()
{
  std::lock_guard<std::mutex> lock(vendor_mutex);
  receiver = nullptr;
  receiver_context = nullptr;
  std::puts("FAKE_BXI_EXIT");
  std::fflush(stdout);
  return 0;
}

extern "C" int motor_pwr_set(unsigned int requested)
{
  std::lock_guard<std::mutex> lock(vendor_mutex);
  const char * fail_off = std::getenv("FAKE_BXI_FAIL_POWER_OFF");
  if (requested == 0 && motor_power != 0 && fail_off && std::strcmp(fail_off, "1") == 0) {
    // A successful write is insufficient: board.cpp must verify actual power.
    std::puts("FAKE_BXI_POWER_OFF_FAILED");
  } else {
    motor_power = requested;
    std::puts(requested ? "FAKE_BXI_POWER_ON" : "FAKE_BXI_POWER_OFF");
  }
  std::fflush(stdout);
  return 0;
}

extern "C" int motor_pwr_get()
{
  std::lock_guard<std::mutex> lock(vendor_mutex);
  std::printf("FAKE_BXI_POWER_GET %u\n", motor_power);
  std::fflush(stdout);
  return static_cast<int>(motor_power);
}

extern "C" int canfd_send_packet(canfd_packet * packets, unsigned int count)
{
  if (!packets && count) {return -1;}
  for (unsigned i = 0; i < count; ++i) {
    const auto & request = packets[i];
    const auto id = request.frame.can_id;
    if (id < 0x601 || id > 0x67f || request.frame.len < 4) {continue;}
    const auto command = request.frame.data[0];
    const bool upload = command == 0x40;
    const bool download = (command & 0xe0) == 0x20 && (command & 0x02) != 0;
    if (!upload && !download) {continue;}
    const auto index = static_cast<std::uint16_t>(
      request.frame.data[1] | (std::uint16_t(request.frame.data[2]) << 8));
    const auto subindex = request.frame.data[3];
    canfd_packet response{};
    response.bus = request.bus;
    response.frame.can_id = id - 0x80;  // 0x600 + node -> 0x580 + node.
    response.frame.len = 8;
    std::copy_n(request.frame.data + 1, 3, response.frame.data + 1);
    canfd_rx_call callback;
    void * context;
    {
      std::lock_guard<std::mutex> lock(vendor_mutex);
      const ObjectKey key{request.bus, id - 0x600, index, subindex};
      if (download) {
        if (request.frame.len < 8) {return -1;}
        ObjectValue value;
        value.size = (command & 1) ? 4 - ((command >> 2) & 3) : 4;
        std::copy_n(request.frame.data + 4, value.size, value.bytes.begin());
        objects[key] = value;
        response.frame.data[0] = 0x60;
      } else {
        ObjectValue value;
        value.size = default_size(index);
        const auto found = objects.find(key);
        if (found != objects.end() && index != 0x6041) {value = found->second;}
        // Status defaults to zero; this stub does not emulate calibration or
        // motion. The integration test cancels during power-settle instead.
        response.frame.data[0] = static_cast<std::uint8_t>(0x43 | ((4 - value.size) << 2));
        std::copy_n(value.bytes.begin(), value.size, response.frame.data + 4);
      }
      callback = receiver;
      context = receiver_context;
    }
    // The transport queues the reply. Do not hold the stub mutex across it.
    if (callback) {(void)callback(context, &response);}
  }
  return 0;
}
