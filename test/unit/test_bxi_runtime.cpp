#include "bxi/board.h"
#include "bxi/can_timing.h"

#include <cerrno>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <sys/wait.h>
#include <unistd.h>
#include <vector>

namespace
{
struct VendorStub
{
  int init_result{0};
  int power_result{0};
  int power_read_result{0};
  int exit_result{0};
  unsigned init_calls{0};
  unsigned exit_calls{0};
  unsigned send_calls{0};
  unsigned power_read_calls{0};
  int cpu{0};
  canfd_rx_call receive{nullptr};
  void * context{nullptr};
  std::vector<unsigned> power;
  std::vector<std::string> events;
  canfd_packet last_sent{};
} vendor;

void require(bool condition, const char * message)
{
  if (!condition) {throw std::runtime_error(message);}
}

template<typename Exception, typename Function>
void rejects(Function function)
{
  try {function();}
  catch (const Exception &) {return;}
  throw std::runtime_error("expected transport construction to be rejected");
}

void deliver(unsigned bus)
{
  require(vendor.receive != nullptr, "vendor must have received the transport callback");
  canfd_packet packet{};
  packet.bus = bus;
  packet.frame.can_id = 0x123;
  packet.frame.len = 1;
  packet.frame.data[0] = 0x5a;
  require(vendor.receive(vendor.context, &packet) == 0, "valid late callback must remain safe");
}

void invalid_bus_does_not_consume_runtime()
{
  require(!chassis::BxiPciTransport::runtime_used(), "runtime must start unused");
  require(!chassis::BxiPciTransport::runtime_cleanup_failed(), "cleanup must initially be healthy");
  rejects<std::invalid_argument>([] {chassis::BxiPciTransport invalid(CANFD_DEVICE_NUM);});
  require(!chassis::BxiPciTransport::runtime_used() && vendor.init_calls == 0 &&
    vendor.power.empty() && vendor.exit_calls == 0,
    "invalid bus must be rejected before any vendor operation or runtime attempt");
  {
    chassis::BxiPciTransport valid(0);
    require(chassis::BxiPciTransport::runtime_used() && vendor.init_calls == 1,
      "a valid first transport must remain possible after an invalid bus");
  }
  require(vendor.power == std::vector<unsigned>{0} && vendor.exit_calls == 1,
    "last transport must power off and exit exactly once");
  require(!chassis::BxiPciTransport::runtime_cleanup_failed(),
    "successful cleanup must not set the failure latch");
}

void concurrent_buses_close_once_and_cannot_reopen()
{
  require(!chassis::BxiPciTransport::runtime_used(), "each isolated process starts unused");
  auto first = std::make_unique<chassis::BxiPciTransport>(0);
  auto second = std::make_unique<chassis::BxiPciTransport>(1);
  require(vendor.init_calls == 1 && vendor.cpu == -1 &&
    chassis::BxiPciTransport::runtime_used(), "concurrent buses must share one vendor runtime");
  require(vendor.power.empty(), "opening a transport must not turn on motor power");
  unsigned first_received = 0;
  unsigned second_received = 0;
  first->set_receive_handler([&](const iswv::CanFrame &) {++first_received;});
  second->set_receive_handler([&](const iswv::CanFrame &) {++second_received;});
  deliver(0);
  deliver(1);
  require(first_received == 1 && second_received == 1, "callbacks must route to their active bus");

  rejects<std::runtime_error>([] {chassis::BxiPciTransport duplicate(0);});
  require(vendor.init_calls == 1 && vendor.exit_calls == 0 && vendor.power.empty(),
    "duplicate bus rejection must not change shared runtime ownership");
  first.reset();
  require(vendor.exit_calls == 0 && vendor.power.empty() && second->is_open(),
    "closing one bus must leave the other bus and shared power interface open");
  deliver(0);
  deliver(1);
  require(first_received == 1 && second_received == 2,
    "late callback must ignore the destroyed bus while the remaining bus receives normally");

  iswv::CanFrame frame;
  frame.id = 0x123;
  frame.size = 1;
  frame.data[0] = 0x5a;
  require(bool(second->send(frame)) && vendor.send_calls == 1 && vendor.last_sent.bus == 1,
    "remaining bus must still send through the existing runtime");
  second.reset();
  require(vendor.power == std::vector<unsigned>{0} && vendor.exit_calls == 1,
    "last bus must power off and exit once even after a rejected duplicate constructor");
  require(vendor.events == std::vector<std::string>{"init", "send", "power_off", "power_read", "exit"},
    "power must be switched off and read back before vendor exit");
  deliver(0);
  deliver(1);
  require(first_received == 1 && second_received == 2,
    "vendor callback surviving exit must not reach destroyed handlers");

  rejects<std::runtime_error>([] {chassis::BxiPciTransport next_session(2);});
  require(chassis::BxiPciTransport::runtime_used() && vendor.init_calls == 1 &&
    vendor.exit_calls == 1 && vendor.power == std::vector<unsigned>{0},
    "a second hardware session must be rejected before vendor reinitialization or power changes");
}

void failed_init_cannot_be_retried(int result)
{
  require(!chassis::BxiPciTransport::runtime_used(), "failed-init fixture must start unused");
  vendor.init_result = result;
  rejects<std::runtime_error>([] {chassis::BxiPciTransport failed(0);});
  require(chassis::BxiPciTransport::runtime_used() && vendor.init_calls == 1,
    "every nonzero vendor result must fail and retain the attempted-runtime latch");
  const auto events = vendor.events;
  const auto power = vendor.power;
  const auto exits = vendor.exit_calls;
  vendor.init_result = 0;
  rejects<std::runtime_error>([] {chassis::BxiPciTransport retry(0);});
  require(vendor.init_calls == 1 && vendor.events == events && vendor.power == power &&
    vendor.exit_calls == exits, "failed initialization cannot be retried or power hardware in-process");
  deliver(0);
}

void timing_validation_failure_cleans_up(bool fail_cleanup)
{
  chassis::bxi_timing::configure_can6_bitrate(1000000);
  vendor.exit_result = fail_cleanup ? 1 : 0;
  // This mock init deliberately omits the seven timing setter pairs.
  rejects<std::runtime_error>([] {chassis::BxiPciTransport failed(0);});
  require(vendor.init_calls == 1 && vendor.exit_calls == 1 && vendor.power.empty(),
    "unverified CAN timing must fail and exit before motor power is enabled");
  require(chassis::BxiPciTransport::runtime_cleanup_failed() == fail_cleanup,
    "timing failure must propagate vendor cleanup failure");
  rejects<std::runtime_error>([] {chassis::BxiPciTransport retry(0);});
  require(vendor.init_calls == 1 && vendor.exit_calls == 1,
    "timing failure must retire the vendor runtime without retry");
}

void cleanup_failure_latches(int power_result, int exit_result, int power_read_result = 0)
{
  require(!chassis::BxiPciTransport::runtime_cleanup_failed(), "each cleanup case starts healthy");
  auto first = std::make_unique<chassis::BxiPciTransport>(0);
  auto last = std::make_unique<chassis::BxiPciTransport>(1);
  vendor.power_result = power_result;
  vendor.power_read_result = power_read_result;
  vendor.exit_result = exit_result;
  first.reset();
  require(!chassis::BxiPciTransport::runtime_cleanup_failed() && vendor.power.empty() &&
    vendor.exit_calls == 0, "a remaining transport must defer shared cleanup");
  last.reset();
  require(chassis::BxiPciTransport::runtime_cleanup_failed(),
    "reported power-off or vendor-exit failure must remain latched");
  require(vendor.power == std::vector<unsigned>{0} && vendor.exit_calls == 1 &&
    vendor.power_read_calls >= 1 && vendor.events.front() == "init" &&
    vendor.events.at(1) == "power_off" && vendor.events.at(2) == "power_read" &&
    vendor.events.back() == "exit",
    "cleanup must attempt power-off, feedback and vendor exit even if one fails");
  vendor.power_result = 0;
  vendor.power_read_result = 0;
  vendor.exit_result = 0;
  rejects<std::runtime_error>([] {chassis::BxiPciTransport retry(0);});
  require(chassis::BxiPciTransport::runtime_cleanup_failed() && vendor.init_calls == 1 &&
    vendor.power == std::vector<unsigned>{0} && vendor.exit_calls == 1,
    "later successful stub results cannot clear the latch or permit vendor reinitialization");
  deliver(0);
  deliver(1);
}

template<typename Function>
void isolated(const char * name, Function function)
{
  // board.cpp intentionally retains its runtime latch for the entire process.
  // Fork before any transport exists so independent cases get fresh statics.
  const pid_t child = fork();
  if (child < 0) {throw std::runtime_error("cannot fork isolated runtime test");}
  if (child == 0) {
    alarm(5);
    try {
      function();
      _exit(0);
    } catch (const std::exception & error) {
      std::cerr << name << ": " << error.what() << std::endl;
      _exit(1);
    }
  }
  int status = 0;
  pid_t waited;
  do {waited = waitpid(child, &status, 0);} while (waited < 0 && errno == EINTR);
  require(waited == child && WIFEXITED(status) && WEXITSTATUS(status) == 0,
    "isolated BXI runtime scenario failed or timed out");
}
}  // namespace

// This executable links board.cpp directly. These stubs satisfy every vendor
// reference; libbxi_pci_drv.a, /dev/mem and real CAN threads are never used.
extern "C" int bxi_pci_init(canfd_rx_call callback, void * context, int cpu)
{
  ++vendor.init_calls;
  vendor.events.emplace_back("init");
  vendor.receive = callback;
  vendor.context = context;
  vendor.cpu = cpu;
  return vendor.init_result;
}

extern "C" int bxi_pci_exit()
{
  ++vendor.exit_calls;
  vendor.events.emplace_back("exit");
  // Match the actual vendor defect: callbacks may still arrive after exit.
  return vendor.exit_result;
}

extern "C" int canfd_send_packet(canfd_packet * packets, unsigned int count)
{
  ++vendor.send_calls;
  vendor.events.emplace_back("send");
  if (count != 1 || packets == nullptr) {return -1;}
  vendor.last_sent = packets[0];
  return 0;
}

extern "C" int motor_pwr_set(unsigned int power)
{
  vendor.power.push_back(power);
  vendor.events.emplace_back(power ? "power_on" : "power_off");
  return vendor.power_result;
}

extern "C" int motor_pwr_get()
{
  ++vendor.power_read_calls;
  vendor.events.emplace_back("power_read");
  return vendor.power_read_result;
}

int main()
{
  try {
    isolated("invalid bus", invalid_bus_does_not_consume_runtime);
    isolated("shared buses and session retirement", concurrent_buses_close_once_and_cannot_reopen);
    isolated("negative init result", [] {failed_init_cannot_be_retried(-1);});
    isolated("positive init result", [] {failed_init_cannot_be_retried(1);});
    isolated("timing validation failure", [] {timing_validation_failure_cleans_up(false);});
    isolated("timing validation cleanup failure", [] {timing_validation_failure_cleans_up(true);});
    isolated("power-off failure", [] {cleanup_failure_latches(-1, 0);});
    isolated("vendor-exit failure", [] {cleanup_failure_latches(0, 1);});
    isolated("power remains on", [] {cleanup_failure_latches(0, 0, 1);});
    isolated("power readback failure", [] {cleanup_failure_latches(0, 0, -1);});
    std::cout << "BXI runtime tests passed\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "BXI runtime: " << error.what() << '\n';
    return 1;
  }
}
