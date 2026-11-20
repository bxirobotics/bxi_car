#include "bxi/can_timing.h"

#include <array>
#include <cerrno>
#include <iostream>
#include <stdexcept>
#include <sys/wait.h>
#include <unistd.h>

extern "C"
{
int __wrap_XCanFd_SetBaudRatePrescaler(void *, unsigned char);
int __wrap_XCanFd_SetBitTiming(void *, unsigned char, unsigned char, unsigned short);
}

namespace
{
struct Registers
{
  unsigned char prescaler{};
  unsigned char sjw{};
  unsigned char s2{};
  unsigned short s1{};
  int prescaler_result{};
  int segments_result{};
  bool bad_prescaler_read{false};
  bool bad_segments_read{false};
  unsigned prescaler_writes{};
  unsigned segment_writes{};
  unsigned reads{};
};

std::array<Registers, 7> registers{};
const std::array<unsigned char, 7> originals{4, 3, 2, 1, 0, 5, 0};

void require(bool condition, const char * message)
{
  if (!condition) {throw std::runtime_error(message);}
}

void enable(unsigned bitrate = 500000)
{
  chassis::bxi_timing::configure_can6_bitrate(bitrate);
  chassis::bxi_timing::before_init();
}

void initialize_bus(unsigned bus)
{
  auto * instance = &registers.at(bus);
  __wrap_XCanFd_SetBaudRatePrescaler(instance, originals.at(bus));
  __wrap_XCanFd_SetBitTiming(
    instance, bus == 6 ? 9 : bus + 1, bus == 6 ? 9 : bus + 2,
    bus == 6 ? 28 : 20 + bus);
}

void initialize_all()
{
  for (unsigned bus = 0; bus < registers.size(); ++bus) {initialize_bus(bus);}
}

void require_error(const char * message)
{
  require(!chassis::bxi_timing::initialization_error().empty(), message);
}

void disabled_preserves_vendor_calls()
{
  chassis::bxi_timing::before_init();
  auto & target = registers[0];
  target.prescaler_result = 12;
  target.segments_result = 23;
  require(__wrap_XCanFd_SetBaudRatePrescaler(&target, 8) == 12,
    "disabled wrapper must preserve the vendor prescaler return value");
  require(__wrap_XCanFd_SetBitTiming(&target, 2, 3, 19) == 23,
    "disabled wrapper must preserve the vendor bit-timing return value");
  require(target.prescaler == 8 && target.sjw == 2 && target.s2 == 3 && target.s1 == 19 &&
    target.reads == 0 && chassis::bxi_timing::initialization_error().empty(),
    "disabled wrapper must pass parameters through without requiring a timing profile");
}

void only_can6_is_configured_and_all_channels_are_verified(unsigned bitrate)
{
  enable(bitrate);
  initialize_all();
  require(chassis::bxi_timing::initialization_error().empty(), "valid initialization must pass");
  for (unsigned bus = 0; bus < registers.size(); ++bus) {
    const auto & target = registers[bus];
    require(target.prescaler == (bus == 6 ? 1000000 / bitrate - 1 : originals[bus]),
      "only CAN6 may change its vendor prescaler");
    require(target.sjw == (bus == 6 ? 9 : bus + 1) &&
      target.s2 == (bus == 6 ? 9 : bus + 2) && target.s1 == (bus == 6 ? 28 : 20 + bus),
      "every bus must retain its vendor segments");
    require(target.prescaler_writes == 1 && target.segment_writes == 1 && target.reads == 4,
      "all channels require immediate and final register verification");
  }
}

void invalid_bitrates_do_not_touch_hardware_or_enable_validation()
{
  for (const unsigned bitrate : {0u, 1u, 125000u, 333333u, 2000000u, ~0u}) {
    bool rejected = false;
    try {chassis::bxi_timing::configure_can6_bitrate(bitrate);}
    catch (const std::invalid_argument &) {rejected = true;}
    require(rejected, "unsupported bitrates must throw invalid_argument");
    require(chassis::bxi_timing::initialization_error().empty(),
      "invalid first configuration must leave validation disabled");
    for (const auto & target : registers) {
      require(target.prescaler_writes == 0 && target.segment_writes == 0 && target.reads == 0,
        "invalid configuration must not access hardware");
    }
  }
  disabled_preserves_vendor_calls();
}

void invalid_bitrate_preserves_previous_selection()
{
  chassis::bxi_timing::configure_can6_bitrate(250000);
  bool rejected = false;
  try {chassis::bxi_timing::configure_can6_bitrate(125000);}
  catch (const std::invalid_argument &) {rejected = true;}
  require(rejected, "invalid reconfiguration must throw");
  chassis::bxi_timing::before_init();
  initialize_all();
  require(chassis::bxi_timing::initialization_error().empty() && registers[6].prescaler == 3,
    "invalid reconfiguration must not replace the previous valid selection");
}

template<typename Function>
void isolated(const char * name, Function function)
{
  const auto pid = ::fork();
  require(pid >= 0, "cannot fork timing test");
  if (pid == 0) {
    ::alarm(3);
    try {function(); ::_exit(0);}
    catch (const std::exception & error) {
      std::cerr << name << ": " << error.what() << std::endl;
      ::_exit(1);
    }
  }
  int status{};
  pid_t waited;
  do {waited = ::waitpid(pid, &status, 0);} while (waited < 0 && errno == EINTR);
  require(waited == pid && WIFEXITED(status) && WEXITSTATUS(status) == 0,
    "CAN timing test failed or timed out");
}
}  // namespace

// Directly satisfy the wrapper's four vendor references. This test never links
// the PCI archive and does not need the production --wrap linker flags.
extern "C" int __real_XCanFd_SetBaudRatePrescaler(void * instance, unsigned char value)
{
  auto & target = *static_cast<Registers *>(instance);
  ++target.prescaler_writes;
  target.prescaler = value;
  return target.prescaler_result;
}

extern "C" unsigned char XCanFd_GetBaudRatePrescaler(void * instance)
{
  auto & target = *static_cast<Registers *>(instance);
  ++target.reads;
  return target.prescaler ^ (target.bad_prescaler_read ? 1 : 0);
}

extern "C" int __real_XCanFd_SetBitTiming(
  void * instance, unsigned char sjw, unsigned char s2, unsigned short s1)
{
  auto & target = *static_cast<Registers *>(instance);
  ++target.segment_writes;
  target.sjw = sjw;
  target.s2 = s2;
  target.s1 = s1;
  return target.segments_result;
}

extern "C" void XCanFd_GetBitTiming(
  void * instance, unsigned char * sjw, unsigned char * s2, unsigned char * s1)
{
  auto & target = *static_cast<Registers *>(instance);
  ++target.reads;
  *sjw = target.sjw ^ (target.bad_segments_read ? 1 : 0);
  *s2 = target.s2;
  *s1 = static_cast<unsigned char>(target.s1);
}

int main()
{
  try {
    isolated("disabled", disabled_preserves_vendor_calls);
    for (const unsigned bitrate : {250000u, 500000u, 1000000u}) {
      isolated("CAN6-only bitrate", [bitrate] {
          only_can6_is_configured_and_all_channels_are_verified(bitrate);
        });
    }
    isolated("invalid bitrates", invalid_bitrates_do_not_touch_hardware_or_enable_validation);
    isolated("invalid reconfiguration", invalid_bitrate_preserves_previous_selection);
    isolated("bit timing before prescaler", [] {
        enable();
        __wrap_XCanFd_SetBitTiming(&registers[0], 9, 9, 28);
        require_error("wrong initialization order must fail");
      });
    isolated("missing controller", [] {
        enable();
        for (unsigned bus = 0; bus < 6; ++bus) {initialize_bus(bus);}
        require_error("missing initialization pair must fail");
      });
    isolated("extra controller", [] {
        enable(); initialize_all();
        Registers extra;
        __wrap_XCanFd_SetBaudRatePrescaler(&extra, 0);
        require_error("extra initialization pair must fail");
      });
    isolated("mismatched instance", [] {
        enable();
        __wrap_XCanFd_SetBaudRatePrescaler(&registers[0], 0);
        __wrap_XCanFd_SetBitTiming(&registers[1], 9, 9, 28);
        require_error("a prescaler/segment pair must use the same instance");
      });
    isolated("duplicate controller", [] {
        enable(); initialize_bus(0);
        __wrap_XCanFd_SetBaudRatePrescaler(&registers[0], 0);
        require_error("two channel positions must not reuse one instance");
      });
    isolated("CAN6 base prescaler", [] {
        enable();
        for (unsigned bus = 0; bus < 6; ++bus) {initialize_bus(bus);}
        __wrap_XCanFd_SetBaudRatePrescaler(&registers[6], 1);
        require(registers[6].prescaler_writes == 0, "unknown baseline must not be written");
        require_error("unknown CAN6 base prescaler must fail");
      });
    for (unsigned segment = 0; segment < 3; ++segment) {
      isolated("CAN6 base segments", [segment] {
          enable();
          for (unsigned bus = 0; bus < 6; ++bus) {initialize_bus(bus);}
          __wrap_XCanFd_SetBaudRatePrescaler(&registers[6], 0);
          __wrap_XCanFd_SetBitTiming(&registers[6], segment == 0 ? 8 : 9,
            segment == 1 ? 8 : 9, segment == 2 ? 27 : 28);
          require(registers[6].segment_writes == 0, "unknown segments must not be written");
          require_error("unknown CAN6 vendor segments must fail");
        });
    }
    isolated("prescaler setter failure", [] {
        enable(); registers[6].prescaler_result = -1; initialize_all();
        require_error("prescaler setter failure must fail startup");
      });
    isolated("segments setter failure", [] {
        enable(); registers[6].segments_result = -1; initialize_all();
        require_error("bit-timing setter failure must fail startup");
      });
    isolated("immediate prescaler mismatch", [] {
        enable(); registers[6].bad_prescaler_read = true; initialize_all();
        registers[6].bad_prescaler_read = false;
        require_error("later correct readback must not erase an immediate failure");
      });
    isolated("immediate segments mismatch", [] {
        enable(); registers[6].bad_segments_read = true; initialize_all();
        registers[6].bad_segments_read = false;
        require_error("later correct readback must not erase an immediate segment failure");
      });
    isolated("CAN6 overwritten after init", [] {
        enable(); initialize_all(); registers[6].prescaler = 0;
        require_error("vendor initialization overwriting CAN6 must fail final verification");
      });
    isolated("other CAN overwritten after init", [] {
        enable(); initialize_all(); registers[2].s1 = 0;
        require_error("unchanged channels must also retain their timing after initialization");
      });
    std::cout << "CAN6 timing tests passed\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "CAN6 timing: " << error.what() << '\n';
    return 1;
  }
}
