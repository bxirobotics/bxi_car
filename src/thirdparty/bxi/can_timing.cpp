#include "bxi/can_timing.h"

#include <array>
#include <cstddef>
#include <cstdio>
#include <stdexcept>

// Adapted from car3/wheeled-lift-robot/owner/backend_real.c. The vendor archive
// configures CAN0..CAN6 in that order, with one prescaler/bit-timing pair per bus.
// Its CAN6 1M baseline is BRP=0, SJW/TSEG2/TSEG1=9/9/28; BRP=1/3 gives 500K/250K.
// GNU ld --wrap redirects only these two setters; no FPGA clock is guessed.
extern "C"
{
int __real_XCanFd_SetBaudRatePrescaler(void * instance, unsigned char value);
unsigned char XCanFd_GetBaudRatePrescaler(void * instance);
int __real_XCanFd_SetBitTiming(
  void * instance, unsigned char sjw, unsigned char s2, unsigned short s1);
void XCanFd_GetBitTiming(
  void * instance, unsigned char * sjw, unsigned char * s2, unsigned char * s1);
}

namespace
{
struct Timing
{
  void * instance{nullptr};
  unsigned char prescaler{0};
  unsigned char sjw{0};
  unsigned char s2{0};
  unsigned short s1{0};
};

unsigned can6_bitrate{0};
std::array<Timing, 7> timings{};
std::size_t prescaler_calls{0};
std::size_t segment_calls{0};
const char * error{nullptr};

void fail(const char * message)
{
  if (error == nullptr) {error = message;}
}

}  // namespace

namespace chassis
{
namespace bxi_timing
{
void configure_can6_bitrate(unsigned bitrate)
{
  if (bitrate != 250000 && bitrate != 500000 && bitrate != 1000000) {
    throw std::invalid_argument("CAN6 bitrate must be 250000, 500000 or 1000000");
  }
  can6_bitrate = bitrate;
}

void before_init()
{
  timings = {};
  prescaler_calls = segment_calls = 0;
  error = nullptr;
}

std::string initialization_error()
{
  if (can6_bitrate == 0) {return {};}
  if (error != nullptr) {return std::string("BXI CAN6 timing: ") + error;}
  if (prescaler_calls != timings.size() || segment_calls != timings.size()) {
    return "BXI CAN6 timing: expected seven vendor initialization pairs";
  }
  // Recheck after the complete vendor init: later mode/FD setup must not have
  // overwritten either CAN6 or the unchanged nominal timing of another bus.
  std::array<unsigned char, 4> actual{};
  for (std::size_t bus = 0; bus < timings.size(); ++bus) {
    const auto & timing = timings[bus];
    if (timing.instance == nullptr) {
      return "BXI CAN6 timing: missing CAN controller instance";
    }
    actual[0] = XCanFd_GetBaudRatePrescaler(timing.instance);
    XCanFd_GetBitTiming(timing.instance, &actual[1], &actual[2], &actual[3]);
    if (actual[0] != timing.prescaler || actual[1] != timing.sjw ||
      actual[2] != timing.s2 || actual[3] != timing.s1)
    {
      return "BXI CAN6 timing: final register readback differs on CAN" +
             std::to_string(bus);
    }
  }
  std::fprintf(stderr,
    "CAN6_TIMING verified bitrate=%u brp=%u sjw=%u tseg2=%u tseg1=%u\n",
    can6_bitrate, static_cast<unsigned>(actual[0]), static_cast<unsigned>(actual[1]),
    static_cast<unsigned>(actual[2]), static_cast<unsigned>(actual[3]));
  return {};
}
}  // namespace bxi_timing
}  // namespace chassis

extern "C" int __wrap_XCanFd_SetBaudRatePrescaler(
  void * instance, unsigned char original)
{
  if (can6_bitrate == 0) {return __real_XCanFd_SetBaudRatePrescaler(instance, original);}
  const auto bus = prescaler_calls++;
  if (bus >= timings.size() || segment_calls != bus || instance == nullptr) {
    fail("unexpected prescaler call sequence");
    return -1;
  }
  for (std::size_t previous = 0; previous < bus; ++previous) {
    if (timings[previous].instance == instance) {
      fail("vendor reused a CAN controller instance");
      return -1;
    }
  }
  timings[bus].instance = instance;
  if (bus == 6 && original != 0) {
    fail("CAN6 vendor prescaler is not the pinned 1M baseline");
    return -1;
  }
  const auto value = static_cast<unsigned char>(
    bus == 6 ? 1000000 / can6_bitrate - 1 : original);
  timings[bus].prescaler = value;
  const int result = __real_XCanFd_SetBaudRatePrescaler(instance, value);
  if (result != 0 || XCanFd_GetBaudRatePrescaler(instance) != value) {
    fail("prescaler write or immediate readback failed");
  }
  return result;
}

extern "C" int __wrap_XCanFd_SetBitTiming(
  void * instance, unsigned char sjw, unsigned char s2, unsigned short s1)
{
  if (can6_bitrate == 0) {return __real_XCanFd_SetBitTiming(instance, sjw, s2, s1);}
  const auto bus = segment_calls++;
  if (bus >= timings.size() || prescaler_calls != bus + 1 ||
    instance == nullptr || timings[bus].instance != instance)
  {
    fail("unexpected bit-timing call sequence or CAN instance");
    return -1;
  }
  if (bus == 6 && (sjw != 9 || s2 != 9 || s1 != 28)) {
    fail("CAN6 vendor segments are not the pinned 1M baseline");
    return -1;
  }
  auto & timing = timings[bus];
  timing.sjw = sjw;
  timing.s2 = s2;
  timing.s1 = s1;
  const int result = __real_XCanFd_SetBitTiming(instance, sjw, s2, s1);
  unsigned char read_sjw{}, read_s2{}, read_s1{};
  XCanFd_GetBitTiming(instance, &read_sjw, &read_s2, &read_s1);
  if (result != 0 || read_sjw != sjw || read_s2 != s2 || read_s1 != s1) {
    fail("bit-timing write or immediate readback failed");
  }
  return result;
}
