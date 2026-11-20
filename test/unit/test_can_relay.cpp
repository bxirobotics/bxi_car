#include "relay/control.h"
#include "iswv/fake_transport.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

namespace
{
using namespace std::chrono_literals;
using Relay = chassis::CanRelay;

void require(bool condition, const char * message)
{
  if (!condition) {throw std::runtime_error(message);}
}

struct Rig
{
  enum class Fault {none, address, address_high, bitrate, bitrate_high, mode,
    mode_high, ack_mask, watchdog, echo, readback};
  std::shared_ptr<iswv::FakeTransport> bus = std::make_shared<iswv::FakeTransport>();
  Relay::Config config;
  Relay relay;
  Relay::Clock::time_point now = Relay::Clock::now();
  std::array<std::uint16_t, 64> outputs{};
  Fault fault{Fault::none};
  unsigned writes{0};
  bool normalized_readback{false};

  explicit Rig(Relay::Config cfg = {}) : config(cfg), relay(bus, cfg)
  {
    for (unsigned i = 0; i < outputs.size(); ++i) {outputs[i] = i % 2 ? 0x1234 : 0;}
    bus->set_send_hook([this](const auto & request) {
        require(!request.extended && !request.fd && !request.remote && !request.error &&
          !request.bitrate_switch && !request.error_state_indicator,
          "relay requests must be classic standard frames");
        require((request.id & 0xff) == config.address, "request must use configured address");
        require(std::all_of(request.data.begin() + request.size, request.data.end(),
          [](auto value) {return value == 0;}), "unused payload bytes must be zero");
        auto response = request;
        const auto channel = config.channel - 1;
        switch (request.id >> 8) {
          case 7:
            require(request.size == 2, "register read requests require exactly two bytes");
            response.size = 2 + 2 * request.data[1];
            if (request.data[0] == 0x43) {
              require(request.data[1] == 2, "module info must read address and bitrate together");
              response.data[2] = fault == Fault::address_high ? 1 : 0;
              response.data[3] = fault == Fault::address ? 0 : config.address;
              response.data[4] = fault == Fault::bitrate_high ? 1 : 0;
              response.data[5] = fault == Fault::bitrate ? 9 : 13;
            } else if (request.data[0] == 0x40) {
              require(request.data[1] == 2, "settings must read acknowledgement and watchdog words");
              response.data[3] = fault == Fault::ack_mask ? 1 : 0;
              response.data[5] = fault == Fault::watchdog ? 1 : 0;
            } else {
              require(request.data[0] == channel && request.data[1] == 1,
                "mode read must address only the selected zero-based channel");
              response.data[2] = fault == Fault::mode_high ? 1 : 0;
              response.data[3] = fault == Fault::mode ? 3 : 0;
            }
            break;
          case 3: {
            require(request.size == 1 && request.data[0] == channel,
              "single-coil reads require exactly one zero-based channel byte");
            response.size = 3;
            auto value = outputs[channel];
            if (normalized_readback && value != 0) {value = 1;}
            if (fault == Fault::readback && writes != 0) {value = value == 0 ? 1 : 0;}
            response.data[1] = value >> 8;
            response.data[2] = value & 0xff;
            break;
          }
          case 2:
            require(request.size == 3 && request.data[0] == channel && request.data[2] == 0 &&
              (request.data[1] == 0 || request.data[1] == 0xff),
              "single-coil writes require channel plus 0000 or FF00");
            ++writes;
            outputs[channel] = (std::uint16_t(request.data[1]) << 8) | request.data[2];
            if (fault == Fault::echo) {response.data[2] ^= 1;}
            break;
          default: throw std::runtime_error("unexpected relay function or configuration write");
        }
        bus->inject(response);
      });
  }

  void advance(unsigned ticks = 1)
  {
    for (unsigned i = 0; i < ticks; ++i) {relay.update(now += 1ms);}
  }

  void initialize()
  {
    relay.start(now);
    advance(8);
    require(relay.ready() && !relay.failed(), "relay startup must confirm logical OFF");
  }
};

void initialization_and_toggle_preserve_other_outputs()
{
  Rig rig;
  const auto original = rig.outputs;
  require(!rig.relay.toggle() && rig.bus->sent_frames().empty(),
    "construction and pre-start toggles must not send frames");
  rig.relay.start(rig.now);
  require(!rig.relay.toggle(), "initialization must not queue a button press");
  rig.advance(5);
  require(!rig.relay.ready(), "write echo is insufficient without independent readback");
  rig.advance();
  auto expected = original;
  expected[1] = 0;
  require(rig.relay.ready() && rig.writes == 1 && rig.outputs == expected,
    "startup must confirm CH2 OFF without touching the other 63 outputs");
  require(rig.relay.take_report().find("state=OFF") != std::string::npos &&
    rig.relay.take_report().empty(), "confirmed state report must be consumed once");
  const auto frames = rig.bus->sent_frames();
  require(frames.size() == 6 && frames[0].id == 0x701 && frames[0].size == 2 &&
    frames[0].data[0] == 0x43 && frames[0].data[1] == 2 && frames[1].id == 0x701 &&
    frames[1].data[0] == 1 && frames[1].data[1] == 1 && frames[2].id == 0x701 &&
    frames[2].data[0] == 0x40 && frames[2].data[1] == 2 && frames[3].id == 0x301 &&
    frames[3].size == 1 && frames[3].data[0] == 1 && frames[4].id == 0x201 &&
    frames[4].size == 3 && frames[4].data[0] == 1 && frames[4].data[1] == 0 &&
    frames[4].data[2] == 0 && frames[5].id == 0x301,
    "startup must use the exact V3.0 register and single-coil protocol sequence");
  require(rig.relay.toggle(), "ready relay must accept a toggle");
  rig.normalized_readback = true;
  rig.advance(4);
  expected[1] = 0xff00;
  require(rig.outputs == expected && rig.relay.take_report().find("state=ON") != std::string::npos,
    "any nonzero readback confirms ON; only CH2 may be written");
  require(rig.relay.toggle(), "second press must request OFF");
  rig.advance(4);
  expected[1] = 0;
  require(rig.outputs == expected, "second press must turn only CH2 OFF");
  const auto count = rig.bus->sent_frames().size();
  rig.advance(20);
  rig.relay.start(rig.now);
  require(rig.bus->sent_frames().size() == count, "idle updates and repeated start must not send");
}

void inactive_output_and_active_low_configuration()
{
  Rig rig({0xab, 64, false, 500ms});
  const auto original = rig.outputs;
  rig.initialize();
  require(rig.writes == 0 && rig.outputs == original,
    "a nonzero active-low output is already logically OFF and needs no write");
  require(rig.relay.toggle(), "active-low configured output must toggle");
  rig.advance(4);
  auto expected = original;
  expected[63] = 0;
  require(rig.outputs == expected && rig.relay.take_report().find("state=ON") != std::string::npos,
    "CH64 active-low ON must clear only the final channel");
  const auto frames = rig.bus->sent_frames();
  require(frames[1].data[0] == 63 && frames[4].id == 0x3ab && frames[4].data[0] == 63,
    "mode and coil requests must both use zero-based CH64 address 63");
}

void busy_presses_coalesce_without_replaying_a_queue()
{
  Rig rig;
  rig.outputs[1] = 0;
  rig.initialize();
  require(rig.relay.toggle(), "ON request must be accepted");
  rig.advance(2);  // The ON write is in flight.
  require(rig.relay.toggle() && rig.relay.toggle(), "busy presses must update the desired state");
  rig.advance(8);
  require(rig.writes == 1 && rig.outputs[1] != 0,
    "two busy presses must cancel without extra output writes");
  require(rig.relay.toggle(), "OFF request must be accepted");
  rig.advance(2);  // The OFF write is in flight.
  require(rig.relay.toggle(), "new desired ON state must be accepted while busy");
  rig.advance(8);
  require(rig.writes == 3 && rig.outputs[1] != 0,
    "one busy press must request only the latest desired state after confirmation");
}

void faults_latch_unknown_without_retry_or_rollback()
{
  for (const auto fault : {Rig::Fault::address, Rig::Fault::address_high, Rig::Fault::bitrate,
      Rig::Fault::bitrate_high, Rig::Fault::mode, Rig::Fault::mode_high, Rig::Fault::ack_mask,
      Rig::Fault::watchdog, Rig::Fault::echo, Rig::Fault::readback})
  {
    Rig rig;
    rig.fault = fault;
    rig.relay.start(rig.now);
    rig.advance(10);
    require(rig.relay.failed() && !rig.relay.ready() && !rig.relay.toggle(),
      "bad address, bitrate, mode, settings, echo and readback must disable relay control");
    require(rig.relay.take_report().find("UNKNOWN") != std::string::npos,
      "failed state must be reported as UNKNOWN");
    require(rig.writes == (fault == Rig::Fault::echo || fault == Rig::Fault::readback ? 1U : 0U),
      "startup configuration failures must be detected before any output or EEPROM write");
    const auto count = rig.bus->sent_frames().size();
    rig.relay.start(rig.now);
    rig.advance(30);
    require(rig.bus->sent_frames().size() == count && rig.relay.take_report().empty(),
      "failure must neither retry, roll back nor restart");
  }
  for (unsigned failure = 0; failure < 4; ++failure) {
    Rig rig;
    rig.initialize();
    const auto count = rig.bus->sent_frames().size();
    if (failure == 0) {
      rig.bus->close();
    } else if (failure == 1) {
      rig.bus->inject_error(iswv::make_error(iswv::ErrorCode::transport_error, "test failure"));
    } else {
      iswv::CanFrame frame;
      frame.id = failure == 2 ? 0x80 : 0x301;
      frame.error = failure == 2;
      frame.size = failure == 2 ? 8 : 3;
      for (unsigned n = 0; n < (failure == 2 ? 1U : 257U); ++n) {rig.bus->inject(frame);}
    }
    rig.advance();
    require(rig.relay.failed() && !rig.relay.toggle(),
      "disconnect, transport errors, error frames and queue overflow must fail closed to requests");
    rig.advance(20);
    require(rig.bus->sent_frames().size() == count, "communication faults must preserve outputs");
  }
}

void malformed_and_old_protocol_frames_cannot_complete_an_exchange()
{
  Rig rig;
  rig.bus->set_send_hook({});
  rig.relay.start(rig.now);
  for (unsigned invalid = 0; invalid < 10; ++invalid) {
    iswv::CanFrame frame;
    frame.id = 0x701;
    frame.size = 6;
    frame.data[0] = 0x43;
    frame.data[1] = 2;
    frame.data[3] = 1;
    frame.data[5] = 13;
    switch (invalid) {
      case 0: frame.id = 0x702; break;
      case 1: frame.extended = true; break;
      case 2: frame.fd = true; break;
      case 3: frame.remote = true; break;
      case 4: frame.size = 8; break;
      case 5: frame.data[0] = 0x44; break;
      case 6: frame.data[1] = 1; break;
      case 7: frame.id = 0x401; frame.size = 8; frame.data[0] = 0xa1; break;
      case 8: frame.id = 0x201; frame.size = 8; break;
      case 9: frame.id = 0x101; frame.size = 8; break;
    }
    rig.bus->inject(frame);
  }
  rig.advance();
  require(!rig.relay.failed() && !rig.relay.ready() && rig.bus->sent_frames().size() == 1,
    "wrong ID, format, register count, DLC and legacy replies must not complete an exchange");
  rig.relay.update(rig.now += 500ms);
  require(rig.relay.failed() && !rig.relay.toggle(), "response timeout must latch UNKNOWN");
  rig.relay.start(rig.now);
  rig.advance(20);
  require(rig.bus->sent_frames().size() == 1, "timeouts must not automatically retry");
}

void coil_replies_require_matching_channel_dlc_and_full_write_echo()
{
  Rig rig;
  rig.outputs[1] = 0;
  rig.initialize();
  rig.bus->set_send_hook({});
  require(rig.relay.toggle(), "ready relay must accept a pending ON request");
  rig.advance();
  iswv::CanFrame read;
  read.id = 0x301;
  read.size = 3;
  read.data[0] = 1;
  auto wrong_channel = read;
  wrong_channel.data[0] = 2;
  auto wrong_dlc = read;
  wrong_dlc.size = 8;
  rig.bus->inject(wrong_channel);
  rig.bus->inject(wrong_dlc);
  const auto count = rig.bus->sent_frames().size();
  rig.advance();
  require(rig.bus->sent_frames().size() == count, "wrong-channel and wrong-DLC reads must be ignored");
  rig.bus->inject(read);
  rig.advance();
  auto echo = rig.bus->sent_frames().back();
  require(echo.id == 0x201 && echo.data[1] == 0xff && echo.data[2] == 0,
    "ON command must be single-channel FF00");
  auto bad_echo_channel = echo;
  bad_echo_channel.data[0] = 2;
  auto bad_echo_dlc = echo;
  bad_echo_dlc.size = 8;
  rig.bus->inject(bad_echo_channel);
  rig.bus->inject(bad_echo_dlc);
  rig.advance();
  require(rig.bus->sent_frames().back().id == 0x201 && !rig.relay.failed(),
    "wrong-channel and wrong-DLC echoes must not start verification");
  rig.bus->inject(echo);
  rig.advance();
  require(rig.bus->sent_frames().back().id == 0x301, "correct echo must request independent readback");
  read.data[2] = 1;
  rig.bus->inject(read);
  rig.advance();
  require(!rig.relay.failed() && rig.relay.take_report().find("state=ON") != std::string::npos,
    "0001 readback must confirm the FF00 command as ON");
}

void transmit_failure_latches_without_retry()
{
  class RejectingTransport final : public iswv::ICanTransport
  {
  public:
    unsigned sends{0};
    iswv::Result<void> send(const iswv::CanFrame &) override
    {
      ++sends;
      return iswv::Result<void>::failure(iswv::ErrorCode::transport_error, "test TX failure");
    }
    void set_receive_handler(iswv::ReceiveHandler) override {}
    void set_error_handler(iswv::TransportErrorHandler) override {}
    bool is_open() const noexcept override {return true;}
    std::string name() const override {return "rejecting test transport";}
  };
  auto bus = std::make_shared<RejectingTransport>();
  Relay relay(bus);
  relay.start();
  require(relay.failed() && !relay.toggle() &&
    relay.take_report().find("UNKNOWN") != std::string::npos,
    "a failed transmission must immediately disable control and report UNKNOWN");
  relay.start();
  relay.update();
  require(bus->sends == 1, "failed transmissions must not be retried");
}

void invalid_configurations_and_destruction_do_not_send()
{
  const auto rejected = [](Relay::Config config) {
      auto bus = std::make_shared<iswv::FakeTransport>();
      bool invalid = false;
      try {Relay relay(bus, config);} catch (const std::invalid_argument &) {invalid = true;}
      require(invalid && bus->sent_frames().empty(), "invalid config must fail before sending");
    };
  rejected({0, 2, true, 500ms});
  rejected({256, 2, true, 500ms});
  rejected({1, 0, true, 500ms});
  rejected({1, 65, true, 500ms});
  rejected({1, 2, true, 19ms});
  rejected({1, 2, true, 2001ms});
  bool invalid = false;
  try {Relay relay(nullptr);} catch (const std::invalid_argument &) {invalid = true;}
  require(invalid, "null transport must be rejected");
  auto bus = std::make_shared<iswv::FakeTransport>();
  {
    Relay relay(bus);
    relay.start();
  }
  require(bus->sent_frames().size() == 1, "destruction must not send an output-OFF request");
  bus->inject_error(iswv::make_error(iswv::ErrorCode::transport_error, "late error"));
  iswv::CanFrame frame;
  frame.id = 0x701;
  frame.size = 6;
  bus->inject(frame);
  require(bus->sent_frames().size() == 1, "late callbacks must be detached without sending");
}
}  // namespace

int main()
{
  try {
    initialization_and_toggle_preserve_other_outputs();
    inactive_output_and_active_low_configuration();
    busy_presses_coalesce_without_replaying_a_queue();
    faults_latch_unknown_without_retry_or_rollback();
    malformed_and_old_protocol_frames_cannot_complete_an_exchange();
    coil_replies_require_matching_channel_dlc_and_full_write_echo();
    transmit_failure_latches_without_retry();
    invalid_configurations_and_destruction_do_not_send();
    std::cout << "CAN relay V3.0 protocol tests passed\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "CAN relay V3.0 protocol: " << error.what() << '\n';
    return 1;
  }
}
