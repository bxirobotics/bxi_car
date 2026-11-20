// Pinquanwei CAN digital-output manual V3.0, pp. 19, 21, 24, 27 and 33.
// Commission CAN6 separately from the motion controller. Never writes coils.
#include "bxi/board.h"
#include "bxi/can_timing.h"
#include "example/hardware_owner.h"

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <csignal>
#include <cstdint>
#include <iostream>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace
{
using namespace std::chrono_literals;

volatile std::sig_atomic_t cancelled = 0;

void cancel(int) {cancelled = 1;}

void check_cancelled()
{
  if (cancelled) {throw std::runtime_error("cancelled; shutting down shared power");}
}

struct Options
{
  unsigned bitrate{1000000};
  unsigned address{1};
  unsigned channel{2};
  bool power_on{false};
  bool set_1m{false};
};

unsigned number(const std::string & text)
{
  if (text.empty() || text.find_first_not_of("0123456789") != std::string::npos) {
    throw std::invalid_argument("expected an unsigned decimal argument");
  }
  const auto value = std::stoul(text);
  if (value > 1000000) {throw std::invalid_argument("numeric argument out of range");}
  return static_cast<unsigned>(value);
}

Options parse(int argc, char ** argv)
{
  Options result;
  for (int i = 1; i < argc; ++i) {
    const std::string arg(argv[i]);
    if (arg == "--power-on") {result.power_on = true;}
    else if (arg == "--set-1m") {result.set_1m = true;}
    else if ((arg == "--bitrate" || arg == "--address" || arg == "--channel") && i + 1 < argc) {
      const auto value = number(argv[++i]);
      if (arg == "--bitrate") {result.bitrate = value;}
      else if (arg == "--address") {result.address = value;}
      else {result.channel = value;}
    } else {throw std::invalid_argument("unknown or incomplete option: " + arg);}
  }
  if ((result.bitrate != 250000 && result.bitrate != 1000000) ||
    result.address < 1 || result.address > 255 || result.channel < 1 || result.channel > 64)
  {
    throw std::invalid_argument("use bitrate 250000/1000000, address 1..255, channel 1..64");
  }
  if (result.set_1m && result.bitrate != 250000) {
    throw std::invalid_argument("--set-1m requires --bitrate 250000");
  }
  return result;
}

struct Inbox
{
  std::mutex mutex;
  std::condition_variable changed;
  unsigned id{0};
  unsigned size{0};
  std::optional<iswv::CanFrame> response;
  std::string error;
};

void inspect(const Options & options, bool & transport_opened)
{
  chassis::bxi_timing::configure_can6_bitrate(options.bitrate);
  auto bus = std::make_shared<chassis::BxiPciTransport>(6, true);
  transport_opened = true;
  auto inbox = std::make_shared<Inbox>();
  bus->set_receive_handler([inbox](const iswv::CanFrame & frame) {
      std::lock_guard<std::mutex> lock(inbox->mutex);
      if (frame.error) {inbox->error = "CAN error frame";}
      else if (!frame.extended && !frame.fd && !frame.remote && !frame.bitrate_switch &&
        !frame.error_state_indicator && frame.id == inbox->id && frame.size == inbox->size)
      {
        inbox->response = frame;
      }
      inbox->changed.notify_one();
    });
  bus->set_error_handler([inbox](const iswv::Error & error) {
      std::lock_guard<std::mutex> lock(inbox->mutex);
      inbox->error = error.message;
      inbox->changed.notify_one();
    });
  if (options.power_on) {
    check_cancelled();
    const auto result = bus->set_motor_power(true);
    if (!result) {throw std::runtime_error(result.error().message);}
    const auto settled = std::chrono::steady_clock::now() + 4s;
    while (std::chrono::steady_clock::now() < settled) {
      check_cancelled();
      std::this_thread::sleep_for(20ms);
    }
    if (motor_pwr_get() != 1) {throw std::runtime_error("motor power ON feedback not confirmed after 4s");}
    std::cerr << "RELAY_SETUP motor_power=ON confirmed; no motor commands\n";
  }
  const auto exchange = [&](unsigned function, const std::vector<std::uint8_t> & data,
      unsigned response_size) {
      check_cancelled();
      iswv::CanFrame request;
      request.id = (function << 8) | options.address;
      request.size = static_cast<std::uint8_t>(data.size());
      std::copy(data.begin(), data.end(), request.data.begin());
      {
        std::lock_guard<std::mutex> lock(inbox->mutex);
        inbox->response.reset();
        inbox->id = request.id;
        inbox->size = response_size;
      }
      const auto sent = bus->send(request);
      if (!sent) {throw std::runtime_error(sent.error().message);}
      std::unique_lock<std::mutex> lock(inbox->mutex);
      const auto deadline = std::chrono::steady_clock::now() + 1s;
      while (!inbox->response && inbox->error.empty()) {
        check_cancelled();
        if (std::chrono::steady_clock::now() >= deadline) {
          throw std::runtime_error("relay response timeout");
        }
        inbox->changed.wait_for(lock, 20ms);
      }
      check_cancelled();
      if (!inbox->error.empty()) {throw std::runtime_error(inbox->error);}
      return *inbox->response;
    };
  const auto read_registers = [&](std::uint8_t start, std::uint8_t count) {
      const auto response = exchange(7, {start, count}, 2 + 2 * count);
      if (response.data[0] != start || response.data[1] != count) {
        throw std::runtime_error("register response header mismatch");
      }
      std::vector<unsigned> values;
      for (unsigned i = 0; i < count; ++i) {
        values.push_back((unsigned(response.data[2 + i * 2]) << 8) | response.data[3 + i * 2]);
      }
      return values;
    };
  const auto info = read_registers(0x43, 2);
  if (info[0] != options.address) {throw std::runtime_error("module address mismatch");}
  const auto expected_rate = options.bitrate == 250000 ? 9U : 13U;
  if (info[1] != expected_rate && !(options.set_1m && info[1] == 13)) {
    throw std::runtime_error("stored module bitrate differs from requested bus rate");
  }
  const auto mode = read_registers(static_cast<std::uint8_t>(options.channel - 1), 1);
  const auto settings = read_registers(0x40, 2);
  std::cout << "RELAY_SETUP connected bitrate=" << options.bitrate << " address=" << info[0]
            << " stored_rate_code=" << info[1] << " channel=" << options.channel
            << " mode=" << mode[0] << " reply_mask=" << settings[0]
            << " watchdog=" << settings[1] << '\n';
  if (mode[0] != 0 || settings[0] != 0 || settings[1] != 0) {
    throw std::runtime_error("pump control requires normal mode, all replies, watchdog disabled");
  }
  const auto coil = static_cast<std::uint8_t>(options.channel - 1);
  const auto output = exchange(3, {coil}, 3);
  if (output.data[0] != coil) {throw std::runtime_error("output response channel mismatch");}
  std::cout << "RELAY_SETUP Y" << options.channel << '='
            << ((output.data[1] || output.data[2]) ? "ON" : "OFF") << " (read only)\n";
  if (options.set_1m) {
    if (info[1] == 13) {
      std::cout << "RELAY_SETUP 1M already stored; EEPROM not rewritten\n";
    } else {
      const std::vector<std::uint8_t> data{0x44, 1, 0, 13};
      const auto echo = exchange(6, data, 4);
      if (!std::equal(data.begin(), data.end(), echo.data.begin())) {
        throw std::runtime_error("bitrate write echo mismatch; state unknown; do not retry blindly");
      }
      std::this_thread::sleep_for(100ms);
      const auto saved = read_registers(0x44, 1);
      if (saved[0] != 13) {throw std::runtime_error("1M setting readback mismatch");}
      std::cout << "RELAY_SETUP 1M write echo and register readback confirmed\n";
    }
    std::cout << "RELAY_SETUP power-cycle module, wait for its power LED to extinguish, "
              << "then verify in a NEW process at 1000000; no coils written\n";
  }
  bus->set_receive_handler({});
  bus->set_error_handler({});
  bus.reset();
}
}  // namespace

int main(int argc, char ** argv)
{
  if (argc == 2 && std::string(argv[1]) == "--help") {
    std::cout << "Usage: can_relay_setup [--bitrate 250000|1000000] [--address 1] [--channel 2]\n"
      "                       [--power-on] [--set-1m]\n"
      "Read Pinquanwei relay registers/output on CAN6; never writes any coil.\n"
      "--set-1m writes ONLY bitrate register 0x44=13 at 250K; module power-cycle required.\n"
      "--power-on enables shared supply for this check; shutdown always switches it off.\n"
      "Stop the main service first. This command holds its hardware owner lock.\n";
    return 0;
  }
  int result = 0;
  bool transport_opened = false;
  // Keep ownership through cleanup reporting, as in the main controller. The
  // vendor poll thread survives transport destruction until this process exits.
  std::unique_ptr<chassis::HardwareOwner> owner;
  std::signal(SIGINT, cancel);
  std::signal(SIGTERM, cancel);
  std::signal(SIGHUP, cancel);
  try {
    const auto options = parse(argc, argv);
    check_cancelled();
    owner = std::make_unique<chassis::HardwareOwner>();
    inspect(options, transport_opened);
  }
  catch (const std::exception & error) {
    std::cerr << "RELAY_SETUP failed: " << error.what() << '\n';
    result = 1;
  }
  if (chassis::BxiPciTransport::runtime_cleanup_failed() ||
    (chassis::BxiPciTransport::runtime_used() && !transport_opened))
  {
    std::cerr << "RELAY_SETUP shutdown unconfirmed; service must remain stopped\n";
    return 2;
  }
  if (transport_opened) {
    std::cerr << "RELAY_SETUP cleanup=OK shared power OFF\n";
  }
  return result;
}
