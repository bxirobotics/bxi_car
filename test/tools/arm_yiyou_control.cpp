#include "bxi/board.h"
#include "motor/yiyou/communication.h"
#include "motor/yiyou/motor.h"
#include "motor/yiyou/position_memory.h"

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <fcntl.h>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <poll.h>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>

namespace
{
using namespace std::chrono_literals;
constexpr double jog_step_degrees = 1.0;
constexpr std::int64_t torque_step_mnm = 50;
volatile std::sig_atomic_t stop_requested = 0;
#ifdef YIYOU_CONTROL_TEST_FAST_WAIT
constexpr auto power_on_wait = 100ms;
constexpr auto save_wait = 100ms;
#else
constexpr auto power_on_wait = 4s;
constexpr auto save_wait = 2s;
#endif

// The vendor library writes diagnostics with printf() and write(1, ...), including
// from its background threads. Reserve the original pipe for complete JSON records
// before initializing the library, and route all ordinary stdout to stderr.
class JsonOutput
{
public:
  JsonOutput()
  {
    std::cout.flush();
    std::fflush(stdout);
    fd_ = dup(STDOUT_FILENO);
    if (fd_ < 0) {throw std::runtime_error("cannot reserve JSON output pipe");}
    const auto flags = fcntl(fd_, F_GETFL);
    if (flags < 0 || fcntl(fd_, F_SETFL, flags | O_NONBLOCK) < 0) {
      close(fd_);
      throw std::runtime_error("cannot make controller output nonblocking");
    }
    if (dup2(STDERR_FILENO, STDOUT_FILENO) < 0) {
      close(fd_);
      throw std::runtime_error("cannot redirect vendor diagnostics to stderr");
    }
    std::setvbuf(stdout, nullptr, _IONBF, 0);
  }

  ~JsonOutput() {close(fd_);}
  JsonOutput(const JsonOutput &) = delete;
  JsonOutput & operator=(const JsonOutput &) = delete;

  void send(std::string record) const
  {
    record += '\n';
    std::size_t offset = 0;
    const auto deadline = std::chrono::steady_clock::now() + 100ms;
    while (offset < record.size()) {
      if (stop_requested) {throw std::runtime_error("control interrupted while sending feedback");}
      const auto now = std::chrono::steady_clock::now();
      if (now >= deadline) {throw std::runtime_error("controller output stalled for 100ms");}
      const auto count = ::write(fd_, record.data() + offset, record.size() - offset);
      if (count < 0 && errno == EINTR) {continue;}
      if (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
        pollfd writable{fd_, POLLOUT, 0};
        const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now).count();
        const auto result = poll(&writable, 1, static_cast<int>(std::max<std::int64_t>(1, remaining)));
        if (result < 0 && errno == EINTR) {continue;}
        if (result < 0 || (writable.revents & (POLLHUP | POLLERR | POLLNVAL))) {
          throw std::runtime_error("controller output disconnected");
        }
        continue;
      }
      if (count <= 0) {throw std::runtime_error("controller output disconnected");}
      offset += static_cast<std::size_t>(count);
    }
  }

private:
  int fd_{-1};
};

void request_stop(int) {stop_requested = 1;}

void check_cancelled()
{
  if (stop_requested) {throw std::runtime_error("control interrupted");}
  pollfd input{STDIN_FILENO, POLLIN, 0};
  if (poll(&input, 1, 0) > 0 && (input.revents & (POLLHUP | POLLERR | POLLNVAL))) {
    throw std::runtime_error("controller input disconnected");
  }
}

void wait_power()
{
  const auto ready = std::chrono::steady_clock::now() + power_on_wait;
  while (std::chrono::steady_clock::now() < ready) {
    check_cancelled();
    std::this_thread::sleep_for(20ms);
  }
}

std::int64_t integer(const std::string & text, std::int64_t low, std::int64_t high)
{
  std::size_t end = 0;
  const auto value = std::stoll(text, &end, 10);
  if (end != text.size() || value < low || value > high) {
    throw std::invalid_argument("integer outside allowed range: " + text);
  }
  return value;
}

double real_number(const std::string & text)
{
  std::size_t end = 0;
  const auto value = std::stod(text, &end);
  if (end != text.size() || !std::isfinite(value)) {
    throw std::invalid_argument("expected a finite number: " + text);
  }
  return value;
}

struct Options
{
  std::string interface;
  std::vector<std::uint16_t> slaves;
  unsigned power_bus{3};
  double speed_rpm{1};
  double timeout_s{60};
  bool current_feedback{false};
  bool torque_control{false};
  bool manual_brake{false};
  std::optional<double> torque_limit_nm;
  std::string position_memory{chassis::PositionMemory::default_path()};
};

void usage()
{
  std::cout <<
    "Usage: arm_yiyou_control --interface NIC --slaves 1,2,3 [--power-bus 3]\n"
    "                         [--speed-rpm 1] [--timeout 60] [--current-feedback]\n"
    "                         [--position-memory FILE]\n"
    "                         [--torque-limit-nm NM]\n"
    "                         [--control-mode position|torque|manual]\n"
    "Persistent backend for yiyou_control.py; use start_yiyou_jog.sh / start_yiyou_home.sh.\n"
    "Commands on stdin: hold; step SLAVE DELTA; zero SLAVE; home; relax; release SLAVE; brake; quit.\n"
    "Manual mode never enables: release one brake, move by hand, brake, then zero.\n"
    "Position DELTA is -100..100 one-degree increments, excluding zero.\n"
    "In torque mode DELTA must be 1|-1, changing nominal motor-side torque by +/-0.05 Nm.\n"
    "Torque starts at zero; relax clears all targets and disables all selected axes.\n"
    "Torque mode uses --speed-rpm as an observed output-speed trip threshold.\n"
    "JSON responses on stdout. --slaves are EtherCAT physical chain positions 1..199.\n"
    "BXI CAN is used only for shared motor power via --power-bus; no motor CAN traffic is sent.\n"
    "Startup reads information without enabling. First step enables only its selected slave.\n"
    "Previously jogged slaves keep holding; home is an explicit command.\n"
    "zero overwrites EEPROM zero and saves all persistent parameters.\n"
    "Torque limit uses absolute drive-reported motor torque, not gearbox output torque.\n"
    "Exceeding it or losing required feedback cuts shared power and latches a fault.\n"
    "Slave 4 shares persistent multi-turn position and 0..1200 degree limits with the robot.\n"
    "Slave 4 home is refused; use bounded jog steps or an explicit zero reference.\n"
    "Help does not open EtherCAT or CAN devices.\n";
}

Options parse(int argc, char ** argv)
{
  Options result;
  std::set<std::string> seen;
  for (int i = 1; i < argc; ++i) {
    const std::string key = argv[i];
    if (!seen.insert(key).second) {throw std::invalid_argument("duplicate option: " + key);}
    if (key == "--current-feedback") {
      result.current_feedback = true;
      continue;
    }
    if (key == "--bus" || key == "--nodes" || key == "--node") {
      throw std::invalid_argument(
              key + " is a retired CAN option; use --interface NIC and --slaves positions");
    }
    if (++i == argc) {throw std::invalid_argument("missing value for " + key);}
    const std::string value = argv[i];
    if (key == "--interface") {
      if (value.empty()) {throw std::invalid_argument("--interface must not be empty");}
      result.interface = value;
    } else if (key == "--slaves") {
      if (value.empty() || value.back() == ',') {throw std::invalid_argument("invalid slave list");}
      std::istringstream list(value);
      std::set<std::uint16_t> ids;
      std::string item;
      while (std::getline(list, item, ',')) {
        const auto id = static_cast<std::uint16_t>(integer(item, 1, 199));
        if (!ids.insert(id).second) {throw std::invalid_argument("duplicate slave position");}
        result.slaves.push_back(id);
      }
    } else if (key == "--power-bus") {
      result.power_bus = static_cast<unsigned>(integer(value, 0, CANFD_DEVICE_NUM - 1));
    } else if (key == "--speed-rpm") {
      result.speed_rpm = real_number(value);
    } else if (key == "--timeout") {
      result.timeout_s = real_number(value);
    } else if (key == "--torque-limit-nm") {
      result.torque_limit_nm = real_number(value);
    } else if (key == "--control-mode") {
      if (value != "position" && value != "torque" && value != "manual") {
        throw std::invalid_argument("--control-mode must be position, torque or manual");
      }
      result.torque_control = value == "torque";
      result.manual_brake = value == "manual";
    } else if (key == "--position-memory") {
      if (value.empty()) {throw std::invalid_argument("--position-memory must not be empty");}
      result.position_memory = value;
    } else {
      throw std::invalid_argument("unknown option: " + key);
    }
  }
  if (result.interface.empty() || result.slaves.empty()) {
    throw std::invalid_argument("--interface and --slaves are required");
  }
  if (result.speed_rpm <= 0 || result.speed_rpm >= 60 ||
    result.timeout_s <= 0 || result.timeout_s > 3600)
  {
    throw std::invalid_argument("speed must be in (0, 60) RPM; timeout must be in (0, 3600] seconds");
  }
  chassis::ethercat::validate_selection({result.interface, result.slaves});
  if (result.torque_control || result.manual_brake) {
    if (std::find(result.slaves.begin(), result.slaves.end(), 4) != result.slaves.end()) {
      throw std::invalid_argument("slave 4 requires bounded position control; torque/manual calibration is not allowed");
    }
    if (!result.torque_limit_nm) {result.torque_limit_nm = 100.0;}
    result.current_feedback = true;
  }
  if (result.torque_limit_nm && *result.torque_limit_nm <= 0) {
    throw std::invalid_argument("--torque-limit-nm must be finite and > 0");
  }
  return result;
}

chassis::ethercat::Options master_options(
  const Options & options, std::function<void()> power_off)
{
  chassis::ethercat::Options result{options.interface, options.slaves};
  if (options.torque_control) {result.cst_slaves = options.slaves;}
  if (options.torque_limit_nm) {
    result.monitor_torque = true;
    result.torque_limit_nm = options.torque_limit_nm;
    result.fault_power_off = std::move(power_off);
  }
  return result;
}

template<typename T>
T read(
  chassis::ethercat::Master & master, std::uint16_t id,
  std::uint16_t index, std::uint8_t sub = 0)
{
  const auto result = master.node(id)->read(
    iswv::canopen::Object<T>{{index, sub},
      "control preflight"});
  if (!result) {throw std::runtime_error(result.error().message);}
  return result.value();
}

struct Axis
{
  std::uint16_t id;
  std::unique_ptr<chassis::YiyouMotor> motor;
  std::array<std::uint32_t, 4> identity{};
  double scale{0};
  std::uint32_t speed{0};
  std::uint32_t acceleration{0};
  std::int32_t step{0};
  std::uint32_t tolerance{0};
  std::int32_t target{0};
  bool held{false};
  bool torque_control{false};
  bool manual_brake{false};
  // Verified command state only; the drive exposes no physical brake confirmation here.
  bool brake_release_requested{false};
  double speed_limit_rpm{60};
  std::uint32_t rated_torque_mnm{0};
  std::int64_t torque_steps{0};
  double commanded_torque_nm{0};
  std::int16_t target_permille{0};
  std::uint32_t rated_current_ma{0};
  std::chrono::steady_clock::time_point next_current_feedback{};
  std::unique_ptr<chassis::PositionMemory> position_memory;
  bool position_valid{true};
  std::string position_error;
};

class CommandRefused : public std::runtime_error
{
public:
  using std::runtime_error::runtime_error;
};

std::string json_string(const std::string & text);

struct TorqueCommand
{
  double requested_nm;
  double commanded_nm;
  std::int16_t permille;
};

TorqueCommand torque_command(
  std::int64_t steps, std::uint32_t rated_mnm, double limit_nm)
{
  if (rated_mnm == 0 || !std::isfinite(limit_nm) || limit_nm <= 0) {
    throw std::runtime_error("torque control requires nonzero rated torque and a valid guard limit");
  }
  // EYOU PHU/RHU communication manual v1.11, 0x6071 (p236): nominal
  // target is rated motor torque permille. See docs/yiyou_torque_control.md
  // for the table/description wording discrepancy and quantization limits.
  const double requested = static_cast<double>(static_cast<long double>(steps) * torque_step_mnm / 1000);
  const long double raw = static_cast<long double>(steps) * torque_step_mnm * 1000 / rated_mnm;
  if (std::abs(requested) > limit_nm || std::abs(raw) > 1000) {
    throw CommandRefused("requested torque exceeds the guard or the rated +/-1000 permille command range");
  }
  const auto permille = static_cast<std::int16_t>(std::round(raw));
  const double commanded = static_cast<double>(permille) * rated_mnm / 1000000.0;
  if ((steps != 0 && permille == 0) || std::abs(commanded) > limit_nm) {
    throw CommandRefused("torque target cannot be represented within the configured limit");
  }
  return {requested, commanded, permille};
}

void torque_fields(std::ostream & record, const Axis & axis)
{
  if (axis.manual_brake) {
    record << ",\"brake_release_requested\":" << (axis.brake_release_requested ? "true" : "false");
  }
  if (!axis.torque_control) {return;}
  record << std::setprecision(17)
         << ",\"target_torque_nm\":" << static_cast<double>(axis.torque_steps * torque_step_mnm) / 1000
         << ",\"commanded_torque_nm\":" << axis.commanded_torque_nm
         << ",\"target_permille\":" << axis.target_permille
         << ",\"torque_resolution_nm\":" << axis.rated_torque_mnm / 1000000.0;
}

void position_fields(std::ostream & record, const Axis & axis, std::int32_t raw)
{
  const auto position = axis.position_memory && axis.position_valid ?
    axis.position_memory->logical_position(raw) : static_cast<std::int64_t>(raw);
  record << ",\"position\":" << position
         << ",\"raw_position\":" << raw
         << ",\"position_valid\":" << (axis.position_valid ? "true" : "false");
  if (!axis.position_valid) {
    record << ",\"position_error\":" << json_string(axis.position_error);
  }
}

void current_feedback(Axis & axis, const Options & options, const JsonOutput & output)
{
  if (!options.current_feedback ||
    std::chrono::steady_clock::now() < axis.next_current_feedback)
  {
    return;
  }
  const auto current = axis.motor->actual_current_permille();
  const auto position = axis.motor->actual_position();
  std::ostringstream record;
  record << "{\"event\":\"feedback\",\"slave\":" << axis.id;
  position_fields(record, axis, position);
  record << ",\"current_permille\":" << current
         << ",\"rated_current_ma\":" << axis.rated_current_ma
         << ",\"held\":" << (axis.held ? "true" : "false");
  if (options.torque_limit_nm) {
    const auto torque = axis.motor->actual_torque_newton_meter();
    if (!torque) {throw std::runtime_error("required torque feedback unavailable");}
    record << ",\"torque_nm\":" << std::setprecision(17) << *torque;
  }
  torque_fields(record, axis);
  record << '}';
  output.send(record.str());
  axis.next_current_feedback = std::chrono::steady_clock::now() + 200ms;
}

std::int64_t error_from_target(Axis & axis)
{
  return std::abs(static_cast<std::int64_t>(axis.motor->actual_position()) - axis.target);
}

bool monitor(Axis & axis)
{
  if (axis.position_memory && axis.position_valid) {
    (void)axis.position_memory->logical_position(axis.motor->actual_position());
  }
  bool reached = false;
  if (axis.torque_control) {
    axis.motor->check_current_mode();
  } else {
    reached = axis.motor->position_reached(axis.tolerance);
  }
  const auto velocity = axis.motor->actual_velocity();
  if (std::abs(static_cast<double>(velocity) * 60.0 / axis.scale) >= axis.speed_limit_rpm) {
    throw std::runtime_error("slave " + std::to_string(axis.id) + " reaches speed trip threshold " +
      std::to_string(axis.speed_limit_rpm) + " output RPM");
  }
  return reached && axis.motor->motion_stopped() &&
         (!axis.position_memory || std::abs(static_cast<double>(velocity) * 60.0 / axis.scale) < 0.1);
}

void monitor_all(std::vector<Axis> & axes)
{
  for (auto & axis : axes) {
    if (axis.manual_brake) {
      axis.motor->check_disabled();
      (void)axis.motor->actual_position();
    } else if (axis.held) {
      (void)monitor(axis);
    } else {
      // A passive UI must still observe a fault latched by the cyclic guard.
      (void)axis.motor->actual_position();
    }
  }
}

void wait_reached(
  Axis & moving, std::vector<Axis> & axes, const Options & options, const JsonOutput & output)
{
  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::duration<double>(options.timeout_s);
  while (true) {
    check_cancelled();
    bool reached = false;
    for (auto & axis : axes) {
      if (!axis.held) {continue;}
      const bool stationary = monitor(axis);
      current_feedback(axis, options, output);
      if (&axis == &moving) {reached = stationary;}
    }
    if (reached) {return;}
    if (std::chrono::steady_clock::now() >= deadline) {
      throw std::runtime_error(
              "slave " + std::to_string(moving.id) +
              " motion timed out; target=" + std::to_string(moving.target) +
              ", position error=" + std::to_string(error_from_target(moving)));
    }
    std::this_thread::sleep_for(20ms);
  }
}

void enable_hold(Axis & axis)
{
  check_cancelled();
  if (!axis.motor->motion_stopped()) {
    throw std::runtime_error("slave " + std::to_string(axis.id) + " must be stationary before enabling");
  }
  axis.target = axis.motor->actual_position();
  if (axis.position_memory && axis.position_valid) {
    (void)axis.position_memory->logical_position(axis.target);
  }
  axis.motor->enable_position(axis.speed, axis.acceleration, axis.acceleration);
  axis.held = true;
  check_cancelled();
  axis.motor->move_to_position(axis.target);
}

void done(const JsonOutput & output, const std::string & command, Axis * axis = nullptr)
{
  const auto position = axis == nullptr ? 0 : axis->motor->actual_position();
  std::ostringstream record;
  record << "{\"event\":\"done\",\"command\":\"" << command << '"';
  if (axis != nullptr) {
    record << ",\"slave\":" << static_cast<unsigned>(axis->id);
    position_fields(record, *axis, position);
    record << ",\"held\":" << (axis->held ? "true" : "false");
    torque_fields(record, *axis);
  }
  record << '}';
  output.send(record.str());
}

// No dependency is needed for the small outbound JSON protocol.
std::string json_string(const std::string & text)
{
  std::ostringstream out;
  out << '"';
  for (const unsigned char c : text) {
    if (c == '"' || c == '\\') {out << '\\' << c;} else if (c < 0x20) {
      out << "\\u" << std::hex << std::setw(4) << std::setfill('0') << static_cast<unsigned>(c);
    } else {out << c;}
  }
  out << '"';
  return out.str();
}

void ready(const JsonOutput & output, const std::vector<Axis> & axes)
{
  std::ostringstream record;
  record << "{\"event\":\"ready\",\"axes\":[";
  bool first = true;
  for (const auto & axis : axes) {
    if (!first) {record << ',';}
    first = false;
    record << "{\"slave\":" << static_cast<unsigned>(axis.id) << ",\"identity\":[";
    for (std::size_t i = 0; i < axis.identity.size(); ++i) {
      if (i != 0) {record << ',';}
      record << axis.identity[i];
    }
    record << "],\"scale\":" << std::setprecision(17) << axis.scale;
    position_fields(record, axis, axis.target);
    torque_fields(record, axis);
    record << '}';
  }
  record << "]}";
  output.send(record.str());
}

void move_fourth(
  Axis & axis, std::int64_t target, std::vector<Axis> & axes,
  const Options & options, const JsonOutput & output)
{
  wait_reached(axis, axes, options, output);
  axis.position_memory->queue_target(axis.motor->actual_position(), target);
  while (!axis.position_memory->poll()) {
    check_cancelled();
    monitor_all(axes);
    for (auto & other : axes) {current_feedback(other, options, output);}
    std::this_thread::sleep_for(20ms);
  }
  check_cancelled();
  monitor_all(axes);
  const auto raw_target = axis.position_memory->raw_target(target);
  axis.position_memory->mark_target(target);
  axis.target = raw_target;
  axis.motor->move_to_position(raw_target);
}

std::int64_t step_target(Axis & axis, std::int64_t sign)
{
  if (axis.position_memory) {
    if (!axis.position_valid) {
      throw CommandRefused("slave 4 position is untrusted; motion is disabled until an explicit zero reference: " + axis.position_error);
    }
    const auto actual = axis.position_memory->logical_position(axis.motor->actual_position());
    const auto upper = static_cast<std::int64_t>(std::floor(axis.scale * 1200.0 / 360.0));
    if ((sign < 0 && actual <= 0) || (sign > 0 && actual >= upper)) {
      throw CommandRefused("slave 4 software limit reached (0..1200 degrees); only inward motion is allowed");
    }
    const auto target = axis.position_memory->logical_position(axis.target) + sign * axis.step;
    return sign > 0 ? std::min(target, upper) : std::max(target, std::int64_t{0});
  }
  const auto target = static_cast<std::int64_t>(axis.target) + sign * axis.step;
  if (target < std::numeric_limits<std::int32_t>::min() ||
    target > std::numeric_limits<std::int32_t>::max())
  {
    throw std::out_of_range("step exceeds int32 position range");
  }
  return target;
}

bool command(
  const std::string & line, std::vector<Axis> & axes, const Options & options,
  bool & save_pending, const JsonOutput & output)
{
  std::istringstream input(line);
  std::vector<std::string> words;
  std::string word;
  while (input >> word) {words.push_back(word);}
  if (words.empty()) {throw std::invalid_argument("empty command");}
  const auto & action = words.front();
  if (action == "quit" && words.size() == 1) {return false;}
  if (options.manual_brake) {
    // There is no path from this mode to an enable or movement command.
    if (action == "brake" && words.size() == 1) {
      for (auto & axis : axes) {
        if (!axis.brake_release_requested) {continue;}
        axis.motor->engage_brake(check_cancelled);
        axis.brake_release_requested = false;
      }
      done(output, action);
      return true;
    }
    if (action == "release" && words.size() == 2) {
      const auto id = integer(words[1], 1, 199);
      auto selected = std::find_if(axes.begin(), axes.end(),
        [id](const Axis & axis) {return axis.id == id;});
      if (selected == axes.end()) {throw CommandRefused("slave is not in --slaves");}
      if (std::any_of(axes.begin(), axes.end(), [](const Axis & axis) {
          return axis.brake_release_requested;
        }))
      {
        throw CommandRefused("press space to engage the current brake before releasing an axis");
      }
      check_cancelled();
      monitor_all(axes);
      selected->motor->release_brake(check_cancelled);
      selected->brake_release_requested = true;
      done(output, action, &*selected);
      return true;
    }
    if (action != "zero" || words.size() != 2) {
      throw CommandRefused("manual mode only permits release, brake, zero and quit; motors stay disabled");
    }
  }
  if (action == "relax" && words.size() == 1 && options.torque_control) {
    // Remove force on every active axis before any per-axis disable wait.
    for (auto & axis : axes) {
      if (axis.held) {axis.motor->set_current_permille(0);}
    }
    for (auto & axis : axes) {
      axis.motor->stop();
      axis.held = false;
      axis.torque_steps = 0;
      axis.commanded_torque_nm = 0;
      axis.target_permille = 0;
    }
    done(output, action);
    return true;
  }
  if ((action == "hold" || action == "home") && words.size() == 1) {
    if (options.torque_control) {
      throw CommandRefused("hold/home is unavailable in torque mode; targets start at zero");
    }
    if (action == "home" &&
      std::any_of(axes.begin(), axes.end(), [](const Axis & a) {return a.id == 4;}))
    {
      throw CommandRefused("slave 4 home is disabled; use bounded jog steps with a valid position reference");
    }
    if (std::any_of(axes.begin(), axes.end(), [](const Axis & a) {return a.held;})) {
      throw std::logic_error("hold/home is only allowed once at startup");
    }
    for (auto & axis : axes) {
      enable_hold(axis);
    }
    for (auto & axis : axes) {
      if (action == "home") {
        check_cancelled();
        axis.target = 0;
        axis.motor->move_to_position(0);
      }
      wait_reached(axis, axes, options, output);
    }
    done(output, action);
    return true;
  }
  if ((action != "step" || words.size() != 3) &&
    (action != "zero" || words.size() != 2))
  {
    throw std::invalid_argument("invalid control command");
  }
  const auto id = integer(words[1], 1, 199);
  auto found = std::find_if(
    axes.begin(), axes.end(),
    [id](const Axis & axis) {return axis.id == id;});
  if (found == axes.end()) {throw std::invalid_argument("slave is not in --slaves");}
  auto & axis = *found;
  check_cancelled();
  if (action == "step") {
    const auto limit = options.torque_control ? 1 : 100;
    const auto sign = integer(words[2], -limit, limit);
    if (sign == 0) {throw std::invalid_argument("step increment must be nonzero");}
    if (options.torque_control) {
      const auto steps = axis.torque_steps + sign;
      const auto target = torque_command(steps, axis.rated_torque_mnm, options.torque_limit_nm.value());
      if (!axis.held) {
        if (!axis.motor->motion_stopped()) {
          throw CommandRefused("slave " + std::to_string(axis.id) + " must be stationary before enabling CST");
        }
        axis.motor->enable_cst();  // Verifies 0x6071/0x60B2 are zero before enabling.
        axis.held = true;
      }
      check_cancelled();
      axis.motor->set_current_permille(target.permille);
      axis.torque_steps = steps;
      axis.commanded_torque_nm = target.commanded_nm;
      axis.target_permille = target.permille;
      done(output, action, &axis);
      return true;
    }
    if (!axis.held) {axis.target = axis.motor->actual_position();}
    auto target = step_target(axis, sign);  // Refuse invalid targets before enabling.
    if (!axis.held) {
      enable_hold(axis);
      target = step_target(axis, sign);  // Use the freshly seeded hold position.
    }
    if (axis.position_memory) {
      move_fourth(axis, target, axes, options, output);
    } else {
      axis.target = static_cast<std::int32_t>(target);
      axis.motor->move_to_position(axis.target);
    }
    wait_reached(axis, axes, options, output);
  } else {
    if (options.manual_brake) {
      if (std::any_of(axes.begin(), axes.end(), [](const Axis & other) {
          return other.brake_release_requested;
        }))
      {
        throw CommandRefused("press space to engage all brakes before saving zero");
      }
      // 0x2130 saves all persistent parameters, including 0x2110/0x2111.
      // Restore and verify automatic/closed BEFORE any HM or EEPROM command.
      for (auto & other : axes) {other.motor->engage_brake(check_cancelled);}
      if (!axis.motor->motion_stopped()) {
        throw CommandRefused("wait until stationary before saving zero");
      }
    }
    if (options.torque_control) {
      if (std::any_of(axes.begin(), axes.end(), [&](const Axis & other) {
          return &other != &axis && other.held;
        }))
      {
        throw CommandRefused("press space to clear/disable all torque axes before saving zero");
      }
      axis.motor->stop();
      axis.held = false;
      axis.torque_steps = 0;
      axis.commanded_torque_nm = 0;
      axis.target_permille = 0;
      if (!axis.motor->motion_stopped()) {
        throw CommandRefused("torque cleared and axis disabled; wait until stationary before saving zero");
      }
    }
    const bool resume_hold = axis.held;
    if (resume_hold) {
      wait_reached(axis, axes, options, output);
    } else {
      axis.target = axis.motor->actual_position();
    }
    axis.motor->stop();
    axis.held = false;
    if (error_from_target(axis) > axis.tolerance) {
      throw std::runtime_error("position drifted after disable; zero was not changed");
    }
    check_cancelled();
    if (axis.position_memory) {
      axis.position_memory->invalidate();
      axis.position_valid = false;
    }
    save_pending = true;  // A failed SDO may still have reached the EEPROM.
    axis.motor->save_zero_position(axis.tolerance);
    std::this_thread::sleep_for(save_wait);  // Do not cut power midway through saving.
    save_pending = false;
    check_cancelled();
    axis.target = 0;
    if (error_from_target(axis) > axis.tolerance) {
      throw std::runtime_error("position drifted while saving zero");
    }
    if (axis.position_memory) {
      const auto raw = axis.motor->actual_position();
      axis.position_memory->set_reference(raw, raw);
      axis.position_valid = true;
      axis.position_error.clear();
    }
    if (resume_hold) {
      enable_hold(axis);
      if (axis.position_memory) {
        move_fourth(axis, 0, axes, options, output);
      } else {
        axis.target = 0;
        axis.motor->move_to_position(0);
      }
      wait_reached(axis, axes, options, output);
    }
  }
  done(output, action, &axis);
  return true;
}

void serve(
  std::vector<Axis> & axes, const Options & options, bool & save_pending, const JsonOutput & output)
{
  std::string pending;
  while (!stop_requested) {
    monitor_all(axes);
    for (auto & axis : axes) {
      current_feedback(axis, options, output);
    }
    pollfd input{STDIN_FILENO, POLLIN, 0};
    const auto count = poll(&input, 1, 20);
    if (count < 0) {
      if (errno == EINTR) {continue;}
      throw std::runtime_error("poll stdin failed");
    }
    if (count == 0) {continue;}
    if (input.revents & (POLLERR | POLLNVAL)) {throw std::runtime_error("stdin failed");}
    char buffer[256];
    const auto received = ::read(STDIN_FILENO, buffer, sizeof(buffer));
    if (received == 0) {return;}
    if (received < 0) {
      if (errno == EINTR) {continue;}
      throw std::runtime_error("read stdin failed");
    }
    pending.append(buffer, static_cast<std::size_t>(received));
    if (pending.size() > 1024) {throw std::invalid_argument("control input too long");}
    auto end = pending.find('\n');
    while (end != std::string::npos) {
      const auto line = pending.substr(0, end);
      pending.erase(0, end + 1);
      try {
        if (!command(line, axes, options, save_pending, output)) {return;}
      } catch (const CommandRefused & error) {
        output.send("{\"event\":\"rejected\",\"message\":" + json_string(error.what()) + "}");
      }
      end = pending.find('\n');
    }
  }
}
}  // namespace

int main(int argc, char ** argv)
{
  if (argc == 2 && std::string(argv[1]) == "--help") {usage(); return 0;}
  Options options;
  try {
    options = parse(argc, argv);
  } catch (const std::exception & error) {
    std::cerr << "Arguments: " << error.what() << '\n';
    return 2;
  }
  std::signal(SIGINT, request_stop);
  std::signal(SIGTERM, request_stop);
  std::signal(SIGHUP, request_stop);
  std::signal(SIGPIPE, SIG_IGN);
  int result = 0;
  bool save_pending = false;
  std::unique_ptr<JsonOutput> output;
  std::mutex power_mutex;
  bool power_cut = false;
  std::shared_ptr<chassis::BxiPciTransport> transport;
  const auto cut_power = [&] {
      std::lock_guard<std::mutex> guard(power_mutex);
      if (transport && !power_cut) {
        const auto off = transport->set_motor_power(false);
        if (!off) {throw std::runtime_error(off.error().message);}
        power_cut = true;
      }
    };
  std::unique_ptr<chassis::ethercat::Master> master;
  std::vector<Axis> axes;
  try {
    output = std::make_unique<JsonOutput>();
    check_cancelled();
    transport = std::make_shared<chassis::BxiPciTransport>(options.power_bus);
    const auto powered = transport->set_motor_power(true);
    if (!powered) {throw std::runtime_error(powered.error().message);}
    wait_power();
    master = std::make_unique<chassis::ethercat::Master>(
      master_options(options, cut_power));
    for (const auto id : options.slaves) {
      check_cancelled();
      Axis axis{};
      axis.id = id;
      axis.torque_control = options.torque_control;
      axis.manual_brake = options.manual_brake;
      axis.speed_limit_rpm = options.torque_control ? options.speed_rpm : 60.0;
      axis.motor = std::make_unique<chassis::YiyouMotor>(*master, id);
      axis.scale = axis.motor->configure_rpm_units();
      axis.speed = static_cast<std::uint32_t>(axis.motor->velocity_from_rpm(options.speed_rpm));
      axis.acceleration = static_cast<std::uint32_t>(axis.motor->velocity_from_rpm(10));
      axis.step = axis.motor->position_from_degrees(jog_step_degrees);
      axis.tolerance = std::min(100U, static_cast<std::uint32_t>(axis.step / 4));
      axis.target = axis.motor->actual_position();
      if (options.torque_control) {
        axis.rated_torque_mnm = axis.motor->rated_torque_millinewton_meter().value_or(0);
        (void)torque_command(1, axis.rated_torque_mnm, options.torque_limit_nm.value());
      }
      for (std::uint8_t sub = 1; sub <= axis.identity.size(); ++sub) {
        axis.identity[sub - 1] = read<std::uint32_t>(*master, id, 0x1018, sub);
      }
      const auto status = read<std::uint16_t>(*master, id, 0x6041);
      const auto state = iswv::cia402::decode_state(status);
      if ((state != iswv::cia402::DriveState::switch_on_disabled &&
        state != iswv::cia402::DriveState::ready_to_switch_on &&
        state != iswv::cia402::DriveState::switched_on) ||
        !axis.motor->motion_stopped())
      {
        std::ostringstream message;
        message << "slave " << id << " must be disabled and stationary; state="
                << iswv::cia402::state_name(state) << ", 0x6041=0x" << std::hex << status;
        throw std::runtime_error(message.str());
      }
      if (options.current_feedback) {
        // Some firmware omits rated current. Keep the raw permille reading usable.
        const auto rated = master->node(id)->read(
          iswv::canopen::Object<std::uint32_t>{{0x6075, 0}, "rated current (mA)"});
        if (rated) {axis.rated_current_ma = rated.value();}
      }
      if (options.manual_brake) {axis.motor->engage_brake(check_cancelled);}
      if (id == 4) {
        axis.position_memory = std::make_unique<chassis::PositionMemory>(
          options.position_memory, options.interface, id, axis.identity, axis.scale);
        try {
          (void)axis.position_memory->restore(axis.target);
        } catch (const std::exception & error) {
          axis.position_valid = false;
          axis.position_error = error.what();
        }
      }
      axes.push_back(std::move(axis));
    }
    ready(*output, axes);
    serve(axes, options, save_pending, *output);
    if (options.manual_brake) {
      for (auto & axis : axes) {
        if (!axis.brake_release_requested) {continue;}
        axis.motor->engage_brake();
        axis.brake_release_requested = false;
      }
    }
  } catch (const std::exception & error) {
    result = 1;
    if (options.torque_limit_nm) {
      // Power first: neither frontend output nor EEPROM/stop waits may delay a trip.
      try {cut_power();} catch (const std::exception & off_error) {
        std::cerr << "Shared power-off failed: " << off_error.what() << '\n';
      }
    }
    std::cerr << "Calibration failure: " << error.what() << '\n';
    try {
      if (output) {
        output->send("{\"event\":\"error\",\"message\":" + json_string(error.what()) + "}");
      } else {
        std::cerr << error.what() << '\n';
      }
    } catch (const std::exception &) {
      // A closed frontend pipe must not bypass motor stop or the EEPROM save wait.
      std::cerr << error.what() << '\n';
    }
  }
  if (save_pending) {std::this_thread::sleep_for(save_wait);}
  for (auto & axis : axes) {
    try {
      axis.motor->stop();
    } catch (const std::exception & error) {
      std::cerr << "Stop slave " << static_cast<unsigned>(axis.id) << ": " << error.what() << '\n';
      result = 1;
    }
  }
  try {cut_power();} catch (const std::exception & error) {
    std::cerr << error.what() << '\n'; result = 1;
  }
  // A stalled filesystem writer may delay PositionMemory destruction. Shared
  // power must already be off before waiting for any outstanding disk operation.
  axes.clear();
  master.reset();
  return result;
}
