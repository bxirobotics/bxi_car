// Exercise the real command handlers with a simulated CoE object dictionary.
#define YIYOU_CONTROL_TEST_FAST_WAIT
#define main yiyou_control_program_main
#include "../tools/arm_yiyou_control.cpp"
#undef main

#include <filesystem>
#include <functional>
#include <map>
#include <yaml-cpp/yaml.h>
#include "motor/yiyou/torque_protection.h"

namespace
{
void require(bool condition, const char * message)
{
  if (!condition) {throw std::runtime_error(message);}
}

template<typename Exception = std::exception, typename Function>
void rejects(Function function)
{
  try {
    function();
  } catch (const Exception &) {
    return;
  }
  throw std::runtime_error("expected rejection");
}

// The command handlers expect a connected frontend, independently of whether
// ctest itself was launched from a terminal, /dev/null, or an exhausted SSH pipe.
class LiveFrontendInput
{
public:
  LiveFrontendInput()
  {
    saved_stdin_ = ::dup(STDIN_FILENO);
    if (saved_stdin_ < 0) {throw std::runtime_error("cannot save test stdin");}
    int connection[2];
    if (::pipe(connection) != 0) {
      ::close(saved_stdin_);
      throw std::runtime_error("cannot create test frontend pipe");
    }
    if (::dup2(connection[0], STDIN_FILENO) < 0) {
      ::close(connection[0]);
      ::close(connection[1]);
      ::close(saved_stdin_);
      throw std::runtime_error("cannot connect test frontend stdin");
    }
    ::close(connection[0]);
    writer_ = connection[1];
  }

  ~LiveFrontendInput()
  {
    (void)::dup2(saved_stdin_, STDIN_FILENO);
    ::close(saved_stdin_);
    ::close(writer_);
  }

  LiveFrontendInput(const LiveFrontendInput &) = delete;
  LiveFrontendInput & operator=(const LiveFrontendInput &) = delete;

private:
  int saved_stdin_{-1};
  int writer_{-1};
};

class FakeNode : public chassis::ethercat::Node
{
public:
  explicit FakeNode(std::uint16_t id = 4) : id_(id)
  {
    put<std::uint16_t>(0x6041, 0x40);
    put<std::uint8_t>(0x2100, 1);
    put<std::int8_t>(0x6060, 0);
    put<std::int8_t>(0x6061, 0);
    put<std::int8_t>(0x2707, 1);
    put<std::int32_t>(0x6064, 12000);
    put<std::int32_t>(0x606C, 0);
    put<std::int16_t>(0x6071, 0);
    put<std::int16_t>(0x60B2, 0);
    put<std::uint8_t>(0x2110, 1);
    put<std::int8_t>(0x2111, 1);
    put<std::uint16_t>(0x2113, 0);
    put<std::uint16_t>(0x2114, 0);
  }

  std::uint16_t id() const noexcept override {return id_;}

  template<typename T>
  void put(std::uint16_t index, T value)
  {
    const auto bytes = iswv::canopen::encode_little_endian(value);
    values[{index, 0}] = {bytes.begin(), bytes.end()};
  }

  iswv::Result<iswv::canopen::SdoResponse> upload_blocking(
    iswv::canopen::ObjectAddress object, iswv::canopen::SdoOptions) override
  {
    if (fault && !fault->empty()) {
      return iswv::Result<iswv::canopen::SdoResponse>::failure(
        iswv::ErrorCode::transport_error, fault->message());
    }
    const auto found = values.find(object);
    if (found == values.end()) {
      return iswv::Result<iswv::canopen::SdoResponse>::failure(
        iswv::make_sdo_abort(0x06020000, "unknown test object"));
    }
    iswv::canopen::SdoResponse response;
    response.object = object;
    response.size = static_cast<std::uint8_t>(found->second.size());
    std::copy(found->second.begin(), found->second.end(), response.data.begin());
    return iswv::Result<iswv::canopen::SdoResponse>::success(response);
  }

  iswv::Result<void> download_blocking(
    iswv::canopen::ObjectAddress object, const std::uint8_t * data, std::size_t size,
    iswv::canopen::SdoOptions) override
  {
    if (fault && !fault->empty()) {
      return iswv::Result<void>::failure(iswv::ErrorCode::transport_error, fault->message());
    }
    std::uint32_t value = 0;
    for (std::size_t i = 0; i < size; ++i) {value |= std::uint32_t{data[i]} << (8U * i);}
    if (before_write) {before_write(object.index, value);}
    if (fault && !fault->empty()) {
      return iswv::Result<void>::failure(iswv::ErrorCode::transport_error, fault->message());
    }
    values[object] = {data, data + size};
    if (object.index == 0x6060) {put<std::int8_t>(0x6061, static_cast<std::int8_t>(value));}
    if (object.index == 0x607A) {target = static_cast<std::int32_t>(value);}
    if (object.index == 0x6071) {target_current = static_cast<std::int16_t>(value);}
    if (object.index == 0x2130) {++eeprom_writes;}
    if (object.index == 0x6040) {
      if ((value & 0x0f) == 15) {++enable_writes;}
      require((value & 0x80) == 0, "calibration must never automatically reset a drive fault");
      std::uint16_t status = 0x40;
      switch (value & 0x0f) {
        case 6: status = 0x21; break;
        case 7: status = 0x23; break;
        case 15: status = (value & 0x10) ? 0x1427 : 0x0427; break;
        default: break;
      }
      if (value == 0x3f) {
        ++triggers;
        put<std::int32_t>(0x6064, target);
      }
      if (value == 0x10) {put<std::int32_t>(0x6064, 0);}
      put<std::uint16_t>(0x6041, status);
    }
    return iswv::Result<void>::success();
  }

  std::function<void(std::uint16_t, std::uint32_t)> before_write;
  chassis::ethercat::CyclicFault * fault{nullptr};
  std::int32_t target{};
  std::int16_t target_current{};
  unsigned eeprom_writes{};
  unsigned triggers{};
  unsigned enable_writes{};
  std::map<iswv::canopen::ObjectAddress, std::vector<std::uint8_t>> values;
  std::uint16_t id_;
};

constexpr std::array<std::uint32_t, 4> identity{4247, 65537, 64, 65553};
constexpr double scale = 36000;

struct Fixture
{
  Fixture(const std::filesystem::path & path, bool seeded)
  : node(std::make_shared<FakeNode>())
  {
    options.position_memory = path.string();
    options.timeout_s = 0.2;
    if (seeded) {
      chassis::PositionMemory seed(path.string(), "eth0", 4, identity, scale);
      seed.set_reference(120000, 120000);
    }
    Axis axis{};
    axis.id = 4;
    axis.identity = identity;
    axis.scale = scale;
    axis.speed = 1200;
    axis.acceleration = 2400;
    axis.step = 10000;
    axis.tolerance = 2;
    axis.target = 12000;
    axis.motor = std::make_unique<chassis::YiyouMotor>(node);
    axis.position_memory = std::make_unique<chassis::PositionMemory>(
      path.string(), "eth0", 4, identity, scale);
    try {
      (void)axis.position_memory->restore(axis.target);
    } catch (const std::exception & error) {
      axis.position_valid = false;
      axis.position_error = error.what();
    }
    axes.push_back(std::move(axis));
  }

  void run(const std::string & text, const JsonOutput & output)
  {
    require(command(text, axes, options, save_pending, output), "unexpected quit");
  }

  std::shared_ptr<FakeNode> node;
  Options options;
  bool save_pending{false};
  std::vector<Axis> axes;
};

void restored_motion(const std::filesystem::path & path, const JsonOutput & output)
{
  Fixture fixture(path, true);
  auto & axis = fixture.axes.front();
  require(axis.position_valid, "1200 degree record restores from raw 120 degrees");
  std::ostringstream json;
  position_fields(json, axis, 12000);
  require(json.str().find("\"position\":120000") != std::string::npos,
    "debug protocol exposes the logical position");
  require(json.str().find("\"raw_position\":12000") != std::string::npos,
    "debug protocol retains raw feedback separately");
  fixture.node->before_write = [&](std::uint16_t index, std::uint32_t value) {
      if (index == 0x6040 && value == 0x3f) {
        const auto record = YAML::LoadFile(path.string());
        const auto logical_target = fixture.node->target + 108000;
        require(record["valid"].as<bool>(), "trigger requires a valid durable record");
        require(record["lower"].as<std::int64_t>() <= logical_target &&
          record["upper"].as<std::int64_t>() >= logical_target,
          "new target must be durable before the trigger");
      }
    };
  rejects<CommandRefused>([&]() {fixture.run("step 4 1", output);});
  require(fixture.node->enable_writes == 0, "upper limit must be rejected before the first enable");
  fixture.run("hold", output);
  require(fixture.node->target == 12000, "startup holds current raw position");
  const auto triggers = fixture.node->triggers;
  rejects<CommandRefused>([&]() {fixture.run("step 4 1", output);});
  require(fixture.node->triggers == triggers, "upper limit does not trigger motion");
  fixture.run("step 4 -1", output);
  require(fixture.node->target == 2000, "logical 1100 degrees converts to raw 20 degrees");
  require(axis.position_memory->logical_position(2000) == 110000, "logical feedback follows inward motion");
  fixture.run("step 4 1", output);
  require(fixture.node->target == 12000, "inward and outward steps retain the fixed offset");
  fixture.node->put<std::int32_t>(0x606C, 120);  // 0.2 RPM despite a set stop flag.
  const auto before_moving = fixture.node->triggers;
  rejects([&]() {fixture.run("step 4 -1", output);});
  require(fixture.node->triggers == before_moving, "nonzero velocity blocks a replacement target");
  fixture.node->put<std::int32_t>(0x606C, 0);
  std::filesystem::remove(path);
  std::filesystem::create_directory(path);
  rejects([&]() {fixture.run("step 4 -1", output);});
  require(fixture.node->triggers == before_moving, "failed durable write blocks motion");
}

void untrusted_zero(const std::filesystem::path & path, const JsonOutput & output)
{
  Fixture fixture(path, false);
  auto & axis = fixture.axes.front();
  require(!axis.position_valid, "missing state does not silently adopt raw position");
  rejects<CommandRefused>([&]() {fixture.run("home", output);});
  require(fixture.node->triggers == 0, "home refusal precedes enabling");
  rejects<CommandRefused>([&]() {fixture.run("step 4 -1", output);});
  require(fixture.node->enable_writes == 0, "untrusted position must be rejected before the first enable");
  fixture.run("hold", output);
  rejects<CommandRefused>([&]() {fixture.run("step 4 -1", output);});
  fixture.node->before_write = [&](std::uint16_t index, std::uint32_t value) {
      if ((index == 0x6040 && value == 0x10) || index == 0x2130) {
        require(!YAML::LoadFile(path.string())["valid"].as<bool>(),
          "zero must invalidate the record before changing hardware");
      }
    };
  fixture.run("zero 4", output);
  require(axis.position_valid && axis.position_memory->logical_position(0) == 0,
    "explicit zero repairs missing state");
  rejects<CommandRefused>([&]() {fixture.run("step 4 -1", output);});
  fixture.run("step 4 1", output);
  require(fixture.node->target == 10000, "valid zero permits bounded positive steps");
  fixture.node->before_write = [&](std::uint16_t index, std::uint32_t) {
      if (index == 0x2130) {throw std::runtime_error("injected EEPROM save failure");}
    };
  rejects([&]() {fixture.run("zero 4", output);});
  require(!YAML::LoadFile(path.string())["valid"].as<bool>(), "failed zero leaves durable record invalid");
}
}  // namespace

void one_degree_jog(const JsonOutput & output)
{
  std::array<std::shared_ptr<FakeNode>, 3> nodes;
  std::vector<Axis> axes;
  Options options;
  options.timeout_s = 0.2;
  bool save_pending = false;
  for (std::uint16_t id = 1; id <= 3; ++id) {
    auto node = std::make_shared<FakeNode>(id);
    node->put<std::int32_t>(0x2025, 36000);
    node->put<std::uint16_t>(0x26A2, id);
    node->put<std::uint16_t>(0x26A3, 1);
    Axis axis{};
    axis.id = id;
    axis.motor = std::make_unique<chassis::YiyouMotor>(node);
    axis.scale = axis.motor->configure_rpm_units();
    axis.speed = static_cast<std::uint32_t>(axis.motor->velocity_from_rpm(1));
    axis.acceleration = static_cast<std::uint32_t>(axis.motor->velocity_from_rpm(10));
    axis.step = axis.motor->position_from_degrees(jog_step_degrees);
    axis.tolerance = 2;
    nodes[id - 1] = node;
    axes.push_back(std::move(axis));
  }
  monitor_all(axes);
  for (const auto & node : nodes) {
    require(node->enable_writes == 0 && node->triggers == 0,
      "passive startup and monitoring must not enable or move any axis");
  }
  for (const auto * invalid : {"step 1 0", "step 1 -101", "step 1 101", "step 4 -1"}) {
    rejects<std::invalid_argument>([&]() {command(invalid, axes, options, save_pending, output);});
  }
  nodes[0]->put<std::int32_t>(0x6064, std::numeric_limits<std::int32_t>::min());
  rejects<std::out_of_range>([&]() {command("step 1 -1", axes, options, save_pending, output);});
  require(nodes[0]->enable_writes == 0, "invalid first steps must fail before enabling");
  nodes[0]->put<std::int32_t>(0x6064, 12000);
  nodes[0]->put<std::int8_t>(0x2707, 0);
  rejects([&]() {command("step 1 -1", axes, options, save_pending, output);});
  require(nodes[0]->enable_writes == 0, "a moving disabled axis must not be enabled");
  nodes[0]->put<std::int8_t>(0x2707, 1);
  for (std::uint16_t id = 1; id <= 3; ++id) {
    // The mechanism may have shifted since the startup position was read.
    const auto start = 12000 + 1000 * id;
    nodes[id - 1]->put<std::int32_t>(0x6064, start);
    std::array<unsigned, 3> old_triggers{};
    for (std::size_t i = 0; i < nodes.size(); ++i) {old_triggers[i] = nodes[i]->triggers;}
    command("step " + std::to_string(id) + " -1", axes, options, save_pending, output);
    require(nodes[id - 1]->target == start - 100 * id,
      "first downward jog must retreat exactly one output degree from fresh feedback");
    for (std::size_t i = 0; i < nodes.size(); ++i) {
      if (i != id - 1U) {require(nodes[i]->triggers == old_triggers[i], "jog must only move the selected axis");}
      require(axes[i].held == (i < id), "only previously jogged axes may remain holding");
    }
    command("step " + std::to_string(id) + " 1", axes, options, save_pending, output);
    require(nodes[id - 1]->target == start, "one positive jog must return by one output degree");
    command("step " + std::to_string(id) + " -100", axes, options, save_pending, output);
    require(nodes[id - 1]->target == start - 10000 * id,
      "manual downward jog must move exactly 100 output degrees");
    command("step " + std::to_string(id) + " 100", axes, options, save_pending, output);
    require(nodes[id - 1]->target == start,
      "manual upward jog must return by exactly 100 output degrees");
  }
  rejects<std::invalid_argument>([&]() {
    command("step 4 1", axes, options, save_pending, output);
  });
  nodes[0]->put<std::uint16_t>(0x6041, 0x071f);
  try {
    axes[0].motor->check_position_mode();
    throw std::logic_error("fault status was accepted");
  } catch (const std::runtime_error & error) {
    const std::string message = error.what();
    require(message.find("slave 1 PP state=fault reaction active") != std::string::npos &&
      message.find("following_error=0, internal_limit=0") != std::string::npos,
      "drive fault diagnostics must identify the axis and actual status bits");
  }
}

void passive_zero_and_fault(const std::filesystem::path & path, const JsonOutput & output)
{
  bool power_cut = false;
  chassis::ethercat::CyclicFault fault([&] {power_cut = true;});
  Fixture fixture(path, false);
  fixture.run("zero 4", output);
  require(fixture.axes.front().position_valid && !fixture.axes.front().held &&
    fixture.node->enable_writes == 0 && fixture.node->triggers == 0,
    "zeroing a passive axis must leave it disabled without a PP motion trigger");
  fixture.node->fault = &fault;
  fault.latch(chassis::ethercat::torque_fault_message(4, 100, 1001, 100000U));
  require(power_cut, "idle torque fault still cuts shared power");
  rejects([&] {monitor_all(fixture.axes);});
  rejects([&] {fixture.run("step 4 1", output);});
  require(fixture.node->enable_writes == 0,
    "latched idle fault must surface to the UI and prevent first-step enabling");
}

void fault_during_first_retreat(const std::filesystem::path & path, const JsonOutput & output)
{
  bool power_cut = false;
  chassis::ethercat::CyclicFault fault([&] {power_cut = true;});
  Fixture fixture(path, true);
  fixture.node->fault = &fault;
  fixture.node->before_write = [&](std::uint16_t index, std::uint32_t value) {
      if (index == 0x6040 && (value & 0x0f) == 15) {
        fault.latch(chassis::ethercat::torque_fault_message(4, 100, -1001, 100000U));
      }
    };
  rejects([&] {fixture.run("step 4 -1", output);});
  require(power_cut && !fixture.axes.front().held && fixture.node->triggers == 0,
    "a torque trip during initial enable must stop before a retreat trigger");
  rejects([&] {fixture.run("step 4 -1", output);});
  require(fixture.node->triggers == 0, "a latched startup fault must not retry the retreat");
}

void torque_protection_options()
{
  auto parse_limit = [](const std::string & limit) {
      std::vector<std::string> words{
        "arm_yiyou_control", "--interface", "eth0", "--slaves", "1,2,3",
        "--torque-limit-nm", limit};
      std::vector<char *> arguments;
      for (auto & word : words) {arguments.push_back(word.data());}
      return parse(static_cast<int>(arguments.size()), arguments.data());
    };
  for (const auto * value : {"0", "-1", "nan", "inf", "-inf", "100Nm"}) {
    rejects<std::invalid_argument>([&]() {parse_limit(value);});
  }
  const auto options = parse_limit("100");
  require(options.torque_limit_nm == 100.0, "configured torque limit is retained");
  bool power_cut = false;
  auto configuration = master_options(options, [&]() {power_cut = true;});
  require(configuration.monitor_torque, "torque protection requires PDO torque monitoring");
  require(configuration.torque_limit_nm == 100.0, "backend forwards limit to cyclic guard");
  chassis::ethercat::validate_selection(configuration);
  configuration.fault_power_off();
  require(power_cut, "cyclic fault callback reaches shared power action");
  Options old_options;
  old_options.interface = "eth0";
  old_options.slaves = {1};
  const auto unprotected = master_options(old_options, [] {});
  require(!unprotected.monitor_torque && !unprotected.torque_limit_nm &&
    !unprotected.fault_power_off, "unconfigured callers retain their existing behavior");
}

void torque_trip_during_zero_prevents_reenable(
  const std::filesystem::path & path, const JsonOutput & output)
{
  bool power_cut = false;
  chassis::ethercat::CyclicFault fault([&] {power_cut = true;});
  Fixture fixture(path, true);
  fixture.run("hold", output);
  fixture.node->fault = &fault;
  const auto triggers_before = fixture.node->triggers;
  bool save_entered = false;
  fixture.node->before_write = [&](std::uint16_t index, std::uint32_t) {
      if (index == 0x2130) {
        save_entered = true;
        fault.latch(chassis::ethercat::torque_fault_message(4, 100, -1001, 100000U));
        require(power_cut, "shared power is cut before the blocked EEPROM call returns");
      }
    };
  rejects([&] {fixture.run("zero 4", output);});
  require(save_entered && power_cut && fixture.save_pending,
    "zero failure retains pending-save state but cannot delay already-issued power cut");
  require(!fixture.axes.front().held && fixture.node->triggers == triggers_before,
    "failed zero must not re-enable or issue a replacement target");
  require(!YAML::LoadFile(path.string())["valid"].as<bool>(), "interrupted zero is not recorded as valid");
  rejects([&] {fixture.run("step 4 1", output);});
}

struct TorqueFixture
{
  TorqueFixture()
  {
    options.interface = "eth0";
    options.slaves = {1, 2, 3};
    options.torque_control = true;
    options.torque_limit_nm = 100;
    for (std::uint16_t id = 1; id <= 3; ++id) {
      auto node = std::make_shared<FakeNode>(id);
      node->put<std::int16_t>(0x6071, 123);
      node->put<std::int16_t>(0x60B2, -20);
      Axis axis{};
      axis.id = id;
      axis.motor = std::make_unique<chassis::YiyouMotor>(node);
      axis.scale = 36000;
      axis.tolerance = 2;
      axis.torque_control = true;
      axis.rated_torque_mnm = id == 3 ? 1800 : 3600;
      axis.speed_limit_rpm = 1;
      nodes.push_back(node);
      axes.push_back(std::move(axis));
    }
  }

  void run(const std::string & text, const JsonOutput & output)
  {
    require(command(text, axes, options, save_pending, output), "unexpected quit");
  }

  Options options;
  bool save_pending{false};
  std::vector<std::shared_ptr<FakeNode>> nodes;
  std::vector<Axis> axes;
};

void torque_mode_options_and_quantization()
{
  std::vector<std::string> words{
    "arm_yiyou_control", "--interface", "eth0", "--slaves", "1,2,3",
    "--control-mode", "torque"};
  std::vector<char *> args;
  for (auto & word : words) {args.push_back(word.data());}
  const auto options = parse(static_cast<int>(args.size()), args.data());
  require(options.torque_control && options.current_feedback && options.torque_limit_nm == 100,
    "torque mode always requires feedback and defaults to the existing 100Nm guard");
  const auto master = master_options(options, [] {});
  chassis::ethercat::validate_selection(master);
  require(master.cst_slaves == options.slaves && !master.cst_slave,
    "all selected torque axes use CST mapping without a legacy single-CST selection");
  words[5] = "1,4";
  args.clear();
  for (auto & word : words) {args.push_back(word.data());}
  rejects<std::invalid_argument>([&] {parse(static_cast<int>(args.size()), args.data());});

  const auto first = torque_command(1, 3600, 100);
  require(first.requested_nm == 0.05 && first.permille == 14 &&
    std::abs(first.commanded_nm - 0.0504) < 1e-12, "0.05Nm keeps requested and quantized values separate");
  require(torque_command(-1, 3600, 100).permille == -14, "negative torque preserves sign");
  require(torque_command(5, 3600, 100).permille == 69,
    "five increments recompute from 0.25Nm rather than accumulating five rounded steps");
  require(torque_command(1, 1800, 100).permille == 28, "each axis uses its own rated torque");
  require(torque_command(0, 3600, 100).permille == 0, "zero target remains exactly zero");
  rejects([&] {torque_command(1, 0, 100);});
  rejects<CommandRefused>([&] {torque_command(1, 3600, 0.05);});
  rejects<CommandRefused>([&] {torque_command(73, 3600, 100);});
  rejects<CommandRefused>([&] {torque_command(-73, 3600, 100);});
  rejects<CommandRefused>([&] {torque_command(1, 1000000, 100);});
}

void torque_keys_and_zero(const JsonOutput & output)
{
  TorqueFixture fixture;
  monitor_all(fixture.axes);
  for (const auto & axis : fixture.axes) {
    require(!axis.held && axis.torque_steps == 0, "torque startup leaves every axis disabled at zero request");
  }
  rejects<CommandRefused>([&] {fixture.run("hold", output);});
  rejects<CommandRefused>([&] {fixture.run("home", output);});
  for (const auto * invalid : {"step 1 100", "step 1 -100"}) {
    rejects<std::invalid_argument>([&] {fixture.run(invalid, output);});
  }
  require(fixture.nodes[0]->enable_writes == 0,
    "position-sized increments must be refused before torque mode enables");
  bool first_enable = true;
  fixture.nodes[0]->before_write = [&](std::uint16_t index, std::uint32_t value) {
      if (index == 0x6040 && (value & 0x0f) == 15 && first_enable) {
        first_enable = false;
        const auto current = fixture.nodes[0]->read(iswv::canopen::Object<std::int16_t>{{0x6071, 0}, "target"});
        require(current && current.value() == 0, "CST must clear stale target before first enable");
        const auto offset = fixture.nodes[0]->read(iswv::canopen::Object<std::int16_t>{{0x60B2, 0}, "offset"});
        require(offset && offset.value() == 0, "CST must clear stale offset before enabling");
      }
    };
  for (int i = 0; i < 5; ++i) {fixture.run("step 1 1", output);}
  require(fixture.axes[0].torque_steps == 5 && fixture.nodes[0]->target_current == 69,
    "five up keys produce the quantized cumulative 0.25Nm target");
  require(fixture.nodes[1]->enable_writes == 0 && fixture.nodes[2]->enable_writes == 0,
    "torque keys must not enable other selected axes");
  fixture.run("step 2 -1", output);
  require(fixture.nodes[1]->target_current == -14 && fixture.nodes[0]->target_current == 69,
    "changing axis preserves the earlier target and adjusts only the selected axis");
  rejects<CommandRefused>([&] {fixture.run("zero 1", output);});
  require(fixture.nodes[0]->eeprom_writes == 0, "zero cannot block monitoring while other torque axes remain enabled");
  fixture.nodes[0]->before_write = [&](std::uint16_t index, std::uint32_t value) {
      if (index == 0x6040 && value == 0) {
        require(fixture.nodes[0]->target_current == 0 && fixture.nodes[1]->target_current == 0,
          "relax must clear all active-axis torque targets before the first disable wait");
      }
    };
  fixture.run("relax", output);
  for (std::size_t i = 0; i < fixture.axes.size(); ++i) {
    require(!fixture.axes[i].held && fixture.axes[i].torque_steps == 0 &&
      fixture.nodes[i]->target_current == 0 && fixture.nodes[i]->triggers == 0,
      "space clears targets and disables every active axis without a PP target");
  }
  fixture.run("step 3 -1", output);
  require(fixture.nodes[2]->target_current == -28, "third axis starts again from zero using its own rating");
  fixture.nodes[2]->put<std::int8_t>(0x2707, 0);
  rejects<CommandRefused>([&] {fixture.run("zero 3", output);});
  require(!fixture.axes[2].held && fixture.nodes[2]->target_current == 0 &&
    fixture.nodes[2]->eeprom_writes == 0, "moving CST zero clears force but refuses EEPROM writes");
  fixture.nodes[2]->put<std::int8_t>(0x2707, 1);
  const auto enables = fixture.nodes[2]->enable_writes;
  fixture.run("zero 3", output);
  require(fixture.nodes[2]->eeprom_writes == 1 && !fixture.axes[2].held &&
    fixture.nodes[2]->enable_writes == enables && fixture.axes[2].torque_steps == 0,
    "torque-mode zero saves while disabled and never resumes the prior force");
  fixture.run("step 3 1", output);
  require(fixture.nodes[2]->target_current == 28, "a later explicit key may re-enter CST at a fresh 0.05Nm target");
  for (const auto & node : fixture.nodes) {
    require(node->triggers == 0, "torque mode must never issue a PP motion trigger");
  }
}

void torque_fault_and_speed_checks(const JsonOutput & output)
{
  bool power_cut = false;
  chassis::ethercat::CyclicFault fault([&] {power_cut = true;});
  TorqueFixture fixture;
  fixture.run("step 1 1", output);
  for (const int velocity : {-600, 600}) {
    fixture.nodes[0]->put<std::int32_t>(0x606C, velocity);
    rejects([&] {monitor_all(fixture.axes);});
  }
  fixture.nodes[0]->put<std::int32_t>(0x606C, 0);
  monitor_all(fixture.axes);
  fixture.nodes[0]->fault = &fault;
  fixture.nodes[0]->before_write = [&](std::uint16_t index, std::uint32_t value) {
      if (index == 0x6071 && value != 0) {
        fault.latch(chassis::ethercat::torque_fault_message(1, 100, -1001, 100000U));
      }
    };
  rejects([&] {fixture.run("step 1 1", output);});
  require(power_cut && fixture.axes[0].torque_steps == 1 && fixture.nodes[0]->target_current == 14,
    "failed torque write cuts power without claiming or storing a new applied target");
  const auto enables = fixture.nodes[0]->enable_writes;
  rejects([&] {fixture.run("step 1 -1", output);});
  require(fixture.nodes[0]->enable_writes == enables, "latched torque fault must not re-enable or retry");
}

void manual_brake_and_zero(const JsonOutput & output)
{
  std::vector<std::string> words{
    "arm_yiyou_control", "--interface", "eth0", "--slaves", "1,2,3", "--control-mode", "manual"};
  std::vector<char *> args;
  for (auto & word : words) {args.push_back(word.data());}
  const auto parsed = parse(static_cast<int>(args.size()), args.data());
  require(parsed.manual_brake && !parsed.torque_control && parsed.torque_limit_nm == 100,
    "manual mode preserves the cyclic fault/torque guard without CST mapping");
  require(master_options(parsed, [] {}).cst_slaves.empty(), "manual mode uses the passive profile layout");
  TorqueFixture fixture;
  fixture.options.torque_control = false;
  fixture.options.manual_brake = true;
  for (auto & axis : fixture.axes) {
    axis.torque_control = false;
    axis.manual_brake = true;
  }
  for (auto & node : fixture.nodes) {
    node->before_write = [&](std::uint16_t index, std::uint32_t value) {
        require(index != 0x6071 && index != 0x607A && index != 0x60FF,
          "manual calibration never sends a torque/position/velocity target");
        if (index == 0x6040) {
          require(value == 0 || value == 0x10, "manual calibration never enables or resets a fault");
        }
        if (index == 0x2130) {
          for (const auto & other : fixture.nodes) {
            const auto mode = other->read(iswv::canopen::Object<std::uint8_t>{{0x2110, 0}, "brake mode"});
            const auto closed = other->read(iswv::canopen::Object<std::int8_t>{{0x2111, 0}, "brake output"});
            require(mode && mode.value() == 1 && closed && closed.value() == 1,
              "all brakes must be automatic/closed BEFORE an EEPROM save");
          }
        }
      };
  }
  for (const auto & forbidden : {"hold", "home", "step 1 1", "step 1 -1", "relax"}) {
    rejects<CommandRefused>([&] {fixture.run(forbidden, output);});
  }
  monitor_all(fixture.axes);
  fixture.run("release 1", output);
  require(fixture.axes[0].brake_release_requested && !fixture.axes[1].brake_release_requested,
    "release only changes the selected axis");
  rejects<CommandRefused>([&] {fixture.run("release 2", output);});
  rejects<CommandRefused>([&] {fixture.run("zero 1", output);});
  require(fixture.nodes[0]->eeprom_writes == 0, "released brakes prohibit EEPROM writes");
  fixture.nodes[0]->put<std::int32_t>(0x6064, 30000);  // Hand movement.
  monitor_all(fixture.axes);
  fixture.run("brake", output);
  require(!fixture.axes[0].brake_release_requested, "space restores the automatic brake command");
  fixture.nodes[0]->put<std::int8_t>(0x2707, 0);
  rejects<CommandRefused>([&] {fixture.run("zero 1", output);});
  require(fixture.nodes[0]->eeprom_writes == 0, "moving motor must not be zeroed");
  fixture.nodes[0]->put<std::int8_t>(0x2707, 1);
  fixture.run("zero 1", output);
  require(fixture.nodes[0]->eeprom_writes == 1 && !fixture.axes[0].held,
    "manual zero saves once without enabling or releasing again");
  fixture.run("release 2", output);
  fixture.run("brake", output);
  for (auto & node : fixture.nodes) {
    require(node->enable_writes == 0 && node->triggers == 0, "manual workflow has no active motion");
  }
  fixture.nodes[2]->put<std::uint16_t>(0x6041, 0x27);
  rejects([&] {monitor_all(fixture.axes);});
  fixture.nodes[2]->put<std::uint16_t>(0x6041, 0x40);
  bool power_cut = false;
  chassis::ethercat::CyclicFault fault([&] {power_cut = true;});
  fixture.nodes[0]->fault = &fault;
  fixture.nodes[0]->before_write = [&](std::uint16_t index, std::uint32_t value) {
      if (index == 0x2111 && value == 0) {fault.latch("simulated disconnect while releasing brake");}
    };
  rejects([&] {fixture.run("release 1", output);});
  require(power_cut && !fixture.axes[0].brake_release_requested,
    "a failed brake command trips power without reporting a successful release");
  rejects([&] {fixture.run("release 1", output);});
}

void stalled_feedback_is_bounded()
{
  const int saved = ::dup(STDOUT_FILENO);
  int channel[2];
  require(saved >= 0 && ::pipe(channel) == 0, "create backpressure test pipe");
  const auto restore = [&] {
      (void)::dup2(saved, STDOUT_FILENO);
      ::close(saved);
      ::close(channel[0]);
      ::close(channel[1]);
    };
  try {
    require(::dup2(channel[1], STDOUT_FILENO) >= 0, "connect stalled frontend output");
    JsonOutput blocked;
    char data[4096]{};
    while (::write(channel[1], data, sizeof(data)) > 0) {}
    require(errno == EAGAIN || errno == EWOULDBLOCK, "feedback pipe must be full");
    const auto started = std::chrono::steady_clock::now();
    rejects<std::runtime_error>([&] {blocked.send("{\"event\":\"feedback\"}");});
    require(std::chrono::steady_clock::now() - started < 2s,
      "stalled frontend must produce a bounded failure instead of indefinitely retaining CST output");
  } catch (...) {
    restore();
    throw;
  }
  restore();
}

int main()
{
  char pattern[] = "/tmp/yiyou-control-memory-XXXXXX";
  const auto directory = ::mkdtemp(pattern);
  if (!directory) {return 1;}
  const std::filesystem::path root(directory);
  try {
    LiveFrontendInput input;
    JsonOutput output;
    torque_protection_options();
    stalled_feedback_is_bounded();
    torque_mode_options_and_quantization();
    torque_keys_and_zero(output);
    torque_fault_and_speed_checks(output);
    manual_brake_and_zero(output);
    one_degree_jog(output);
    passive_zero_and_fault(root / "passive.yaml", output);
    fault_during_first_retreat(root / "startup-trip.yaml", output);
    restored_motion(root / "restored.yaml", output);
    untrusted_zero(root / "untrusted.yaml", output);
    torque_trip_during_zero_prevents_reenable(root / "torque-trip.yaml", output);
    std::filesystem::remove_all(root);
    return 0;
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    std::filesystem::remove_all(root);
    return 1;
  }
}
