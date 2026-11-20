#include "motor/iswv/motor.h"
#include "motor/yiyou/motor.h"
#include "motor/bxi/communication.h"
#include "iswv/fake_transport.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
using iswv::canopen::ObjectAddress;
using iswv::canopen::SdoResponse;

void require(bool condition, const char * message)
{
  if (!condition) {throw std::runtime_error(message);}
}

template<typename Exception = std::exception, typename Function>
std::string rejects(Function function)
{
  try {
    function();
  } catch (const Exception & error) {
    return error.what();
  }
  throw std::runtime_error("expected operation to be rejected");
}

struct Access
{
  bool write;
  std::uint16_t index;
  std::uint32_t value;
  std::size_t size{0};
};

// A device-side object dictionary, shared by CAN SDO and EtherCAT CoE fakes.
// State changes are responses to controlwords, not prescribed read sequences.
class Dictionary
{
public:
  Dictionary()
  {
    put<std::uint16_t>(0x6041, 0x40);
    put<std::uint8_t>(0x2100, 1);
    put<std::int8_t>(0x6060, 0);
    put<std::int8_t>(0x6061, 0);
    put<std::int8_t>(0x2707, 1);
    put<std::int32_t>(0x6064, 12345);
    put<std::int32_t>(0x606C, 0);
    put<std::int32_t>(0x60FF, 2000);
    put<std::int16_t>(0x6071, 500);
    put<std::int16_t>(0x60B2, 100);
    put<std::uint8_t>(0x2110, 1);
    put<std::int8_t>(0x2111, 1);
    put<std::uint16_t>(0x2113, 0);
    put<std::uint16_t>(0x2114, 0);
  }

  template<typename T>
  void put(std::uint16_t index, T value)
  {
    const auto bytes = iswv::canopen::encode_little_endian(value);
    values_[{index, 0}] = {bytes.begin(), bytes.end()};
  }

  void erase(std::uint16_t index)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    values_.erase({index, 0});
  }

  iswv::Result<SdoResponse> read(ObjectAddress object)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto found = values_.find(object);
    if (found == values_.end()) {
      return iswv::Result<SdoResponse>::failure(
        iswv::make_sdo_abort(0x06020000, "unknown test object"));
    }
    SdoResponse response;
    response.object = object;
    response.size = static_cast<std::uint8_t>(found->second.size());
    std::copy(found->second.begin(), found->second.end(), response.data.begin());
    accesses_.push_back({false, object.index, raw(response.data.data(), response.size), response.size});
    return iswv::Result<SdoResponse>::success(response);
  }

  iswv::Result<void> write(ObjectAddress object, const std::uint8_t * data, std::size_t size)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto value = raw(data, size);
    accesses_.push_back({true, object.index, value, size});
    if (fail_write_ == object.index) {
      fail_write_ = 0;
      return iswv::Result<void>::failure(
        iswv::make_sdo_abort(0x08000020, "injected device write failure"));
    }
    values_[object] = {data, data + size};
    if (object.index == 0x6060) {put<std::int8_t>(0x6061, static_cast<std::int8_t>(value));}
    if (object.index == 0x6040) {
      std::uint16_t status = 0x40;
      switch (value & 0x0f) {
        case 0x0b: status = 0x07; break;
        case 0x06: status = 0x21; break;
        case 0x07: status = 0x23; break;
        case 0x0f: status = (value & 0x10) ? 0x1027 : 0x27; break;
        default: break;
      }
      put<std::uint16_t>(0x6041, status);
    }
    return iswv::Result<void>::success();
  }

  void fail_next_write(std::uint16_t index)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    fail_write_ = index;
  }

  std::vector<Access> accesses() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return accesses_;
  }

  std::vector<Access> writes() const
  {
    auto result = accesses();
    result.erase(
      std::remove_if(
        result.begin(), result.end(),
        [](const Access & access) {return !access.write;}), result.end());
    return result;
  }

private:
  static std::uint32_t raw(const std::uint8_t * data, std::size_t size)
  {
    std::uint32_t value = 0;
    for (std::size_t i = 0; i < size; ++i) {
      value |= static_cast<std::uint32_t>(data[i]) << (8U * i);
    }
    return value;
  }

  mutable std::mutex mutex_;
  std::map<ObjectAddress, std::vector<std::uint8_t>> values_;
  std::vector<Access> accesses_;
  std::uint16_t fail_write_{0};
};

bool is_write(const Access & access, std::uint16_t index, std::uint32_t value)
{
  return access.write && access.index == index && access.value == value;
}

bool has_write(const std::vector<Access> & accesses, std::uint16_t index, std::uint32_t value)
{
  return std::any_of(
    accesses.begin(), accesses.end(), [ = ](const Access & access) {
      return is_write(access, index, value);
    });
}

class CanDevice
{
public:
  explicit CanDevice(std::optional<std::int8_t> mode_readback = {})
  {
    transport->set_send_hook(
      [this, mode_readback](const iswv::CanFrame & request) {
        if (request.id != 0x601 || request.size != 8) {return;}
        const ObjectAddress object{
          static_cast<std::uint16_t>(request.data[1] | (request.data[2] << 8U)),
          request.data[3]};
        iswv::CanFrame response;
        response.id = 0x581;
        response.size = 8;
        std::copy_n(request.data.begin() + 1, 3, response.data.begin() + 1);
        if (request.data[0] == 0x40) {
          const auto result = dictionary.read(object);
          if (result) {
            response.data[0] = static_cast<std::uint8_t>(0x43 | ((4 - result.value().size) << 2));
            std::copy(
              result.value().data.begin(), result.value().data.end(),
              response.data.begin() + 4);
            if (object == ObjectAddress{0x6060, 0} && mode_readback) {
              response.data[4] = static_cast<std::uint8_t>(*mode_readback);
            }
          } else {
            abort_reply(response, result.error());
          }
        } else if ((request.data[0] & 0xe3) == 0x23) {
          const auto size = 4U - ((request.data[0] >> 2U) & 3U);
          const auto result = dictionary.write(object, request.data.data() + 4, size);
          if (result) {response.data[0] = 0x60;} else {abort_reply(response, result.error());}
        } else {
          abort_reply(response, iswv::make_sdo_abort(0x05040001, "unsupported test SDO"));
        }
        transport->inject(response);
      });
  }

  Dictionary dictionary;
  std::shared_ptr<iswv::FakeTransport> transport = std::make_shared<iswv::FakeTransport>();

private:
  static void abort_reply(iswv::CanFrame & response, const iswv::Error & error)
  {
    response.data[0] = 0x80;
    const auto bytes = iswv::canopen::encode_little_endian(
      error.sdo_abort_code.value_or(0x08000000));
    std::copy(bytes.begin(), bytes.end(), response.data.begin() + 4);
  }
};

class CoeNode final : public chassis::ethercat::Node
{
public:
  explicit CoeNode(std::uint16_t node_id = 1) : id_(node_id) {}
  std::uint16_t id() const noexcept override {return id_;}

  iswv::Result<SdoResponse> upload_blocking(
    ObjectAddress object, iswv::canopen::SdoOptions options) override
  {
    request_timeouts.push_back(options.timeout);
    if (object.index == 0x607A) {target_read_timeout = options.timeout;}
    return dictionary.read(object);
  }

  iswv::Result<void> download_blocking(
    ObjectAddress object, const std::uint8_t * data, std::size_t size,
    iswv::canopen::SdoOptions options) override
  {
    request_timeouts.push_back(options.timeout);
    if (object.index == 0x607A) {
      target_write_timeout = options.timeout;
      if (options.timeout < target_write_delay) {
        return iswv::Result<void>::failure(iswv::ErrorCode::timeout, "simulated PDO recovery delay");
      }
    }
    auto result = dictionary.write(object, data, size);
    if (result && after_write) {after_write(object);}
    return result;
  }

  std::optional<std::int16_t> cached_torque_permille() const override
  {
    return torque_permille;
  }

  std::optional<std::uint32_t> cached_rated_torque_millinewton_meter() const override
  {
    return rated_torque_mnm;
  }

  std::uint16_t id_;
  Dictionary dictionary;
  std::optional<std::int16_t> torque_permille;
  std::optional<std::uint32_t> rated_torque_mnm;
  std::function<void(ObjectAddress)> after_write;
  std::vector<std::chrono::milliseconds> request_timeouts;
  std::chrono::milliseconds target_write_delay{0}, target_write_timeout{0}, target_read_timeout{0};
};

void require_iswv_stopped(const Dictionary & dictionary)
{
  const auto writes = dictionary.writes();
  require(writes.size() >= 3, "cleanup must attempt all three stop commands");
  require(is_write(writes[writes.size() - 3], 0x60FF, 0), "cleanup clears target velocity");
  require(is_write(writes[writes.size() - 2], 0x6040, 0x0b), "cleanup sends quick stop");
  require(is_write(writes.back(), 0x6040, 0), "cleanup disables voltage last");
}

void bxi_transport_rejects_invalid_frames()
{
  iswv::FakeTransport transport;
  const std::array<std::uint8_t, 8> payload{1, 2, 3, 4, 5, 6, 7, 8};
  require(!bxi::send_payload(transport, 0x800, {}, payload), "reject nonstandard CAN ID");
  require(!bxi::send_payload(transport, 1, {false, true}, payload), "reject BRS on classic CAN");
  require(transport.sent_frames().empty(), "invalid frames must not reach the transport");
  require(
    bxi::send_payload(transport, 0x7ff, {false, false}, payload).has_value(),
    "valid classic CAN frame reaches the transport");
  const auto frames = transport.sent_frames();
  require(
    frames.size() == 1 && frames[0].id == 0x7ff && frames[0].size == 8 &&
    !frames[0].fd && !frames[0].bitrate_switch &&
    std::equal(payload.begin(), payload.end(), frames[0].data.begin()),
    "communication preserves payload, ID and frame flags");
}

void iswv_tuning_order()
{
  CanDevice device;
  iswv::canopen::CanopenMaster master(device.transport);
  chassis::IswvDrive motor(master, iswv::AxisConfiguration::manual_travel(1));
  unsigned callbacks = 0;
  motor.enable_rpm(
    60, 60, [&](iswv::Axis & axis) {
      ++callbacks;
      const auto accesses = device.dictionary.accesses();
      require(accesses.size() >= 3, "tuning requires disable/zero/status confirmation");
      require(is_write(accesses[0], 0x6040, 0), "disable voltage precedes tuning");
      require(is_write(accesses[1], 0x60FF, 0), "zero velocity precedes tuning");
      require(
        !accesses.back().write && accesses.back().index == 0x6041 &&
        accesses.back().value == 0x40, "tuning requires confirmed disabled state");
      require(
        axis.node()->write(iswv::objects::quick_stop_option, 2).has_value(),
        "tuning callback can issue a real SDO transaction");
    });
  require(callbacks == 1, "tuning callback runs exactly once");
  require(has_write(device.dictionary.writes(), 0x6040, 0x0f), "normal enable completes");
  motor.stop();
  require_iswv_stopped(device.dictionary);
}

void iswv_callback_failure_cleans_up()
{
  CanDevice device;
  iswv::canopen::CanopenMaster master(device.transport);
  chassis::IswvDrive motor(master, iswv::AxisConfiguration::manual_travel(1));
  const auto message = rejects<std::runtime_error>(
    [&] {
      motor.enable_rpm(60, 60, [](iswv::Axis &) {throw std::runtime_error("tuning failed");});
    });
  require(message == "tuning failed", "original callback failure must propagate");
  require_iswv_stopped(device.dictionary);
  require(!has_write(device.dictionary.writes(), 0x6040, 0x0f), "failed tuning must not enable");
}

void iswv_failed_clear_never_tunes()
{
  CanDevice device;
  iswv::canopen::CanopenMaster master(device.transport);
  chassis::IswvDrive motor(master, iswv::AxisConfiguration::manual_travel(1));
  device.dictionary.fail_next_write(0x60FF);
  bool tuned = false;
  const auto message = rejects<std::runtime_error>(
    [&] {
      motor.enable_rpm(60, 60, [&](iswv::Axis &) {tuned = true;});
    });
  require(!tuned, "failed zero command must prevent tuning");
  require(message.find("clear target velocity") != std::string::npos, "SDO error retains context");
  require_iswv_stopped(device.dictionary);
}

void iswv_velocity_mode_survives_pdo_commands()
{
  CanDevice device;
  iswv::canopen::CanopenMaster master(device.transport);
  chassis::IswvDrive motor(master, iswv::AxisConfiguration::manual_travel(1));
  for (const int attempt : {0, 1, 2}) {
    const std::int8_t expected_mode = attempt == 1 ? -3 : 3;
    if (attempt == 0) {
      motor.enable_rpm(60, 60);
    } else if (attempt == 1) {
      motor.enable_rpm(60, 60, {}, true);
    } else {
      motor.enable_rpm(60, 60, [](iswv::Axis &) {});
    }
    const auto accesses = device.dictionary.accesses();
    const auto mode_read = std::find_if(
      accesses.rbegin(), accesses.rend(), [](const Access & access) {
        return !access.write && access.index == 0x6060;
      });
    require(
      mode_read != accesses.rend() &&
      mode_read->value == static_cast<std::uint8_t>(expected_mode),
      "enable must write and confirm the selected velocity mode");

    device.transport->clear_sent();
    for (const double rpm : {10.0, 0.0, -10.0}) {
      motor.set_velocity_pdo_rpm(rpm);
    }
    auto frames = device.transport->sent_frames();
    frames.erase(
      std::remove_if(
        frames.begin(), frames.end(),
        [](const iswv::CanFrame & frame) {return frame.id != 0x201;}), frames.end());
    require(frames.size() == 3, "each velocity command must send one RPDO");
    const std::array<int, 3> signs{1, 0, -1};
    for (std::size_t i = 0; i < frames.size(); ++i) {
      const auto & frame = frames[i];
      require(
        frame.size == 7 && frame.data[0] == 0x0f && frame.data[1] == 0 &&
        frame.data[2] == static_cast<std::uint8_t>(expected_mode),
        "motion and zero-speed RPDOs must preserve the selected velocity mode");
      const auto raw = iswv::canopen::decode_little_endian<std::int32_t>(
        frame.data.data() + 3, 4).value();
      require(
        signs[i] == 0 ? raw == 0 : raw * signs[i] > 0,
        "velocity RPDO must preserve commanded direction and zero");
    }
    motor.stop();
    require_iswv_stopped(device.dictionary);
  }
}

void iswv_velocity_mode_readback_failure_cleans_up()
{
  for (const bool immediate : {false, true}) {
    CanDevice device(static_cast<std::int8_t>(immediate ? 3 : -3));
    iswv::canopen::CanopenMaster master(device.transport);
    chassis::IswvDrive motor(master, iswv::AxisConfiguration::manual_travel(1));
    const auto message = rejects<std::runtime_error>(
      [&] {motor.enable_rpm(60, 60, {}, immediate);});
    require(
      message.find("mode readback mismatch") != std::string::npos &&
      message.find(immediate ? "requested -3, got 3" : "requested 3, got -3") !=
      std::string::npos, "mode mismatch reports the requested and actual values");
    require_iswv_stopped(device.dictionary);
    require(
      !has_write(device.dictionary.writes(), 0x6040, 0x0f),
      "wrong velocity mode must prevent enable");
  }
}

void yiyou_pp_hold_handshake_stop()
{
  auto node = std::make_shared<CoeNode>();
  chassis::YiyouMotor motor(node);
  motor.enable_position(1000, 2000, 2000);
  auto writes = node->dictionary.writes();
  const auto hold = std::find_if(
    writes.begin(), writes.end(), [](const Access & access) {
      return is_write(access, 0x607A, 12345);
    });
  const auto enable = std::find_if(
    writes.begin(), writes.end(), [](const Access & access) {
      return is_write(access, 0x6040, 0x0f);
    });
  require(hold < enable && enable != writes.end(), "PP seeds current position before enable");
  require(!has_write(writes, 0x6040, 0x3f), "PP enable must not trigger a motion target");
  motor.prepare_position_command(12500);
  require(motor.stage_position_command(), "PP stage waits for cleared acknowledgment");
  motor.trigger_position_command();
  require(!motor.finish_position_command(), "PP completion first clears acknowledged set-point");
  require(motor.finish_position_command(), "PP command completes after acknowledgment clears");
  require(has_write(node->dictionary.writes(), 0x607A, 12500), "staged position reaches device");
  rejects<std::logic_error>([&] {motor.set_velocity(1);});
  motor.stop();
  writes = node->dictionary.writes();
  require(is_write(writes[writes.size() - 2], 0x6040, 0x010f), "PP stops with halt");
  require(is_write(writes.back(), 0x6040, 0), "PP disables voltage after halt");
}

void yiyou_pp_stage_has_bounded_recovery_budget()
{
  using namespace std::chrono_literals;
  for (const auto delay : {6ms, 11ms}) {
    auto node = std::make_shared<CoeNode>();
    chassis::YiyouMotor motor(node);
    motor.enable_position(1000, 2000, 2000);
    node->target_write_delay = delay;
    motor.prepare_position_command(12500);
    if (delay == 6ms) {
      require(motor.stage_position_command(), "two lost cycles followed by valid delivery fit budget");
      require(node->target_read_timeout == 5ms, "mailbox verification remains bounded to 5 ms");
    } else {
      rejects<std::runtime_error>([&] {motor.stage_position_command();});
      rejects<std::logic_error>([&] {motor.trigger_position_command();});
    }
    require(node->target_write_timeout == 10ms, "PDO staging wait is bounded to 10 ms");
    require(!has_write(node->dictionary.writes(), 0x6040, 0x3f), "staging alone never triggers motion");
    motor.stop();
  }
}

void yiyou_pp_profile_updates_are_verified_without_motion()
{
  using namespace std::chrono_literals;
  for (const bool halted : {false, true}) {
    auto node = std::make_shared<CoeNode>();
    chassis::YiyouMotor motor(node);
    motor.enable_position(1000, 2000, 2000);
    if (halted) {motor.halt_position();}
    const auto before = node->dictionary.accesses().size();
    node->request_timeouts.clear();
    motor.set_position_profile(500, 800, 900);
    const auto accesses = node->dictionary.accesses();
    require(accesses.size() == before + 10, "profile update checks state and standstill then verifies all fields");
    const std::uint16_t indices[]{0x6081, 0x6083, 0x6084};
    const std::uint32_t values[]{500, 800, 900};
    for (unsigned i = 0; i < 3; ++i) {
      const auto & write = accesses[before + 4 + i * 2];
      const auto & read = accesses[before + 5 + i * 2];
      require(is_write(write, indices[i], values[i]) && write.size == 4,
        "profile change writes only the three uint32 profile objects");
      require(!read.write && read.index == indices[i] && read.value == values[i],
        "each profile write must be read back before proceeding");
    }
    require(node->request_timeouts.size() == 10 &&
      std::all_of(node->request_timeouts.begin(), node->request_timeouts.end(),
        [](auto timeout) {return timeout == 5ms;}),
      "every profile precheck/read/write must have a 5 ms CoE budget");
    motor.prepare_position_command(12500);
    require(is_write(node->dictionary.writes().back(), 0x6040, halted ? 0x012f : 0x002f),
      "profile update must preserve the existing halt state for subsequent preparation");
    motor.stop();
  }
}

void yiyou_pp_profile_rejects_invalid_parameters_and_modes_without_writes()
{
  auto node = std::make_shared<CoeNode>();
  chassis::YiyouMotor motor(node);
  rejects<std::logic_error>([&] {motor.set_position_profile(500, 800, 900);});
  require(node->dictionary.writes().empty(), "disabled profile update must not write");
  for (const bool cst : {false, true}) {
    if (cst) {motor.enable_cst();} else {motor.enable(2000, 2000);}
    const auto count = node->dictionary.writes().size();
    rejects<std::logic_error>([&] {motor.set_position_profile(500, 800, 900);});
    require(node->dictionary.writes().size() == count, "other modes must reject PP profile without writes");
    motor.stop();
  }
  motor.enable_position(1000, 2000, 2000);
  const auto before = node->dictionary.accesses().size();
  for (unsigned zero = 0; zero < 3; ++zero) {
    std::uint32_t values[]{500, 800, 900};
    values[zero] = 0;
    rejects<std::invalid_argument>([&] {motor.set_position_profile(values[0], values[1], values[2]);});
    require(node->dictionary.accesses().size() == before, "zero profile fields must fail before any I/O");
  }
  motor.stop();
}

void yiyou_pp_profile_rejects_pending_handshakes_without_io()
{
  for (unsigned phase = 0; phase < 4; ++phase) {
    auto node = std::make_shared<CoeNode>();
    chassis::YiyouMotor motor(node);
    motor.enable_position(1000, 2000, 2000);
    motor.prepare_position_command(12500);
    if (phase >= 1) {require(motor.stage_position_command(), "profile test target stages");}
    if (phase >= 2) {motor.trigger_position_command();}
    if (phase == 3) {require(!motor.finish_position_command(), "profile test ACK reset is pending");}
    const auto before = node->dictionary.accesses().size();
    rejects<std::logic_error>([&] {motor.set_position_profile(500, 800, 900);});
    require(node->dictionary.accesses().size() == before,
      "pending PP handshake must reject profile change before any I/O");
    motor.stop();
  }
}

void yiyou_pp_profile_requires_healthy_stationary_feedback()
{
  auto node = std::make_shared<CoeNode>();
  chassis::YiyouMotor motor(node);
  motor.enable_position(1000, 2000, 2000);
  const auto before = node->dictionary.writes().size();
  for (const auto & state : {std::pair<int, int>{0, 0}, {1, 1}, {1, -1}, {-1, 0}, {2, 0}}) {
    node->dictionary.put<std::int8_t>(0x2707, static_cast<std::int8_t>(state.first));
    node->dictionary.put<std::int32_t>(0x606C, state.second);
    rejects([&] {motor.set_position_profile(500, 800, 900);});
    require(node->dictionary.writes().size() == before,
      "moving or inconsistent standstill feedback must prevent all profile writes");
  }
  node->dictionary.put<std::int8_t>(0x2707, 1);
  node->dictionary.put<std::int32_t>(0x606C, 0);
  for (const auto status : {0x0040, 0x0008, 0x0827, 0x2027}) {
    node->dictionary.put<std::uint16_t>(0x6041, static_cast<std::uint16_t>(status));
    rejects<std::runtime_error>([&] {motor.set_position_profile(500, 800, 900);});
    require(node->dictionary.writes().size() == before, "unhealthy PP state must prevent profile writes");
  }
  node->dictionary.put<std::uint16_t>(0x6041, 0x0027);
  node->dictionary.put<std::int8_t>(0x6061, 3);
  rejects<std::runtime_error>([&] {motor.set_position_profile(500, 800, 900);});
  require(node->dictionary.writes().size() == before, "unexpected displayed mode must prevent profile writes");
  node->dictionary.put<std::int8_t>(0x6061, 1);
  motor.stop();

  for (const auto missing : {0x6041, 0x6061, 0x2707, 0x606C}) {
    auto missing_node = std::make_shared<CoeNode>();
    chassis::YiyouMotor missing_motor(missing_node);
    missing_motor.enable_position(1000, 2000, 2000);
    const auto count = missing_node->dictionary.writes().size();
    missing_node->dictionary.erase(static_cast<std::uint16_t>(missing));
    rejects<std::runtime_error>([&] {missing_motor.set_position_profile(500, 800, 900);});
    require(missing_node->dictionary.writes().size() == count, "missing precheck feedback must prevent profile writes");
  }
}

void yiyou_pp_profile_stationary_tolerance_requires_stop_flag_and_bounds_signed_speed()
{
  auto node = std::make_shared<CoeNode>();
  chassis::YiyouMotor motor(node);
  motor.enable_position(1000, 2000, 2000);
  struct Case {std::int32_t velocity; bool accepted;};
  for (const auto test : {Case{-5, true}, Case{5, true}, Case{-1, true}, Case{1, true},
      Case{-6, false}, Case{6, false}, Case{std::numeric_limits<std::int32_t>::min(), false},
      Case{std::numeric_limits<std::int32_t>::max(), false}})
  {
    node->dictionary.put<std::int32_t>(0x606C, test.velocity);
    const auto before = node->dictionary.writes().size();
    if (test.accepted) {
      motor.set_position_profile(500, 800, 900, 5);
      require(node->dictionary.writes().size() == before + 3,
        "signed velocity inside the supplied tolerance must allow all three profile writes");
    } else {
      rejects<std::logic_error>([&] {motor.set_position_profile(500, 800, 900, 5);});
      require(node->dictionary.writes().size() == before,
        "velocity above tolerance, including INT32_MIN, must reject before any profile write");
    }
  }
  node->dictionary.put<std::int32_t>(0x606C, 0);
  node->dictionary.put<std::int8_t>(0x2707, 0);
  const auto before = node->dictionary.writes().size();
  rejects<std::logic_error>([&] {motor.set_position_profile(500, 800, 900, 5);});
  require(node->dictionary.writes().size() == before,
    "velocity tolerance must never override a drive that reports motion");
  node->dictionary.put<std::int8_t>(0x2707, 1);
  motor.stop();
}

void yiyou_pp_profile_write_and_readback_failures_propagate_without_motion()
{
  const std::uint16_t indices[]{0x6081, 0x6083, 0x6084};
  for (unsigned field = 0; field < 3; ++field) {
    for (int fault = 0; fault < 3; ++fault) {
      auto node = std::make_shared<CoeNode>();
      chassis::YiyouMotor motor(node);
      motor.enable_position(1000, 2000, 2000);
      const auto before = node->dictionary.writes().size();
      if (fault == 0) {node->dictionary.fail_next_write(indices[field]);}
      node->after_write = [&, fault, field](ObjectAddress object) {
          if (object.index != indices[field]) {return;}
          if (fault == 1) {node->dictionary.erase(object.index);}
          if (fault == 2) {node->dictionary.put<std::uint32_t>(object.index, 9999);}
        };
      const auto reason = rejects<std::runtime_error>([&] {motor.set_position_profile(500, 800, 900);});
      if (fault == 2) {
        require(reason.find("readback mismatch") != std::string::npos, "profile mismatch must be diagnosed");
      }
      const auto writes = node->dictionary.writes();
      require(writes.size() == before + field + 1, "failed profile must stop immediately without retry or cleanup writes");
      for (unsigned i = 0; i <= field; ++i) {
        require(writes[before + i].index == indices[i], "profile failure must never write target/controlword/EEPROM");
      }
      node->after_write = {};
      motor.stop();
    }
  }
}

void yiyou_pp_halt_cancels_every_handshake_stage()
{
  // idle (with an accepted target), clearing, staged, triggered, resetting.
  for (unsigned phase = 0; phase < 5; ++phase) {
    auto node = std::make_shared<CoeNode>();
    chassis::YiyouMotor motor(node);
    motor.enable_position(1000, 2000, 2000);
    motor.move_to_position(12500);
    if (phase >= 1) {motor.prepare_position_command(13000);}
    if (phase >= 2) {require(motor.stage_position_command(), "stage old command");}
    if (phase >= 3) {motor.trigger_position_command();}
    if (phase == 4) {require(!motor.finish_position_command(), "reset old acknowledge");}
    if (phase == 1 || phase == 4) {
      node->dictionary.put<std::uint16_t>(0x6041, 0x1027);  // Delayed acknowledge clear.
    } else if (phase == 3) {
      node->dictionary.put<std::uint16_t>(0x6041, 0x0027);  // Delayed acknowledge set.
    }

    const auto before_halt = node->dictionary.writes().size();
    motor.halt_position();
    auto writes = node->dictionary.writes();
    require(writes.size() == before_halt + 1, "halt must only write its controlword");
    require(is_write(writes.back(), 0x6040, 0x012f), "halt preserves enable and clears bit 4");
    node->dictionary.put<std::uint16_t>(0x6041, 0x1427);  // Late old ACK and target-reached.
    require(motor.finish_position_command(), "cancelled handshake is idle despite late ACK");
    rejects<std::logic_error>([&] {motor.stage_position_command();});
    rejects<std::logic_error>([&] {motor.trigger_position_command();});
    require(!motor.position_reached(), "cancelled target must not become reached");
    require(
      node->dictionary.writes().size() == writes.size(),
      "late completion must not write a controlword or revive the cancelled target");

    motor.prepare_position_command(12000);
    require(
      is_write(node->dictionary.writes().back(), 0x6040, 0x012f),
      "preparing a replacement must not resume the previous target");
    node->dictionary.put<std::uint16_t>(0x6041, 0x1027);
    const auto waiting_writes = node->dictionary.writes().size();
    require(!motor.stage_position_command(), "replacement waits for old ACK to clear");
    require(
      node->dictionary.writes().size() == waiting_writes,
      "waiting for old ACK must leave halt and target unchanged");
    node->dictionary.put<std::uint16_t>(0x6041, 0x0027);
    require(motor.stage_position_command(), "replacement stages after ACK clears");
    writes = node->dictionary.writes();
    require(is_write(writes.back(), 0x607A, 12000), "replacement target is staged under halt");
    require(
      !has_write(std::vector<Access>(writes.begin() + before_halt, writes.end()), 0x6040, 0x003f),
      "staging must not release halt");
    motor.trigger_position_command();
    require(
      is_write(node->dictionary.writes().back(), 0x6040, 0x003f),
      "only the verified replacement trigger releases halt");
    require(!motor.finish_position_command(), "replacement acknowledge resets");
    require(motor.finish_position_command(), "replacement handshake completes");
    motor.stop();
  }
}

void yiyou_pp_blocking_replacement_preserves_halt()
{
  auto node = std::make_shared<CoeNode>();
  chassis::YiyouMotor motor(node);
  motor.enable_position(1000, 2000, 2000);
  motor.move_to_position(13000);
  motor.halt_position();
  const auto before = node->dictionary.accesses().size();
  motor.move_to_position(12000);
  const auto accesses = node->dictionary.accesses();
  bool confirmed = false;
  bool triggered = false;
  for (auto it = accesses.begin() + before; it != accesses.end(); ++it) {
    if (!it->write && it->index == 0x607A && it->value == 12000) {confirmed = true;}
    if (it->write && it->index == 0x6040) {
      if (it->value == 0x003f) {
        require(confirmed, "replacement target must be read back before releasing halt");
        triggered = true;
      } else if (!triggered) {
        require(it->value == 0x012f, "blocking prepare must preserve halt");
      }
    }
  }
  require(triggered, "blocking replacement sends a new trigger");
  motor.stop();
}

void yiyou_pp_failed_halt_cancels_and_mode_reset_clears_halt()
{
  auto node = std::make_shared<CoeNode>();
  chassis::YiyouMotor motor(node);
  rejects<std::logic_error>([&] {motor.halt_position();});
  motor.enable_position(1000, 2000, 2000);
  motor.prepare_position_command(13000);
  require(motor.stage_position_command(), "stage before failed halt");
  node->dictionary.fail_next_write(0x6040);
  rejects<std::runtime_error>([&] {motor.halt_position();});
  const auto count = node->dictionary.writes().size();
  require(motor.finish_position_command(), "failed halt still cancels local handshake");
  rejects<std::logic_error>([&] {motor.trigger_position_command();});
  require(node->dictionary.writes().size() == count, "failed halt leaves no stale trigger");
  motor.prepare_position_command(12000);
  require(
    is_write(node->dictionary.writes().back(), 0x6040, 0x012f),
    "failed halt must still be preserved by replacement preparation");
  require(motor.stage_position_command(), "stage replacement after failed halt");
  node->dictionary.fail_next_write(0x6040);
  rejects<std::runtime_error>([&] {motor.trigger_position_command();});
  motor.halt_position();
  motor.stop();
  motor.enable_position(1000, 2000, 2000);
  motor.prepare_position_command(12500);
  require(
    is_write(node->dictionary.writes().back(), 0x6040, 0x002f),
    "new mode enable clears the previous halted state");
  motor.stop();
  motor.enable_cst();
  const auto cst_count = node->dictionary.writes().size();
  rejects<std::logic_error>([&] {motor.halt_position();});
  require(node->dictionary.writes().size() == cst_count, "PP halt must reject CST without writes");
  motor.stop();
}

void yiyou_pp_health_failure_has_no_stop_side_effects()
{
  auto node = std::make_shared<CoeNode>();
  chassis::YiyouMotor motor(node);
  rejects<std::logic_error>([&] {motor.check_position_mode();});
  require(node->dictionary.writes().empty(), "disabled PP health check must not write");
  motor.enable_position(1000, 2000, 2000);
  motor.check_position_mode();
  const auto before = node->dictionary.writes().size();
  node->dictionary.put<std::int8_t>(0x6061, 3);
  rejects<std::runtime_error>([&] {motor.check_position_mode();});
  require(node->dictionary.writes().size() == before, "wrong mode must not initiate stop");
  node->dictionary.put<std::int8_t>(0x6061, 1);
  for (const auto status : {0x0040, 0x0008, 0x0827, 0x2027}) {
    node->dictionary.put<std::uint16_t>(0x6041, static_cast<std::uint16_t>(status));
    rejects<std::runtime_error>([&] {motor.check_position_mode();});
    require(
      node->dictionary.writes().size() == before,
      "invalid PP state must propagate without stop so owner can power off first");
  }
  node->dictionary.put<std::uint16_t>(0x6041, 0x0027);
  motor.check_position_mode();
  motor.stop();
}

void yiyou_cst_zero_limits_stop()
{
  auto node = std::make_shared<CoeNode>();
  chassis::YiyouMotor motor(node);
  motor.enable_cst();
  const auto accesses = node->dictionary.accesses();
  const auto enable = std::find_if(
    accesses.begin(), accesses.end(), [](const Access & access) {
      return is_write(access, 0x6040, 0x0f);
    });
  for (const auto index : {0x6071, 0x60B2}) {
    const auto confirmed = std::find_if(
      accesses.begin(), accesses.end(), [ = ](const Access & access) {
        return !access.write && access.index == index && access.value == 0;
      });
    require(confirmed < enable && enable != accesses.end(), "CST confirms zero before enable");
  }
  motor.set_current_permille(1000);
  const auto count = node->dictionary.writes().size();
  rejects<std::out_of_range>([&] {motor.set_current_permille(1001);});
  rejects<std::out_of_range>([&] {motor.set_current_permille(-1001);});
  rejects<std::logic_error>([&] {motor.set_velocity(1);});
  require(node->dictionary.writes().size() == count, "invalid commands must not transmit");
  motor.stop();
  const auto writes = node->dictionary.writes();
  require(is_write(writes[writes.size() - 2], 0x6071, 0), "CST clears current before disabling");
  require(is_write(writes.back(), 0x6040, 0), "CST disables voltage");
}

void yiyou_stop_continues_after_sdo_failure()
{
  auto node = std::make_shared<CoeNode>();
  chassis::YiyouMotor motor(node);
  motor.enable_cst();
  node->dictionary.fail_next_write(0x6071);
  const auto message = rejects<std::runtime_error>([&] {motor.stop();});
  require(message.find("clear 0x6071") != std::string::npos, "stop preserves first failure");
  require(
    is_write(node->dictionary.writes().back(), 0x6040, 0),
    "failed zero-current command must still disable voltage");
}

void yiyou_invalid_enable_does_not_write()
{
  auto node = std::make_shared<CoeNode>();
  chassis::YiyouMotor motor(node);
  rejects<std::invalid_argument>([&] {motor.enable_position(0, 1, 1);});
  rejects<std::invalid_argument>([&] {motor.enable(0, 1);});
  rejects<std::logic_error>([&] {motor.set_current_permille(1);});
  require(node->dictionary.writes().empty(), "invalid initial commands must not write objects");
}

void yiyou_disabled_check_and_brake_order()
{
  auto node = std::make_shared<CoeNode>();
  {
    chassis::YiyouMotor motor(node);
    motor.check_disabled();
    require(node->dictionary.writes().empty(), "checking disabled state must be read-only");
    motor.release_brake();
    const auto accesses = node->dictionary.accesses();
    const auto first_write = std::find_if(accesses.begin(), accesses.end(),
      [](const Access & access) {return access.write;});
    for (const auto & item : std::vector<std::pair<std::uint16_t, std::size_t>>{
        {0x6041, 2}, {0x2707, 1}, {0x2110, 1}, {0x2111, 1}, {0x2113, 2}, {0x2114, 2}})
    {
      const auto found = std::find_if(accesses.begin(), first_write, [&](const Access & access) {
          return !access.write && access.index == item.first && access.size == item.second;
        });
      require(found != first_write, "brake objects and disabled/stopped feedback must be verified before any write");
    }
    const auto writes = node->dictionary.writes();
    require(writes.size() == 3 && is_write(writes[0], 0x2111, 1) &&
      is_write(writes[1], 0x2110, 0) && is_write(writes[2], 0x2111, 0),
      "release must seed closed, select manual mode, then request release");
    motor.engage_brake();
    motor.engage_brake();
  }
  const auto writes = node->dictionary.writes();
  require(writes.size() == 7, "brake operations and destruction must not write enable, motion, fault reset or EEPROM objects");
  for (std::size_t i : {std::size_t{3}, std::size_t{5}}) {
    require(is_write(writes[i], 0x2111, 1) && is_write(writes[i + 1], 0x2110, 1),
      "engage is repeatable and restores automatic mode after requesting closed");
  }
  const auto accesses = node->dictionary.accesses();
  for (std::size_t i = 0; i < accesses.size(); ++i) {
    if (!accesses[i].write) {continue;}
    require(accesses[i].size == 1 && i + 1 < accesses.size() && !accesses[i + 1].write &&
      accesses[i + 1].index == accesses[i].index && accesses[i + 1].value == accesses[i].value &&
      accesses[i + 1].size == 1, "each correctly sized brake write must be read back before another write");
  }
  auto startup = std::make_shared<CoeNode>();
  chassis::YiyouMotor motor(startup);
  motor.engage_brake();
  require(startup->dictionary.writes().size() == 2, "startup engage does not require a preceding release");
}

void yiyou_brakes_reject_non_disabled_or_moving_state()
{
  for (std::uint16_t status : {0x0000, 0x0021, 0x0023, 0x0027, 0x0007, 0x000f, 0x0008, 0xffff}) {
    auto node = std::make_shared<CoeNode>();
    node->dictionary.put<std::uint16_t>(0x6041, status);
    chassis::YiyouMotor motor(node);
    rejects([&] {motor.check_disabled();});
    rejects([&] {motor.release_brake();});
    rejects([&] {motor.engage_brake();});
    require(node->dictionary.writes().empty(), "non-disabled, unknown and fault states must not cause any brake or enable writes");
  }
  for (std::int8_t stopped : {0, 2, -1}) {
    auto node = std::make_shared<CoeNode>();
    node->dictionary.put<std::int8_t>(0x2707, stopped);
    chassis::YiyouMotor motor(node);
    rejects([&] {motor.release_brake();});
    require(node->dictionary.writes().empty(), "moving or invalid stopped feedback must block release before writing");
  }
  auto node = std::make_shared<CoeNode>();
  node->dictionary.erase(0x6041);
  chassis::YiyouMotor motor(node);
  rejects([&] {motor.check_disabled();});
  rejects([&] {motor.release_brake();});
  rejects([&] {motor.engage_brake();});
  require(node->dictionary.writes().empty(), "unavailable state feedback cannot permit brake operations");
}

void yiyou_brakes_require_supported_valid_objects()
{
  const std::vector<std::function<void(Dictionary &)>> invalid{
    [](Dictionary & d) {d.put<std::uint8_t>(0x2110, 2);},
    [](Dictionary & d) {d.put<std::int8_t>(0x2111, -1);},
    [](Dictionary & d) {d.put<std::int8_t>(0x2111, 2);},
    [](Dictionary & d) {d.put<std::uint16_t>(0x2113, 10001);},
    [](Dictionary & d) {d.put<std::uint16_t>(0x2114, 6501);},
    [](Dictionary & d) {d.put<std::uint16_t>(0x2110, 1);},
    [](Dictionary & d) {d.put<std::uint16_t>(0x2111, 1);},
    [](Dictionary & d) {d.put<std::uint8_t>(0x2113, 0);},
    [](Dictionary & d) {d.put<std::uint8_t>(0x2114, 0);},
  };
  for (bool release : {false, true}) {
    const auto operate = [=](chassis::YiyouMotor & motor) {
        if (release) {motor.release_brake();} else {motor.engage_brake();}
      };
    for (auto index : {0x2110, 0x2111, 0x2113, 0x2114}) {
      auto node = std::make_shared<CoeNode>();
      node->dictionary.erase(static_cast<std::uint16_t>(index));
      chassis::YiyouMotor motor(node);
      rejects([&] {operate(motor);});
      require(node->dictionary.writes().empty(), "missing brake objects must fail before any write");
    }
    for (const auto & invalidate : invalid) {
      auto node = std::make_shared<CoeNode>();
      invalidate(node->dictionary);
      chassis::YiyouMotor motor(node);
      rejects([&] {operate(motor);});
      require(node->dictionary.writes().empty(), "invalid brake widths, values or delays must fail before any write");
    }
  }
}

void yiyou_brake_write_and_readback_failures_do_not_retry()
{
  for (bool release : {false, true}) {
    for (int fault = 0; fault < 4; ++fault) {
      auto node = std::make_shared<CoeNode>();
      chassis::YiyouMotor motor(node);
      if (fault == 0) {node->dictionary.fail_next_write(0x2111);}
      node->after_write = [&, fault](ObjectAddress object) {
          if (object.index == 0x2111 && fault == 1) {node->dictionary.erase(0x2111);}
          if (object.index == 0x2111 && fault == 2) {node->dictionary.put<std::int8_t>(0x2111, 0);}
          if (object.index == 0x2110 && fault == 3) {
            node->dictionary.put<std::uint8_t>(0x2110, release ? 1 : 0);
          }
        };
      rejects([&] {
        if (release) {motor.release_brake();} else {motor.engage_brake();}
      });
      require(node->dictionary.writes().size() == (fault == 3 ? 2U : 1U),
        "failed brake writes or readbacks must abort without retry, enable, release or EEPROM write");
    }
  }
  auto node = std::make_shared<CoeNode>();
  chassis::YiyouMotor motor(node);
  node->after_write = [&](ObjectAddress object) {
      if (object.index == 0x2110) {node->dictionary.put<std::uint16_t>(0x6041, 0x27);}
    };
  rejects([&] {motor.release_brake();});
  require(node->dictionary.writes().size() == 2 && !has_write(node->dictionary.writes(), 0x2111, 0),
    "unexpected enabling after selecting manual mode must block the release write");
}

void yiyou_brake_cancellation_is_checked_before_release_and_during_wait()
{
  auto node = std::make_shared<CoeNode>();
  chassis::YiyouMotor motor(node);
  rejects([&] {motor.release_brake([] {throw std::runtime_error("cancelled");});});
  require(node->dictionary.writes().empty(), "already-cancelled release must not write");
  unsigned checks = 0;
  rejects([&] {motor.release_brake([&] {
      if (++checks == 2) {throw std::runtime_error("cancel before release");}
    });});
  require(node->dictionary.writes().size() == 2 && !has_write(node->dictionary.writes(), 0x2111, 0),
    "cancellation after manual-mode selection must still prevent releasing the brake");

  for (bool release : {false, true}) {
    auto waiting = std::make_shared<CoeNode>();
    chassis::YiyouMotor waiting_motor(waiting);
    const auto cancel_wait = [&] {
        if (waiting->dictionary.writes().size() == (release ? 3U : 2U)) {
          throw std::runtime_error("cancel brake wait");
        }
      };
    const auto message = rejects([&] {
      if (release) {waiting_motor.release_brake(cancel_wait);} else {waiting_motor.engage_brake(cancel_wait);}
    });
    require(message == "cancel brake wait", "both brake waits must invoke cancellation even with zero configured delay");
  }
}

void yiyou_torque_feedback_is_cached_optional()
{
  auto node = std::make_shared<CoeNode>();
  chassis::YiyouMotor motor(node);
  require(!motor.actual_torque_permille(), "missing cached torque stays unknown");
  require(!motor.rated_torque_millinewton_meter(), "missing cached rated torque stays unknown");
  require(!motor.actual_torque_newton_meter(), "missing rating prevents Nm conversion");
  require(node->dictionary.accesses().empty(), "cached torque getters must not issue SDO reads");

  node->torque_permille = static_cast<std::int16_t>(-125);
  node->rated_torque_mnm = 2400U;
  require(motor.actual_torque_permille() == -125, "cached signed torque preserves negative values");
  require(
    motor.rated_torque_millinewton_meter() == 2400U,
    "cached rated torque returns motor-side mNm");
  const auto nm = motor.actual_torque_newton_meter();
  require(nm && std::abs(*nm + 0.3) < 1e-12, "Nm conversion uses raw * rated_mNm / 1e6");
  require(node->dictionary.accesses().empty(), "torque conversion must not issue SDO reads");

  node->rated_torque_mnm = 0U;
  require(!motor.actual_torque_newton_meter(), "zero rated torque is treated as unknown");
}

void yiyou_torque_feedback_does_not_change_motion_output()
{
  auto node = std::make_shared<CoeNode>();
  node->torque_permille = static_cast<std::int16_t>(321);
  node->rated_torque_mnm = 1000U;
  chassis::YiyouMotor motor(node);
  motor.enable_position(1000, 2000, 2000);
  const auto writes_before = node->dictionary.writes().size();
  require(motor.actual_torque_permille() == 321, "enabled PP motor exposes cached torque");
  require(motor.actual_torque_newton_meter().has_value(), "enabled PP motor converts cached torque");
  require(
    node->dictionary.writes().size() == writes_before,
    "reading cached torque must not write motion objects");
  motor.stop();
}
}  // namespace

int main()
{
  const std::vector<std::pair<const char *, std::function<void()>>> tests{
    {"BXI invalid transport frames", bxi_transport_rejects_invalid_frames},
    {"ISWV tuning order", iswv_tuning_order},
    {"ISWV callback failure cleanup", iswv_callback_failure_cleans_up},
    {"ISWV failed zero blocks tuning", iswv_failed_clear_never_tunes},
    {"ISWV velocity modes survive motion and zero PDOs", iswv_velocity_mode_survives_pdo_commands},
    {"ISWV velocity mode readback cleanup", iswv_velocity_mode_readback_failure_cleans_up},
    {"Yiyou PP hold handshake stop", yiyou_pp_hold_handshake_stop},
    {"Yiyou PP bounded recovery budget", yiyou_pp_stage_has_bounded_recovery_budget},
    {"Yiyou PP profile verified without motion", yiyou_pp_profile_updates_are_verified_without_motion},
    {"Yiyou PP profile invalid parameters and modes", yiyou_pp_profile_rejects_invalid_parameters_and_modes_without_writes},
    {"Yiyou PP profile rejects pending handshakes", yiyou_pp_profile_rejects_pending_handshakes_without_io},
    {"Yiyou PP profile requires healthy standstill", yiyou_pp_profile_requires_healthy_stationary_feedback},
    {"Yiyou PP profile bounded stationary tolerance", yiyou_pp_profile_stationary_tolerance_requires_stop_flag_and_bounds_signed_speed},
    {"Yiyou PP profile failures propagate without motion", yiyou_pp_profile_write_and_readback_failures_propagate_without_motion},
    {"Yiyou PP halt cancels every handshake stage", yiyou_pp_halt_cancels_every_handshake_stage},
    {"Yiyou PP blocking replacement preserves halt", yiyou_pp_blocking_replacement_preserves_halt},
    {"Yiyou PP failed halt cancellation and mode reset", yiyou_pp_failed_halt_cancels_and_mode_reset_clears_halt},
    {"Yiyou PP health failure has no stop side effects", yiyou_pp_health_failure_has_no_stop_side_effects},
    {"Yiyou CST zero limits stop", yiyou_cst_zero_limits_stop},
    {"Yiyou failed stop still disables", yiyou_stop_continues_after_sdo_failure},
    {"Yiyou invalid enable", yiyou_invalid_enable_does_not_write},
    {"Yiyou disabled brake sequence", yiyou_disabled_check_and_brake_order},
    {"Yiyou brake state rejection", yiyou_brakes_reject_non_disabled_or_moving_state},
    {"Yiyou brake object validation", yiyou_brakes_require_supported_valid_objects},
    {"Yiyou brake write and readback failures", yiyou_brake_write_and_readback_failures_do_not_retry},
    {"Yiyou brake cancellation", yiyou_brake_cancellation_is_checked_before_release_and_during_wait},
    {"Yiyou torque feedback cached optional", yiyou_torque_feedback_is_cached_optional},
    {"Yiyou torque feedback leaves output unchanged", yiyou_torque_feedback_does_not_change_motion_output},
  };
  unsigned failed = 0;
  for (const auto & test : tests) {
    try {
      test.second();
      std::cout << "PASS " << test.first << '\n';
    } catch (const std::exception & error) {
      ++failed;
      std::cerr << "FAIL " << test.first << ": " << error.what() << '\n';
    }
  }
  return failed == 0 ? 0 : 1;
}
