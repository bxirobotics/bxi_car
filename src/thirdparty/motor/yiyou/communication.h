#pragma once

#include "iswv/canopen/types.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <optional>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

namespace chassis::ethercat
{
// CoE uses the same typed object dictionary as CANopen, but never CAN frames/NMT.
// This small boundary also permits offline drive/state-machine tests.
class Node
{
public:
  virtual ~Node() = default;
  virtual std::uint16_t id() const noexcept = 0;
  virtual iswv::Result<iswv::canopen::SdoResponse> upload_blocking(
    iswv::canopen::ObjectAddress object, iswv::canopen::SdoOptions options = {}) = 0;
  virtual iswv::Result<void> download_blocking(
    iswv::canopen::ObjectAddress object, const std::uint8_t * data, std::size_t size,
    iswv::canopen::SdoOptions options = {}) = 0;
  virtual std::optional<std::int16_t> cached_torque_permille() const {return {};}
  virtual std::optional<std::uint32_t> cached_rated_torque_millinewton_meter() const {return {};}

  template<typename T>
  iswv::Result<T> read(
    iswv::canopen::Object<T> object,
    iswv::canopen::SdoOptions options = {})
  {
    auto response = upload_blocking(object.address, options);
    if (!response) {return iswv::Result<T>::failure(response.error());}
    return iswv::canopen::decode_little_endian<T>(
      response.value().data.data(),
      response.value().size);
  }
  template<typename T, typename U>
  iswv::Result<void> write(
    iswv::canopen::Object<T> object, U value, iswv::canopen::SdoOptions options = {})
  {
    static_assert(std::is_convertible_v<U, T>, "object value is not convertible");
    const auto bytes = iswv::canopen::encode_little_endian(static_cast<T>(value));
    return download_blocking(object.address, bytes.data(), bytes.size(), options);
  }
};

struct Options
{
  std::string interface;
  // Physical chain positions (1 based), not CAN Node-IDs. Maximum 199 for SOEM 1.4.
  std::vector<std::uint16_t> slaves;
  // Legacy single-CST selection; mutually exclusive with a nonempty cst_slaves.
  std::optional<std::uint16_t> cst_slave {};
  bool monitor_torque{false};
  // Absolute drive-reported torque limit, converted from 0x6077 and 0x6076.
  // Requires verified torque feedback on every selected slave.
  std::optional<double> torque_limit_nm{};
  // Called once on the first latched fault, before logging or cleanup waits.
  // Must be bounded and must not call Master/Node: the image mutex may be held.
  // The caller owns power hardware and must keep it alive until Master is closed.
  std::function<void()> fault_power_off{};
  // Each selected position uses the existing CST PDO layout (0x6071/0x6078).
  // Kept last to preserve existing aggregate initialization.
  std::vector<std::uint16_t> cst_slaves{};
};

inline bool uses_cst(const Options & options, std::uint16_t slave)
{
  return (options.cst_slave && *options.cst_slave == slave) ||
         std::find(options.cst_slaves.begin(), options.cst_slaves.end(), slave) !=
         options.cst_slaves.end();
}

// Snapshot of one healthy CST PDO image, read while holding the image mutex.
// disabled means the feedback confirms CiA402 switch_on_disabled, not just bit2=0.
struct CstPdoState
{
  bool disabled;
  std::uint16_t control;
  std::int16_t target_permille;
  std::int8_t commanded_mode;
  std::int8_t displayed_mode;
  bool outputs_applied;
};

inline bool cst_pdo_write_allowed(
  const CstPdoState & state, std::uint16_t index, std::int32_t value)
{
  if (index == 0x6060) {
    // HM is used only for disabled in-place zeroing; clear its trigger before
    // returning to CST. Even an identical mode write requires a safe transition.
    return (value == 6 || value == 10) && state.outputs_applied && state.disabled &&
           state.control == 0 && state.target_permille == 0;
  }
  if (index == 0x6040) {
    if (value == 0) {return true;}  // Never prevent a request to disable voltage.
    if (state.commanded_mode == 6 && state.displayed_mode == 6) {
      return value == 0x0010 && state.outputs_applied && state.disabled &&
             state.control == 0 && state.target_permille == 0;
    }
    return state.outputs_applied && state.commanded_mode == 10 && state.displayed_mode == 10;
  }
  if (index == 0x6071) {
    return value == 0 || (state.commanded_mode == 10 && state.displayed_mode == 10);
  }
  return false;
}

// Pure argument validation, before a caller opens the shared motor power switch.
inline void validate_selection(const Options & options)
{
  if (options.interface.empty() || options.interface.size() > 15 ||
    options.interface == "lo" ||
    options.interface.find_first_not_of(
      "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_.:-") != std::string::npos)
  {
    throw std::invalid_argument("specify a physical EtherCAT --interface; lo is loopback");
  }
  if (options.slaves.empty()) {throw std::invalid_argument("select EtherCAT slave positions");}
  std::set<std::uint16_t> positions;
  for (auto slave : options.slaves) {
    if (slave == 0 || slave > 199 || !positions.insert(slave).second) {
      throw std::invalid_argument("EtherCAT positions must be unique and in 1..199");
    }
  }
  if (options.cst_slave && !options.cst_slaves.empty()) {
    throw std::invalid_argument("select CST slaves using either cst_slave or cst_slaves, not both");
  }
  if (options.cst_slave && positions.count(*options.cst_slave) == 0) {
    throw std::invalid_argument("CST EtherCAT slave must be one of the selected positions");
  }
  std::set<std::uint16_t> cst_positions;
  for (auto slave : options.cst_slaves) {
    if (positions.count(slave) == 0 || !cst_positions.insert(slave).second) {
      throw std::invalid_argument("CST EtherCAT positions must be unique selected positions");
    }
  }
  if (options.torque_limit_nm &&
    (!std::isfinite(*options.torque_limit_nm) || *options.torque_limit_nm <= 0 ||
    !options.monitor_torque || !options.fault_power_off))
  {
    throw std::invalid_argument(
            "torque protection requires a finite positive Nm limit, torque monitoring and power-off callback");
  }
}

// Owns the EtherCAT session and cyclic process data, not shared board power.
// Construct one master for the selected chain and share its nodes among motors.
class Master
{
public:
  explicit Master(const Options & options);
  ~Master();
  Master(const Master &) = delete;
  Master & operator=(const Master &) = delete;
  std::shared_ptr<Node> node(std::uint16_t position);
  // Throws on a latched WKC/state/deadline fault; never automatically re-enables.
  void check_health() const;

private:
  struct Impl;
  std::shared_ptr<Impl> impl_;
};
}  // namespace chassis::ethercat
