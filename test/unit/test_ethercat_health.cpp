#include "motor/yiyou/cyclic_health.h"
#include "motor/yiyou/communication.h"
#include "motor/yiyou/torque_protection.h"

#include <iostream>
#include <limits>
#include <stdexcept>
#include <vector>

using namespace std::chrono_literals;
using chassis::ethercat::CyclicHealth;
using chassis::ethercat::CyclicFault;
using chassis::ethercat::HealthDecision;
using chassis::ethercat::torque_fault_message;

namespace
{
void require(bool condition, const char * message)
{
  if (!condition) {throw std::runtime_error(message);}
}

void cst_selection_supports_multiple_slaves_and_legacy_calls()
{
  using chassis::ethercat::Options;
  using chassis::ethercat::uses_cst;
  using chassis::ethercat::validate_selection;
  Options options{"eth0", {1, 2, 3}};
  validate_selection(options);
  for (auto slave : options.slaves) {
    require(!uses_cst(options, slave), "default selections retain the profile PDO layout");
  }

  // All pre-existing aggregate fields retain their original position.
  Options legacy{"eth0", {1, 2, 3}, 2, true, 100.0, [] {}};
  validate_selection(legacy);
  require(!uses_cst(legacy, 1) && uses_cst(legacy, 2) && !uses_cst(legacy, 3),
    "legacy cst_slave still selects exactly one CST layout with torque protection");

  options.cst_slaves = {3, 1};
  validate_selection(options);
  require(uses_cst(options, 1) && !uses_cst(options, 2) && uses_cst(options, 3),
    "an unordered CST subset selects every requested layout without changing other slaves");
  require(!uses_cst(options, 4), "unselected positions are not implicitly CST");

  options.cst_slaves = options.slaves;
  options.monitor_torque = true;
  options.torque_limit_nm = 100.0;
  options.fault_power_off = [] {};
  validate_selection(options);
  for (auto slave : options.slaves) {
    require(uses_cst(options, slave), "all three selected slaves can use CST with the torque guard");
  }
}

void invalid_cst_selections_are_rejected_before_hardware()
{
  chassis::ethercat::Options options{"eth0", {1, 2, 3}};
  const auto rejected = [](const auto & value) {
      try {chassis::ethercat::validate_selection(value);} catch (const std::invalid_argument &) {
        return true;
      }
      return false;
    };
  for (const auto & selection : std::vector<std::vector<std::uint16_t>>{
      {1, 1}, {4}, {0}, {200}, {1, 4}})
  {
    options.cst_slaves = selection;
    require(rejected(options), "CST positions must be unique members of the selected chain");
  }
  options.cst_slave = 1;
  for (const auto & selection : std::vector<std::vector<std::uint16_t>>{{1}, {2, 3}}) {
    options.cst_slaves = selection;
    require(rejected(options), "legacy and multiple CST selection cannot be combined or silently merged");
  }
  options.cst_slaves.clear();
  chassis::ethercat::validate_selection(options);
  options.cst_slave = 4;
  require(rejected(options), "legacy CST selection must still belong to the selected chain");
}

void cst_homing_requires_disabled_zero_output_and_confirmed_mode()
{
  using chassis::ethercat::CstPdoState;
  using chassis::ethercat::cst_pdo_write_allowed;
  CstPdoState state{true, 0, 0, 10, 10, true};
  for (auto mode : {6, 10}) {
    require(cst_pdo_write_allowed(state, 0x6060, mode), "disabled zero output permits CST/HM mode selection");
    auto unsafe = state;
    unsafe.disabled = false;
    require(!cst_pdo_write_allowed(unsafe, 0x6060, mode), "mode changes need confirmed switch_on_disabled");
    unsafe = state;
    unsafe.outputs_applied = false;
    require(!cst_pdo_write_allowed(unsafe, 0x6060, mode), "a pending PDO write must be acknowledged before mode changes");
    unsafe = state;
    unsafe.control = 0x000f;
    require(!cst_pdo_write_allowed(unsafe, 0x6060, mode), "a pending enable blocks mode changes even before feedback changes");
    for (auto target : {std::int16_t{-1}, std::int16_t{1}}) {
      unsafe = state;
      unsafe.target_permille = target;
      require(!cst_pdo_write_allowed(unsafe, 0x6060, mode), "both signs of pending torque block mode changes");
    }
  }
  for (auto mode : {-4, 0, 1, 3, 4, 7, 13}) {
    require(!cst_pdo_write_allowed(state, 0x6060, mode), "CST layout cannot select unrelated motion modes");
  }

  state.commanded_mode = 6;
  require(!cst_pdo_write_allowed(state, 0x6040, 0x0010), "HM display must be confirmed before its trigger");
  require(!cst_pdo_write_allowed(state, 0x6071, 1), "pending HM mode cannot accept nonzero torque");
  state.displayed_mode = 6;
  require(cst_pdo_write_allowed(state, 0x6040, 0x0010), "confirmed disabled HM permits the vendor in-place-zero trigger");
  state.outputs_applied = false;
  require(!cst_pdo_write_allowed(state, 0x6040, 0x0010), "HM mode must be acknowledged before the zero trigger");
  state.outputs_applied = true;
  for (auto control : {0x0006, 0x0007, 0x000f, 0x001f, 0x0080}) {
    require(!cst_pdo_write_allowed(state, 0x6040, control), "HM must never enable, start powered homing or reset faults");
  }
  state.disabled = false;
  require(!cst_pdo_write_allowed(state, 0x6040, 0x0010), "HM cannot trigger with enabled or unconfirmed feedback");
  require(cst_pdo_write_allowed(state, 0x6040, 0), "disable voltage remains available with unsafe feedback");
  state.disabled = true;
  state.target_permille = 1;
  require(!cst_pdo_write_allowed(state, 0x6040, 0x0010), "HM trigger also checks the shared torque output");
  require(cst_pdo_write_allowed(state, 0x6071, 0), "clearing torque remains available in HM");
  state.target_permille = 0;
  state.control = 0x0010;
  require(!cst_pdo_write_allowed(state, 0x6060, 10), "returning to CST must first clear the HM trigger");
  state.control = 0;
  require(cst_pdo_write_allowed(state, 0x6060, 10), "cleared disabled HM can return to CST");
  state.commanded_mode = 10;
  require(!cst_pdo_write_allowed(state, 0x6040, 0x000f), "a pending mode change cannot enable while the drive still reports HM");
  require(!cst_pdo_write_allowed(state, 0x6071, -1), "a pending return to CST cannot apply torque while HM is reported");
  state.displayed_mode = 10;
  state.outputs_applied = false;
  require(!cst_pdo_write_allowed(state, 0x6040, 0x000f), "returning to CST must be acknowledged before enabling");
  state.outputs_applied = true;
  for (auto control : {0x0006, 0x0007, 0x000f}) {
    require(cst_pdo_write_allowed(state, 0x6040, control), "confirmed CST retains the normal enable sequence");
  }
  for (auto target : {-1000, 0, 1000}) {
    require(cst_pdo_write_allowed(state, 0x6071, target), "confirmed CST retains its target writes");
  }
}

void transient_loss_does_not_publish_or_acknowledge()
{
  CyclicHealth health;
  auto last_feedback = 0ms;
  unsigned acknowledged = 7;
  const unsigned pending = 8;
  for (const auto now : {2ms, 4ms}) {
    require(health.observe_pdo(-1, 12) == HealthDecision::retry, "two lost frames may be retried");
    if (CyclicHealth::can_publish(-1, 12, now - last_feedback, false)) {
      last_feedback = now;
      acknowledged = pending;
    }
  }
  require(last_feedback == 0ms && acknowledged == 7, "invalid data must not refresh or acknowledge");
  require(health.observe_pdo(12, 12) == HealthDecision::accept, "good frame recovers transient loss");
  require(CyclicHealth::can_publish(12, 12, 6ms - last_feedback, false), "fresh complete frame publishes");
  require(health.pdo_failures() == 0, "success resets only the current consecutive burst");
  require(health.observe_pdo(0, 12) == HealthDecision::retry, "new loss burst starts from one");
}

void persistent_loss_and_deadline_still_fault()
{
  CyclicHealth health;
  require(health.observe_pdo(-1, 12) == HealthDecision::retry, "first miss");
  require(health.observe_pdo(9, 12) == HealthDecision::retry, "partial WKC is also invalid");
  require(health.observe_pdo(0, 12) == HealthDecision::fault, "third consecutive miss must fault");
  require(!CyclicHealth::can_publish(12, 12, 2ms, true), "recovery must not bypass latched fault");
  require(!CyclicHealth::feedback_expired(20ms), "existing strict deadline boundary retained");
  require(CyclicHealth::feedback_expired(20ms + 1us), "stale feedback must fault without three misses");
  require(!CyclicHealth::can_publish(12, 12, 21ms, false), "late good frame cannot erase elapsed deadline");
  require(!CyclicHealth::can_publish(9, 12, 1ms, false), "partial image cannot be published");
}

void al_retry_is_per_slave_and_does_not_wait_twenty_ms()
{
  CyclicHealth health;
  require(health.should_poll_al(10, 1), "normal AL polling cadence");
  require(!health.should_poll_al(11, 1), "healthy slave is not polled every cycle");
  require(health.observe_al(1, false, false) == HealthDecision::retry, "first timeout is retried");
  require(health.should_poll_al(11, 1), "failed slave is retried on the next cycle");
  require(!health.should_poll_al(11, 2), "retry does not add queries for healthy slaves");
  require(health.observe_pdo(12, 12) == HealthDecision::accept, "PDO stays independently healthy");
  require(health.al_failures(1) == 1, "good PDO does not erase AL failure count");
  require(health.observe_al(2, true, true) == HealthDecision::accept, "other slave remains healthy");
  require(health.observe_al(1, false, false) == HealthDecision::retry, "second timeout is retried");
  require(health.observe_al(1, false, false) == HealthDecision::fault, "third timeout faults same slave");
}

void al_recovery_and_explicit_device_fault_are_distinct()
{
  CyclicHealth health;
  health.observe_al(4, false, false);
  require(health.observe_al(4, true, true) == HealthDecision::accept, "OP response resets AL retry");
  require(!health.should_poll_al(13, 4), "recovered slave resumes normal poll cadence");
  require(health.observe_al(4, false, false) == HealthDecision::retry, "isolated new timeout is allowed");
  require(health.observe_al(4, true, false) == HealthDecision::fault, "explicit non-OP/error faults immediately");
  require(health.observe_al(1, true, false) == HealthDecision::fault, "no previous misses needed for device fault");
}

void torque_limits_use_absolute_drive_units()
{
  for (const auto raw : {std::int16_t{0}, std::int16_t{999}, std::int16_t{-999},
    std::int16_t{1000}, std::int16_t{-1000}})
  {
    require(torque_fault_message(1, 100.0, raw, 100000U).empty(),
      "a valid sample at or below the 100 Nm threshold must pass in either direction");
  }
  for (const auto raw : {std::int16_t{1001}, std::int16_t{-1001},
    std::numeric_limits<std::int16_t>::min(), std::numeric_limits<std::int16_t>::max()})
  {
    const auto reason = torque_fault_message(3, 100.0, raw, 100000U);
    require(reason.find("slave 3") != std::string::npos &&
      reason.find("limit_nm=100") != std::string::npos,
      "positive and negative overloads, including INT16_MIN, identify the affected slave");
  }
  require(torque_fault_message(2, 1.0, 500, 2000U).empty(),
    "rated mNm and raw permille convert to Nm without extra gearbox scaling");
  require(!torque_fault_message(2, 1.0, 501, 2000U).empty(),
    "the smallest sample above a non-default threshold trips");
}

void missing_torque_data_and_invalid_limits_fail_closed()
{
  require(!torque_fault_message(1, 100.0, {}, 100000U).empty(), "missing 6077 is not zero torque");
  require(!torque_fault_message(1, 100.0, 0, {}).empty(), "missing 6076 refuses protection");
  require(!torque_fault_message(1, 100.0, 0, 0U).empty(), "zero rated torque refuses protection");
  for (double limit : {0.0, -1.0, std::numeric_limits<double>::infinity(),
    std::numeric_limits<double>::quiet_NaN()})
  {
    require(!torque_fault_message(1, limit, 0, 100000U).empty(), "invalid limit fails closed");
  }
  chassis::ethercat::Options options{"eth0", {1, 2, 3}};
  options.monitor_torque = true;
  chassis::ethercat::validate_selection(options);  // Existing optional telemetry remains valid.
  options.torque_limit_nm = 100.0;
  const auto rejected = [](const auto & value) {
      try {chassis::ethercat::validate_selection(value);} catch (const std::invalid_argument &) {
        return true;
      }
      return false;
    };
  require(rejected(options), "a limit without a power-off callback is rejected before hardware");
  options.fault_power_off = [] {};
  chassis::ethercat::validate_selection(options);
  options.monitor_torque = false;
  require(rejected(options), "a limit without mapped torque monitoring is rejected");
  options.monitor_torque = true;
  for (double limit : {0.0, -1.0, std::numeric_limits<double>::infinity(),
    std::numeric_limits<double>::quiet_NaN()})
  {
    options.torque_limit_nm = limit;
    require(rejected(options), "invalid torque limit is rejected before hardware");
  }
}

void fault_cuts_power_before_acknowledgement_and_never_recovers()
{
  std::vector<std::string> events;
  unsigned power_off_calls = 0;
  CyclicFault fault([&] {
      require(!fault.empty(), "fault is already latched before callback runs");
      ++power_off_calls;
      events.emplace_back("power_off");
    });
  unsigned acknowledged = 7;
  const unsigned pending = 8;
  const auto reason = torque_fault_message(2, 100.0, -1001, 100000U);
  require(fault.latch(reason), "first overload latches");
  events.emplace_back("diagnostic");
  if (CyclicHealth::can_publish(12, 12, 2ms, !fault.empty())) {acknowledged = pending;}
  events.emplace_back("cleanup_wait");
  require(events == std::vector<std::string>{"power_off", "diagnostic", "cleanup_wait"},
    "shared power is cut synchronously before logging and cleanup waits");
  require(acknowledged == 7, "a complete overlimit PDO must not acknowledge a pending command");
  require(torque_fault_message(2, 100.0, 0, 100000U).empty(), "sample may later return to zero");
  require(!fault.empty() && !CyclicHealth::can_publish(12, 12, 2ms, !fault.empty()),
    "zero torque and valid WKC cannot reset an existing fault");
  require(!fault.latch("later fault") && power_off_calls == 1, "power cut is requested only once");
  require(fault.message() == reason, "first fault remains available for diagnosis");
}

void every_fault_reason_and_callback_failure_remain_latched()
{
  for (const char * reason : {"WKC mismatch", "AL non-OP", "stale feedback", "missing torque"}) {
    unsigned calls = 0;
    CyclicFault fault([&] {++calls;});
    require(fault.latch(reason) && calls == 1 && !fault.empty(),
      "communication and missing-data faults use the same power-cut path");
  }
  unsigned calls = 0;
  CyclicFault failed_switch([&] {++calls; throw std::runtime_error("switch unavailable");});
  require(failed_switch.latch("torque fault"), "callback exception cannot bypass the fault latch");
  require(failed_switch.message().find("switch unavailable") != std::string::npos,
    "power-switch errors are retained in the fault message");
  require(!failed_switch.latch("retry") && calls == 1, "throwing callback is not repeatedly invoked");
  CyclicFault unknown_failure([] {throw 1;});
  require(unknown_failure.latch("torque fault") && !unknown_failure.empty(),
    "non-standard callback exceptions also retain the fault");
  CyclicFault telemetry_only;
  require(telemetry_only.latch("ordinary WKC fault"), "existing no-callback users still latch faults");
}
}  // namespace

int main()
{
  try {
    cst_selection_supports_multiple_slaves_and_legacy_calls();
    invalid_cst_selections_are_rejected_before_hardware();
    cst_homing_requires_disabled_zero_output_and_confirmed_mode();
    transient_loss_does_not_publish_or_acknowledge();
    persistent_loss_and_deadline_still_fault();
    al_retry_is_per_slave_and_does_not_wait_twenty_ms();
    al_recovery_and_explicit_device_fault_are_distinct();
    torque_limits_use_absolute_drive_units();
    missing_torque_data_and_invalid_limits_fail_closed();
    fault_cuts_power_before_acknowledgement_and_never_recovers();
    every_fault_reason_and_callback_failure_remain_latched();
    std::cout << "EtherCAT health policy tests passed\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
