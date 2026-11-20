#pragma once

#include <cmath>
#include <cstdint>
#include <exception>
#include <functional>
#include <optional>
#include <sstream>
#include <string>
#include <utility>

namespace chassis::ethercat
{
// Empty means a valid sample is within the limit. Missing data fails closed.
// Nm follows the drive's rated-torque object; no extra gearbox scaling is applied.
inline std::string torque_fault_message(
  std::uint16_t slave, double limit_nm, std::optional<std::int16_t> raw_permille,
  std::optional<std::uint32_t> rated_mnm)
{
  const char * missing = nullptr;
  if (!std::isfinite(limit_nm) || limit_nm <= 0) {
    missing = "invalid positive Nm limit";
  } else if (!raw_permille) {
    missing = "missing valid 0x6077 torque feedback";
  } else if (!rated_mnm || *rated_mnm == 0) {
    missing = "missing nonzero 0x6076 rated torque";
  }
  if (missing) {
    return "EtherCAT torque protection at slave " + std::to_string(slave) + ": " + missing;
  }
  const double torque_nm = static_cast<double>(*raw_permille) *
    static_cast<double>(*rated_mnm) / 1000000.0;
  if (std::abs(torque_nm) <= limit_nm) {return {};}
  std::ostringstream out;
  out << "EtherCAT torque limit exceeded at slave " << slave << ": actual_nm=" << torque_nm
      << " limit_nm=" << limit_nm;
  return out.str();
}

// Access is serialized by the master's image mutex. There is intentionally no reset.
class CyclicFault
{
public:
  explicit CyclicFault(std::function<void()> power_off = {})
  : power_off_(std::move(power_off)) {}

  bool latch(std::string reason)
  {
    if (!reason_.empty()) {return false;}
    reason_ = reason.empty() ? "EtherCAT fault" : std::move(reason);
    if (power_off_) {
      try {
        power_off_();
      } catch (const std::exception & error) {
        reason_ += "; power-off failed: ";
        reason_ += error.what();
      } catch (...) {
        reason_ += "; power-off failed with an unknown exception";
      }
    }
    return true;
  }

  bool empty() const {return reason_.empty();}
  const std::string & message() const {return reason_;}

private:
  std::function<void()> power_off_;
  std::string reason_;
};
}  // namespace chassis::ethercat
