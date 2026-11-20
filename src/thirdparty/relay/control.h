#pragma once

#include "iswv/transport.hpp"

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>

namespace chassis
{

// Pinquanwei digital-output protocol, CAN manual V3.0 sections 2.5/2.6/2.10.
// Exclusively owns this transport's receive handlers; no hardware is opened here.
// start/update/toggle run on the control thread; receive callbacks only enqueue.
class CanRelay
{
public:
  using Clock = std::chrono::steady_clock;

  struct Config
  {
    unsigned address{1};
    unsigned channel{2};
    bool active_high{true};
    std::chrono::milliseconds timeout{500};
  };

  explicit CanRelay(std::shared_ptr<iswv::ICanTransport> transport);
  CanRelay(std::shared_ptr<iswv::ICanTransport> transport, Config config);
  ~CanRelay();

  CanRelay(const CanRelay &) = delete;
  CanRelay & operator=(const CanRelay &) = delete;

  // Verify address/1M, ordinary mode and output-hold settings, then confirm OFF.
  // Call after motor homing/calibration; no module configuration is modified.
  void start(Clock::time_point now = Clock::now());
  void update(Clock::time_point now = Clock::now());
  bool toggle();
  bool ready() const {return initialized_ && !failed();}
  bool failed() const {return stage_ == Stage::failed;}
  std::string take_report();

private:
  enum class Stage {dormant, info, mode, settings, read, write, verify, idle, failed};
  struct Inbox;

  void request(Stage stage, Clock::time_point now);
  void complete();
  void fail(const std::string & reason);

  std::shared_ptr<iswv::ICanTransport> transport_;
  const Config config_;
  std::shared_ptr<Inbox> inbox_;
  Stage stage_{Stage::dormant};
  Clock::time_point deadline_{};
  std::uint16_t target_value_{0};
  bool initialized_{false};
  bool desired_on_{false};
  bool confirmed_on_{false};
  bool target_on_{false};
  std::string report_;
};

}  // namespace chassis
