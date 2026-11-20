#pragma once

#include <cstdint>
#include <functional>
#include <stdexcept>
#include <string>
#include <utility>

namespace chassis
{
// All state changes and command callbacks run on the hardware owner's executor.
// Constructing or requesting a session never opens hardware by itself.
struct RemoteControlSession
{
  enum class Stage {waiting, requested, starting, running, stopping, fault};
  enum class Source {none, app, remote};

  bool start_chassis{true};
  Stage stage{Stage::waiting};
  bool shared_control{false};
  Source owner{Source::none};
  Source startup_source{Source::none};
  std::uint64_t generation{0};
  bool hardware_active{false};
  bool restart_required{false};
  std::string last_error;

  std::function<void()> on_owner_changed;
  std::function<void(std::uint64_t, double, double, double)> remote_velocity;
  std::function<bool(std::uint64_t, unsigned, double)> remote_arm_step;
  std::function<void(std::uint64_t, int)> remote_arm_jog;
  std::function<void(std::uint64_t, bool)> remote_tracking;
  std::function<void(std::uint64_t)> remote_relay_toggle;

  bool request_start(Source source)
  {
    if (source == Source::none || stage == Stage::fault || stage == Stage::stopping ||
      restart_required)
    {
      return false;
    }
    if (stage != Stage::waiting) {return true;}
    startup_source = source;
    owner = Source::none;
    last_error.clear();
    stage = Stage::requested;
    ++generation;
    return true;
  }

  bool claim(Source source)
  {
    if (source == Source::none || stage != Stage::running) {return false;}
    if (owner == source) {return true;}
    owner = source;
    ++generation;
    if (on_owner_changed) {on_owner_changed();}
    return true;
  }

  void release(Source source)
  {
    if (source == Source::none || owner != source) {return;}
    owner = Source::none;
    ++generation;
    if (on_owner_changed) {on_owner_changed();}
  }

  void request_stop()
  {
    if (stage == Stage::fault || stage == Stage::waiting || stage == Stage::stopping) {
      return;
    }
    stage = Stage::stopping;
    owner = Source::none;
    ++generation;
    if (on_owner_changed) {on_owner_changed();}
  }

  void mark_running()
  {
    if (stage != Stage::starting && stage != Stage::requested) {
      throw std::logic_error("cannot mark a cancelled or faulted session running");
    }
    stage = Stage::running;
    hardware_active = true;
    owner = startup_source;
    ++generation;
    if (on_owner_changed) {on_owner_changed();}
  }

  void finish_stop(bool restart_after_stop = false)
  {
    const bool changed = hardware_active || owner != Source::none ||
      (stage != Stage::waiting && stage != Stage::fault);
    hardware_active = false;
    restart_required = restart_required || restart_after_stop;
    owner = Source::none;
    startup_source = Source::none;
    if (stage != Stage::fault) {stage = Stage::waiting;}
    if (changed) {++generation;}
  }

  void latch_fault(std::string message)
  {
    if (stage == Stage::fault) {return;}
    last_error = std::move(message);
    stage = Stage::fault;
    owner = Source::none;
    startup_source = Source::none;
    ++generation;
    if (on_owner_changed) {on_owner_changed();}
  }

  static const char * source_name(Source source) noexcept
  {
    switch (source) {
      case Source::none: return "none";
      case Source::app: return "app";
      case Source::remote: return "remote";
    }
    return "none";
  }

  static const char * stage_name(Stage value) noexcept
  {
    switch (value) {
      case Stage::waiting: return "waiting";
      case Stage::requested: return "requested";
      case Stage::starting: return "starting";
      case Stage::running: return "running";
      case Stage::stopping: return "stopping";
      case Stage::fault: return "fault";
    }
    return "fault";
  }
};

inline const char * source_name(RemoteControlSession::Source source) noexcept
{
  return RemoteControlSession::source_name(source);
}

inline const char * stage_name(RemoteControlSession::Stage stage) noexcept
{
  return RemoteControlSession::stage_name(stage);
}
}  // namespace chassis
