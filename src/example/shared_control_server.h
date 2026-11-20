#pragma once

#include "example/control_session.h"

#include <chrono>
#include <memory>
#include <string>

namespace chassis
{
// Local lifecycle endpoint; poll() must run on the same executor as the session.
// It accepts requests only, leaving hardware construction/cleanup to main.cpp.
class SharedControlServer
{
public:
  SharedControlServer(
    std::shared_ptr<RemoteControlSession> session, const std::string & path, bool arm_enabled,
    std::chrono::milliseconds stop_confirmation_timeout = std::chrono::seconds(30));
  ~SharedControlServer();
  SharedControlServer(const SharedControlServer &) = delete;
  SharedControlServer & operator=(const SharedControlServer &) = delete;
  void poll();
  // Keep the original process alive until stop clients can observe cleanup, or expire.
  bool stop_confirmation_pending() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace chassis
