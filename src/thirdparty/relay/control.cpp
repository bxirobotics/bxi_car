#include "relay/control.h"

#include <algorithm>
#include <deque>
#include <mutex>
#include <stdexcept>
#include <utility>

namespace chassis
{

struct CanRelay::Inbox
{
  std::mutex mutex;
  std::deque<iswv::CanFrame> frames;
  std::string error;
};

CanRelay::CanRelay(std::shared_ptr<iswv::ICanTransport> transport)
: CanRelay(std::move(transport), Config{}) {}

CanRelay::CanRelay(std::shared_ptr<iswv::ICanTransport> transport, Config config)
: transport_(std::move(transport)), config_(config), inbox_(std::make_shared<Inbox>())
{
  if (!transport_ || config_.address < 1 || config_.address > 255 ||
    config_.channel < 1 || config_.channel > 64 ||
    config_.timeout < std::chrono::milliseconds(20) ||
    config_.timeout > std::chrono::milliseconds(2000))
  {
    throw std::invalid_argument("relay requires transport, address 1..255, channel 1..64 and timeout 20..2000 ms");
  }
  // Capture shared storage, not this: a vendor callback may already have copied
  // the handler when destruction detaches it.
  transport_->set_receive_handler([inbox = inbox_, address = config_.address](
      const iswv::CanFrame & frame) {
      std::lock_guard<std::mutex> lock(inbox->mutex);
      if (frame.error) {inbox->error = "CAN error frame"; return;}
      if (frame.extended || frame.fd || frame.remote || frame.bitrate_switch ||
        frame.error_state_indicator) {return;}
      const bool coil_reply = (frame.id == (0x200U | address) ||
        frame.id == (0x300U | address)) && frame.size == 3;
      const bool register_reply = frame.id == (0x700U | address) &&
        (frame.size == 4 || frame.size == 6);
      if (!coil_reply && !register_reply) {return;}
      if (inbox->frames.size() >= 256) {inbox->error = "feedback queue overflow"; return;}
      inbox->frames.push_back(frame);
    });
  transport_->set_error_handler([inbox = inbox_](const iswv::Error & error) {
      std::lock_guard<std::mutex> lock(inbox->mutex);
      inbox->error = "transport: " + error.message;
    });
}

CanRelay::~CanRelay()
{
  transport_->set_receive_handler({});
  transport_->set_error_handler({});
  // Keep the current output, including on disconnect, shutdown or failure.
}

void CanRelay::start(Clock::time_point now)
{
  if (stage_ != Stage::dormant) {return;}
  request(Stage::info, now);
}

bool CanRelay::toggle()
{
  if (!ready()) {return false;}
  desired_on_ = !desired_on_;
  return true;
}

void CanRelay::request(Stage stage, Clock::time_point now)
{
  {
    std::lock_guard<std::mutex> lock(inbox_->mutex);
    if (!inbox_->error.empty()) {fail(inbox_->error); return;}
    // No transaction IDs exist: serialize exchanges and discard old replies.
    inbox_->frames.clear();
  }
  if (!transport_->is_open()) {fail("transport closed"); return;}
  stage_ = stage;
  deadline_ = now + config_.timeout;
  iswv::CanFrame frame;
  if (stage == Stage::info || stage == Stage::mode || stage == Stage::settings) {
    frame.id = 0x700U | config_.address;
    frame.size = 2;
    frame.data[0] = stage == Stage::info ? 0x43 :
      (stage == Stage::settings ? 0x40 : config_.channel - 1);
    frame.data[1] = stage == Stage::mode ? 1 : 2;
  } else if (stage == Stage::write) {
    frame.id = 0x200U | config_.address;
    frame.size = 3;
    frame.data[0] = config_.channel - 1;
    frame.data[1] = target_value_ >> 8;
    frame.data[2] = target_value_ & 0xff;
  } else {
    frame.id = 0x300U | config_.address;
    frame.size = 1;
    frame.data[0] = config_.channel - 1;
  }
  const auto result = transport_->send(frame);
  if (!result) {fail("send: " + result.error().message);}
}

void CanRelay::update(Clock::time_point now)
{
  if (stage_ == Stage::dormant || failed()) {return;}
  std::deque<iswv::CanFrame> frames;
  std::string error;
  {
    std::lock_guard<std::mutex> lock(inbox_->mutex);
    frames.swap(inbox_->frames);
    error = inbox_->error;
  }
  if (!error.empty()) {fail(error); return;}
  if (!transport_->is_open()) {fail("transport closed"); return;}
  if (stage_ == Stage::idle) {
    if (desired_on_ != confirmed_on_) {request(Stage::read, now);}
    return;
  }
  if (now >= deadline_) {fail("response timeout"); return;}
  const bool registers = stage_ == Stage::info || stage_ == Stage::mode || stage_ == Stage::settings;
  const auto id = (registers ? 0x700U :
    (stage_ == Stage::write ? 0x200U : 0x300U)) | config_.address;
  const auto register_start = stage_ == Stage::info ? 0x43U :
    (stage_ == Stage::settings ? 0x40U : config_.channel - 1);
  const auto register_count = stage_ == Stage::mode ? 1U : 2U;
  const auto response = std::find_if(frames.begin(), frames.end(), [&](const auto & frame) {
      return frame.id == id && (registers ?
             frame.size == 2 + register_count * 2 && frame.data[0] == register_start &&
             frame.data[1] == register_count :
             frame.size == 3 && frame.data[0] == config_.channel - 1);
    });
  if (response == frames.end()) {return;}
  const auto word = [&](unsigned offset) {
      return (unsigned(response->data[offset]) << 8) | response->data[offset + 1];
    };
  switch (stage_) {
    case Stage::info:
      if (word(2) != config_.address) {fail("module address mismatch"); return;}
      if (word(4) != 13) {fail("module bitrate is not 1M (register 0x44 must be 13)"); return;}
      request(Stage::mode, now);
      break;
    case Stage::mode:
      if (word(2) != 0) {fail("selected channel is not in ordinary mode"); return;}
      request(Stage::settings, now);
      break;
    case Stage::settings:
      if (word(2) != 0) {fail("write acknowledgements are masked (register 0x40)"); return;}
      if (word(4) != 0) {fail("communication watchdog is enabled (register 0x41)"); return;}
      request(Stage::read, now);
      break;
    case Stage::read: {
      target_on_ = desired_on_;
      const bool energized = target_on_ == config_.active_high;
      if ((word(1) != 0) == energized) {complete(); break;}
      target_value_ = energized ? 0xff00 : 0;
      request(Stage::write, now);
      break;
    }
    case Stage::write:
      if (word(1) != target_value_) {fail("write echo mismatch"); return;}
      request(Stage::verify, now);
      break;
    case Stage::verify:
      if ((word(1) != 0) != (target_value_ != 0)) {fail("output readback mismatch"); return;}
      complete();
      break;
    default: break;
  }
}

void CanRelay::complete()
{
  initialized_ = true;
  confirmed_on_ = target_on_;
  stage_ = Stage::idle;
  report_ = "CAN_RELAY address=" + std::to_string(config_.address) + " output=CH" +
    std::to_string(config_.channel) + " state=" + (confirmed_on_ ? "ON" : "OFF") +
    " confirmed by output readback";
}

void CanRelay::fail(const std::string & reason)
{
  stage_ = Stage::failed;
  report_ = "CAN_RELAY state=UNKNOWN: " + reason + "; output held, control disabled";
}

std::string CanRelay::take_report()
{
  return std::exchange(report_, {});
}

}  // namespace chassis
