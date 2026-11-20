#include "example/control_session.h"
#include "example/shared_control_server.h"

#include <chrono>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <signal.h>
#include <stdexcept>
#include <string>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>

namespace
{
using Session = chassis::RemoteControlSession;
using Stage = Session::Stage;
using Source = Session::Source;

void require(bool condition, const char * message)
{
  if (!condition) {throw std::runtime_error(message);}
}

template<typename Function>
void rejects(Function function)
{
  try {function();} catch (const std::exception &) {return;}
  throw std::runtime_error("expected operation to be rejected");
}

class TemporaryDirectory
{
public:
  TemporaryDirectory()
  {
    char pattern[] = "/tmp/car2-shared-control-XXXXXX";
    const auto value = ::mkdtemp(pattern);
    require(value != nullptr, "cannot create temporary shared-control directory");
    path = value;
  }
  ~TemporaryDirectory() {std::filesystem::remove_all(path);}
  std::filesystem::path path;
};

std::int64_t now_ms()
{
  return std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

int local_socket(const std::string & path, bool listening = false)
{
  const int fd = ::socket(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0);
  require(fd >= 0, "cannot create temporary Unix socket");
  sockaddr_un address{};
  address.sun_family = AF_UNIX;
  require(path.size() < sizeof(address.sun_path), "test socket path too long");
  std::memcpy(address.sun_path, path.c_str(), path.size() + 1);
  const int result = listening ?
    ::bind(fd, reinterpret_cast<sockaddr *>(&address), sizeof(address)) :
    ::connect(fd, reinterpret_cast<sockaddr *>(&address), sizeof(address));
  if (result != 0) {::close(fd); throw std::runtime_error("cannot bind/connect test socket");}
  if (listening) {require(::listen(fd, 4) == 0, "cannot listen on test socket");}
  return fd;
}

std::string receive(chassis::SharedControlServer & server, int fd)
{
  server.poll();
  std::string result;
  char bytes[2048];
  for (unsigned tries = 0; tries < 100 && result.find('\n') == std::string::npos; ++tries) {
    const auto count = ::recv(fd, bytes, sizeof(bytes), MSG_DONTWAIT);
    if (count > 0) {result.append(bytes, static_cast<std::size_t>(count));}
    else if (count == 0) {break;}
    server.poll();
  }
  require(result.find('\n') != std::string::npos, "server did not respond without blocking");
  return result;
}

std::string request(
  chassis::SharedControlServer & server, const std::string & path,
  const std::string & action, std::int64_t deadline = 0)
{
  const int fd = local_socket(path);
  const auto line = "v1 " + action + " " + std::to_string(deadline ? deadline : now_ms() + 5000) + "\n";
  require(::send(fd, line.data(), line.size(), MSG_NOSIGNAL) == static_cast<ssize_t>(line.size()),
    "cannot write shared control request");
  const auto result = receive(server, fd);
  ::close(fd);
  return result;
}

void request_from_other_process(
  chassis::SharedControlServer & server, const std::string & path, const std::string & action)
{
  const auto child = ::fork();
  require(child >= 0, "cannot fork separate IPC client");
  if (child == 0) {
    try {
      const int fd = local_socket(path);
      const auto line = "v1 " + action + " " + std::to_string(now_ms() + 2000) + "\n";
      require(::send(fd, line.data(), line.size(), MSG_NOSIGNAL) == static_cast<ssize_t>(line.size()),
        "cannot write separate-process request");
      char bytes[2048];
      const auto count = ::recv(fd, bytes, sizeof(bytes), 0);
      ::close(fd);
      require(count > 0 && std::string(bytes, count).find("\"ok\":true") != std::string::npos,
        "separate-process request failed");
      ::_exit(0);
    } catch (...) {::_exit(1);}
  }
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  int status{};
  while (std::chrono::steady_clock::now() < deadline) {
    server.poll();
    if (::waitpid(child, &status, WNOHANG) == child) {
      require(WIFEXITED(status) && WEXITSTATUS(status) == 0, "separate-process client failed");
      return;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  ::kill(child, SIGKILL);
  ::waitpid(child, &status, 0);
  throw std::runtime_error("separate-process client timed out");
}

void session_transitions()
{
  Session session;
  require(session.start_chassis && session.stage == Stage::waiting && !session.shared_control,
    "legacy defaults must remain unchanged");
  int handovers = 0;
  session.on_owner_changed = [&] {++handovers;};
  require(!session.request_start(Source::none), "a source is required to start");
  require(session.request_start(Source::remote), "remote start was rejected");
  const auto requested_generation = session.generation;
  require(session.request_start(Source::app) && session.startup_source == Source::remote &&
    session.generation == requested_generation, "duplicate start must preserve the original request");
  session.stage = Stage::starting;
  session.hardware_active = true;
  session.request_stop();
  rejects([&] {session.mark_running();});
  require(session.stage == Stage::stopping && !session.request_start(Source::app),
    "start must not supersede an in-progress stop");
  session.finish_stop();
  require(!session.hardware_active && session.stage == Stage::waiting, "stop did not reach idle");
  require(session.request_start(Source::app), "App could not restart the stopped session");
  session.mark_running();
  const auto app_generation = session.generation;
  require(session.owner == Source::app && session.hardware_active, "startup owner was not applied");
  require(session.claim(Source::remote) && session.generation > app_generation,
    "remote handover must invalidate the previous command generation");
  const auto remote_generation = session.generation;
  session.release(Source::app);
  require(session.owner == Source::remote && session.generation == remote_generation,
    "a non-owner must not cancel the current source");
  require(session.claim(Source::remote) && session.generation == remote_generation,
    "owner heartbeat must not change ownership");
  session.release(Source::remote);
  require(session.owner == Source::none && session.stage == Stage::running,
    "release must stop motion without selecting another owner");
  require(session.claim(Source::app), "explicit App reclaim was rejected");
  session.latch_fault("test fault");
  session.request_stop();
  session.finish_stop();
  require(session.stage == Stage::fault && !session.hardware_active &&
    !session.request_start(Source::app) && !session.claim(Source::remote),
    "a fault must remain latched after cleanup");
  require(session.last_error == "test fault" && handovers >= 5, "handover/fault callbacks were lost");
}

void socket_protocol()
{
  TemporaryDirectory directory;
  const auto path = (directory.path / "control.sock").string();
  auto session = std::make_shared<Session>();
  session->shared_control = true;
  {
    chassis::SharedControlServer server(session, path, true);
    struct stat info{};
    require(::stat(path.c_str(), &info) == 0 && (info.st_mode & 0777) == 0600,
      "management socket must only be accessible to its owner");
    const auto status = request(server, path, "status");
    require(status.find("\"ok\":true") != std::string::npos &&
      status.find("\"stage\":\"waiting\"") != std::string::npos &&
      status.find("\"pid\":" + std::to_string(::getpid())) != std::string::npos,
      "status must identify the actual idle controller");
    require(session->stage == Stage::waiting && session->generation == 0,
      "status must not request hardware startup");
    rejects([&] {chassis::SharedControlServer second(session, path, true);});
    require(request(server, path, "status").find("\"ok\":true") != std::string::npos,
      "a duplicate instance must not remove the original endpoint");
    require(request(server, path, "start", now_ms() - 1).find("\"ok\":false") != std::string::npos,
      "expired start request must fail");
    require(request(server, path, "start", now_ms() + 20000).find("\"ok\":false") != std::string::npos,
      "unbounded future requests must fail");
    require(request(server, path, "unknown").find("\"ok\":false") != std::string::npos,
      "unknown actions must fail");
    require(request(server, path, "claim-app").find("\"ok\":false") != std::string::npos,
      "App cannot claim idle hardware");
    require(session->stage == Stage::waiting, "bad requests changed lifecycle state");

    const int incomplete = local_socket(path);
    require(::send(incomplete, "v1 start ", 9, MSG_NOSIGNAL) == 9, "cannot send partial request");
    const auto began = std::chrono::steady_clock::now();
    for (unsigned i = 0; i < 100; ++i) {server.poll();}
    require(std::chrono::steady_clock::now() - began < std::chrono::milliseconds(100),
      "partial clients must not block the hardware loop");
    require(session->stage == Stage::waiting, "partial start was executed");
    ::close(incomplete);
    require(request(server, path, "start").find("\"ok\":true") != std::string::npos &&
      session->stage == Stage::requested && session->startup_source == Source::app &&
      !session->hardware_active, "start may request but must not initialize hardware");
    const auto generation = session->generation;
    request(server, path, "start");
    require(session->generation == generation, "duplicate IPC start must be idempotent");
    session->stage = Stage::starting;
    session->hardware_active = true;
    request(server, path, "stop");
    require(session->stage == Stage::stopping && session->hardware_active,
      "stop acceptance must not falsely report hardware cleanup complete");
    session->finish_stop();
    session->request_start(Source::remote);
    session->mark_running();
    require(request(server, path, "claim-app").find("\"owner\":\"app\"") != std::string::npos,
      "explicit App claim did not transfer ownership");
    session->latch_fault("fault with \"quotes\"\nand newline");
    session->finish_stop();
    require(request(server, path, "start").find("\"ok\":false") != std::string::npos,
      "IPC must not restart a latched fault");
    require(request(server, path, "status").find("\\\"quotes\\\" and newline") != std::string::npos,
      "error text must remain valid single-line JSON");
  }
  require(!std::filesystem::exists(path), "server did not remove its own socket");
  {
    chassis::SharedControlServer replacement(std::make_shared<Session>(), path, true);
  }
  std::ofstream(path) << "keep this file";
  rejects([&] {chassis::SharedControlServer replacement(session, path, true);});
  require(std::filesystem::file_size(path) == 14, "regular files must never be replaced");
  std::filesystem::remove(path);
  const int live = local_socket(path, true);
  rejects([&] {chassis::SharedControlServer replacement(session, path, true);});
  require(std::filesystem::exists(path), "unmanaged live socket must remain intact");
  ::close(live);
  // The now-closed socket is stale and can be safely replaced while holding the lock.
  chassis::SharedControlServer replacement(session, path, true);
}

void clean_stop_requires_a_fresh_process()
{
  TemporaryDirectory directory;
  const auto path = (directory.path / "control.sock").string();
  auto session = std::make_shared<Session>();
  chassis::SharedControlServer server(session, path, true);
  session->request_start(Source::remote);
  session->mark_running();
  request(server, path, "stop");
  session->finish_stop(true);
  const auto status = request(server, path, "status");
  require(status.find("\"hardware_active\":false") != std::string::npos &&
    status.find("\"restart_required\":true") != std::string::npos,
    "stop completion must be visible before process recycling");
  require(!session->request_start(Source::remote), "remote cannot reopen a used vendor runtime");
  require(request(server, path, "start").find("\"ok\":false") != std::string::npos,
    "App cannot reopen a used vendor runtime");
  request(server, path, "stop");
  session->finish_stop();
  require(session->restart_required && !session->request_start(Source::app),
    "duplicate cleanup must not rearm the old process");
  Session fresh_process;
  require(fresh_process.stage == Stage::waiting && !fresh_process.hardware_active &&
    fresh_process.startup_source == Source::none && !fresh_process.restart_required,
    "replacement process must wait for a new explicit start");
}

void stop_confirmation_requires_delivery_to_the_requesting_process()
{
  TemporaryDirectory directory;
  const auto path = (directory.path / "control.sock").string();
  auto session = std::make_shared<Session>();
  chassis::SharedControlServer server(session, path, true);
  require(!server.stop_confirmation_pending(), "idle server must not await a stop confirmation");
  request(server, path, "stop");
  require(!server.stop_confirmation_pending(), "already stopped reply needs no later confirmation");
  session->request_start(Source::remote);
  session->mark_running();
  session->request_stop();
  require(!server.stop_confirmation_pending(), "remote stop must not wait for an IPC client");
  request(server, path, "stop");
  require(server.stop_confirmation_pending(), "accepted active stop must keep the process alive");
  request(server, path, "status");
  require(server.stop_confirmation_pending(), "active status cannot confirm hardware shutdown");
  session->finish_stop(true);
  require(server.stop_confirmation_pending(), "cleanup alone cannot confirm delivery to the client");
  request_from_other_process(server, path, "status");
  request_from_other_process(server, path, "stop");
  require(server.stop_confirmation_pending(), "another PID cannot acknowledge the stopping client");
  request(server, path, "status", now_ms() - 1);
  request(server, path, "unknown");
  require(server.stop_confirmation_pending(), "rejected requests cannot acknowledge a stop");

  const int abandoned = local_socket(path);
  const auto line = "v1 status " + std::to_string(now_ms() + 5000) + "\n";
  require(::send(abandoned, line.data(), line.size(), MSG_NOSIGNAL) == static_cast<ssize_t>(line.size()),
    "cannot write abandoned final-status request");
  ::close(abandoned);
  server.poll();
  require(server.stop_confirmation_pending(), "failed response delivery must retain the stop confirmation");
  const auto final = request(server, path, "status");
  require(final.find("\"hardware_active\":false") != std::string::npos &&
    !server.stop_confirmation_pending(), "delivered final status must release the original process");
}

void stop_confirmation_expires_and_fault_stop_can_confirm()
{
  TemporaryDirectory directory;
  const auto path = (directory.path / "control.sock").string();
  auto session = std::make_shared<Session>();
  chassis::SharedControlServer server(session, path, true, std::chrono::milliseconds(50));
  session->request_start(Source::app);
  session->mark_running();
  request(server, path, "stop");
  require(server.stop_confirmation_pending(), "active stop was not tracked");
  session->finish_stop();
  std::this_thread::sleep_for(std::chrono::milliseconds(60));
  require(!server.stop_confirmation_pending(), "a vanished client must have a bounded confirmation window");
  server.poll();
  session->request_start(Source::app);
  session->mark_running();
  session->latch_fault("cleanup still pending");
  request(server, path, "stop");
  request(server, path, "status");
  require(server.stop_confirmation_pending(), "fault with active hardware is not shutdown confirmation");
  session->finish_stop();
  request(server, path, "stop");
  require(!server.stop_confirmation_pending(), "delivered inactive fault stop must confirm cleanup");
}
}  // namespace

int main()
{
  try {
    session_transitions();
    socket_protocol();
    clean_stop_requires_a_fresh_process();
    stop_confirmation_requires_delivery_to_the_requesting_process();
    stop_confirmation_expires_and_fault_stop_can_confirm();
    std::cout << "PASS: shared lifecycle, ownership, bounded local IPC and fault latching\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
