#include "example/shared_control_server.h"

#include <algorithm>
#include <cerrno>
#include <charconv>
#include <chrono>
#include <cstring>
#include <filesystem>
#include <fcntl.h>
#include <iomanip>
#include <map>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <sys/file.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <unistd.h>
#include <vector>

namespace chassis
{
namespace
{
using Clock = std::chrono::steady_clock;
constexpr std::size_t max_clients = 16;
constexpr std::size_t max_request_bytes = 256;
constexpr auto connection_lifetime = std::chrono::seconds(5);

std::int64_t monotonic_ms()
{
  return std::chrono::duration_cast<std::chrono::milliseconds>(
    Clock::now().time_since_epoch()).count();
}

std::runtime_error system_error(const char * action)
{
  return std::runtime_error(std::string(action) + ": " + std::strerror(errno));
}

std::string json_string(std::string value)
{
  value.resize(std::min<std::size_t>(value.size(), 512));
  for (auto & character : value) {
    if (static_cast<unsigned char>(character) < 0x20) {character = ' ';}
  }
  std::ostringstream result;
  result << std::quoted(value);
  return result.str();
}

bool same_file(const std::string & path, dev_t device, ino_t inode)
{
  struct stat info{};
  return ::lstat(path.c_str(), &info) == 0 && info.st_dev == device && info.st_ino == inode;
}
}  // namespace

struct SharedControlServer::Impl
{
  struct Client
  {
    int fd;
    pid_t peer_pid;
    Clock::time_point accepted;
    std::string request;
    std::string response;
    std::size_t sent{0};
    std::optional<Clock::time_point> stop_confirmation;
  };

  Impl(
    std::shared_ptr<RemoteControlSession> value, const std::string & path, bool arm,
    std::chrono::milliseconds confirmation_timeout)
  : session(std::move(value)), socket_path(path), arm_enabled(arm),
    stop_confirmation_timeout(confirmation_timeout)
  {
    if (!session) {throw std::invalid_argument("shared control session is missing");}
    if (confirmation_timeout <= std::chrono::milliseconds::zero() ||
      confirmation_timeout > std::chrono::seconds(30))
    {
      throw std::invalid_argument("stop confirmation timeout must be in (0, 30] seconds");
    }
    sockaddr_un address{};
    if (path.empty() || path.front() != '/' || path.find('\0') != std::string::npos ||
      path.size() >= sizeof(address.sun_path))
    {
      throw std::invalid_argument("shared control socket must be a short absolute filesystem path");
    }
    address.sun_family = AF_UNIX;
    std::memcpy(address.sun_path, path.c_str(), path.size() + 1);
    try {
      const auto parent = std::filesystem::path(path).parent_path();
      if (std::filesystem::create_directories(parent)) {
        std::filesystem::permissions(parent, std::filesystem::perms::owner_all);
      }
      const auto lock_path = path + ".lock";
      lock_fd = ::open(lock_path.c_str(), O_RDWR | O_CREAT | O_CLOEXEC | O_NOFOLLOW, 0600);
      if (lock_fd < 0) {throw system_error("open shared control lock");}
      struct stat lock_info{};
      if (::fstat(lock_fd, &lock_info) != 0) {throw system_error("inspect shared control lock");}
      if (!S_ISREG(lock_info.st_mode) || lock_info.st_nlink != 1 ||
        lock_info.st_uid != ::geteuid())
      {
        throw std::runtime_error("shared control lock has an unexpected type or owner");
      }
      if (::flock(lock_fd, LOCK_EX | LOCK_NB) != 0) {
        throw std::runtime_error("another shared controller owns this endpoint");
      }
      struct stat previous{};
      if (::lstat(path.c_str(), &previous) == 0) {
        if (!S_ISSOCK(previous.st_mode) || previous.st_uid != ::geteuid()) {
          throw std::runtime_error("refusing to replace a non-socket or foreign shared endpoint");
        }
        const int probe = ::socket(AF_UNIX, SOCK_STREAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
        if (probe < 0) {throw system_error("inspect existing shared endpoint");}
        const int connected = ::connect(probe, reinterpret_cast<sockaddr *>(&address), sizeof(address));
        const int error = errno;
        ::close(probe);
        if (connected == 0 || (error != ECONNREFUSED && error != ENOENT)) {
          throw std::runtime_error("refusing to replace a live shared control socket");
        }
        if (!same_file(path, previous.st_dev, previous.st_ino) || ::unlink(path.c_str()) != 0) {
          throw std::runtime_error("shared control socket changed while checking stale endpoint");
        }
      } else if (errno != ENOENT) {
        throw system_error("inspect shared control socket path");
      }
      listen_fd = ::socket(AF_UNIX, SOCK_STREAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
      if (listen_fd < 0) {throw system_error("create shared control socket");}
      if (::bind(listen_fd, reinterpret_cast<sockaddr *>(&address), sizeof(address)) != 0) {
        throw system_error("bind shared control socket");
      }
      struct stat created{};
      if (::lstat(path.c_str(), &created) != 0) {throw system_error("inspect bound shared socket");}
      socket_device = created.st_dev;
      socket_inode = created.st_ino;
      bound = true;
      if (::chmod(path.c_str(), 0600) != 0) {throw system_error("protect shared control socket");}
      if (::listen(listen_fd, static_cast<int>(max_clients)) != 0) {
        throw system_error("listen on shared control socket");
      }
    } catch (...) {
      close();
      throw;
    }
  }

  ~Impl() {close();}

  void close() noexcept
  {
    for (const auto & client : clients) {::close(client.fd);}
    clients.clear();
    if (listen_fd >= 0) {::close(listen_fd); listen_fd = -1;}
    if (bound && same_file(socket_path, socket_device, socket_inode)) {
      (void)::unlink(socket_path.c_str());
    }
    bound = false;
    // Keep the empty lock inode: unlinking a flock file can create two independent locks.
    if (lock_fd >= 0) {::close(lock_fd); lock_fd = -1;}
  }

  std::string reply(bool ok, const std::string & error = {}) const
  {
    std::ostringstream result;
    result << "{\"protocol\":1,\"ok\":" << (ok ? "true" : "false")
           << ",\"pid\":" << ::getpid()
           << ",\"stage\":\"" << stage_name(session->stage)
           << "\",\"owner\":\"" << source_name(session->owner)
           << "\",\"generation\":" << session->generation
           << ",\"hardware_active\":" << (session->hardware_active ? "true" : "false")
           << ",\"restart_required\":" << (session->restart_required ? "true" : "false")
           << ",\"arm_enabled\":" << (arm_enabled ? "true" : "false")
           << ",\"error\":" << json_string(error.empty() ? session->last_error : error) << "}\n";
    return result.str();
  }

  std::string terminal_reply(Client & client)
  {
    const auto pending = stop_confirmations.find(client.peer_pid);
    if (pending != stop_confirmations.end() && !session->hardware_active &&
      (session->stage == RemoteControlSession::Stage::waiting ||
      session->stage == RemoteControlSession::Stage::fault))
    {
      client.stop_confirmation = pending->second;
    }
    return reply(true);
  }

  std::string handle(const std::string & request, Client & client)
  {
    std::istringstream stream(request);
    std::string protocol, action, deadline_text, extra;
    if (!(stream >> protocol >> action >> deadline_text) || stream >> extra || protocol != "v1") {
      return reply(false, "expected v1 action deadline_monotonic_ms");
    }
    std::int64_t deadline{};
    const auto parsed = std::from_chars(
      deadline_text.data(), deadline_text.data() + deadline_text.size(), deadline);
    const auto now = monotonic_ms();
    if (parsed.ec != std::errc{} || parsed.ptr != deadline_text.data() + deadline_text.size() ||
      deadline <= now || deadline - now > 10000)
    {
      return reply(false, "expired or invalid shared control request deadline");
    }
    try {
      if (action == "status") {return terminal_reply(client);}
      if (action == "start") {
        if (!session->request_start(RemoteControlSession::Source::app)) {
          return reply(false, "controller is stopping, restarting or faulted; start rejected");
        }
      } else if (action == "stop") {
        session->request_stop();
        if (session->hardware_active) {
          // Repeated requests from one client do not extend its bounded grace period.
          stop_confirmations.try_emplace(
            client.peer_pid, Clock::now() + stop_confirmation_timeout);
        }
        return terminal_reply(client);
      } else if (action == "claim-app") {
        if (!session->claim(RemoteControlSession::Source::app)) {
          return reply(false, "controller must be ready before App can take control");
        }
      } else {
        return reply(false, "unknown shared control action");
      }
      return reply(true);
    } catch (const std::exception & error) {
      try {session->latch_fault(error.what());} catch (...) {}
      return reply(false, error.what());
    }
  }

  void poll()
  {
    const auto now = Clock::now();
    for (auto pending = stop_confirmations.begin(); pending != stop_confirmations.end();) {
      if (pending->second <= now) {pending = stop_confirmations.erase(pending);}
      else {++pending;}
    }
    // Bound both connection acceptance and per-client bytes on every hardware cycle.
    for (unsigned accepted = 0; accepted < 4 && clients.size() < max_clients; ++accepted) {
      const int fd = ::accept4(listen_fd, nullptr, nullptr, SOCK_NONBLOCK | SOCK_CLOEXEC);
      if (fd < 0) {
        if (errno == EINTR) {continue;}
        if (errno == EAGAIN || errno == EWOULDBLOCK) {break;}
        throw system_error("accept shared control connection");
      }
      struct ucred peer{};
      socklen_t size = sizeof(peer);
      if (::getsockopt(fd, SOL_SOCKET, SO_PEERCRED, &peer, &size) != 0 ||
        peer.pid <= 0 || (peer.uid != ::geteuid() && peer.uid != 0))
      {
        ::close(fd);
        continue;
      }
      clients.push_back({fd, peer.pid, Clock::now(), {}, {}, 0, {}});
    }
    for (auto item = clients.begin(); item != clients.end();) {
      bool done = Clock::now() - item->accepted >= connection_lifetime;
      if (!done && item->response.empty()) {
        char bytes[max_request_bytes + 1];
        const auto received = ::recv(item->fd, bytes, sizeof(bytes), MSG_DONTWAIT);
        if (received > 0) {
          item->request.append(bytes, static_cast<std::size_t>(received));
          const auto newline = item->request.find('\n');
          if (item->request.size() > max_request_bytes) {
            item->response = reply(false, "shared control request is too large");
          } else if (newline != std::string::npos) {
            item->response = newline + 1 == item->request.size() ?
              handle(item->request.substr(0, newline), *item) :
              reply(false, "one shared control request per connection is required");
          }
        } else if (received == 0 || (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR)) {
          done = true;
        }
      }
      if (!done && !item->response.empty()) {
        const auto sent = ::send(
          item->fd, item->response.data() + item->sent, item->response.size() - item->sent,
          MSG_DONTWAIT | MSG_NOSIGNAL);
        if (sent > 0) {
          item->sent += static_cast<std::size_t>(sent);
          done = item->sent == item->response.size();
          if (done && item->stop_confirmation) {
            const auto pending = stop_confirmations.find(item->peer_pid);
            if (pending != stop_confirmations.end() &&
              pending->second == *item->stop_confirmation)
            {
              stop_confirmations.erase(pending);
            }
          }
        } else if (sent == 0 || (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR)) {
          done = true;
        }
      }
      if (done) {
        ::close(item->fd);
        item = clients.erase(item);
      } else {
        ++item;
      }
    }
  }

  std::shared_ptr<RemoteControlSession> session;
  std::string socket_path;
  bool arm_enabled;
  std::chrono::milliseconds stop_confirmation_timeout;
  std::map<pid_t, Clock::time_point> stop_confirmations;
  int listen_fd{-1};
  int lock_fd{-1};
  bool bound{false};
  dev_t socket_device{};
  ino_t socket_inode{};
  std::vector<Client> clients;
};

SharedControlServer::SharedControlServer(
  std::shared_ptr<RemoteControlSession> session, const std::string & path, bool arm_enabled,
  std::chrono::milliseconds stop_confirmation_timeout)
: impl_(std::make_unique<Impl>(std::move(session), path, arm_enabled, stop_confirmation_timeout)) {}

SharedControlServer::~SharedControlServer() = default;

void SharedControlServer::poll() {impl_->poll();}

bool SharedControlServer::stop_confirmation_pending() const
{
  const auto now = Clock::now();
  return std::any_of(
    impl_->stop_confirmations.begin(), impl_->stop_confirmations.end(),
    [now](const auto & pending) {return pending.second > now;});
}
}  // namespace chassis
