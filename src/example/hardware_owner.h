#pragma once

#include <cerrno>
#include <cstring>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <fcntl.h>
#include <sys/file.h>
#include <unistd.h>

namespace chassis
{
// Held from before power-on until after controller destruction. Keep the lock
// inode in place so two launch paths cannot acquire different replacement files.
class HardwareOwner
{
public:
  explicit HardwareOwner(const std::string & path = "/run/car2/hardware.lock")
  {
    std::filesystem::create_directories(std::filesystem::path(path).parent_path());
    fd_ = ::open(path.c_str(), O_CREAT | O_RDWR | O_CLOEXEC | O_NOFOLLOW, 0600);
    if (fd_ < 0) {throw std::runtime_error("cannot open hardware owner lock: " +
        std::string(std::strerror(errno)));}
    if (::flock(fd_, LOCK_EX | LOCK_NB) != 0) {
      ::close(fd_);
      fd_ = -1;
      throw std::runtime_error("another controller owns the motor hardware");
    }
  }
  ~HardwareOwner() {if (fd_ >= 0) {::close(fd_);}}
  HardwareOwner(const HardwareOwner &) = delete;
  HardwareOwner & operator=(const HardwareOwner &) = delete;
private:
  int fd_{-1};
};
}  // namespace chassis
