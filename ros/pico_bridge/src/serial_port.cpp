#include "pico_bridge/serial_port.hpp"

#include <fcntl.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <stdexcept>

namespace pico_bridge {

SerialPort::SerialPort(const std::string& path)
    : fd_(open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK)) {
  if (fd_ < 0) {
    const std::string hint =
        errno == EBUSY ? " (is another pico_bridge already running?)" : "";
    throw std::runtime_error("can't open " + path + ": " +
                             std::strerror(errno) + hint);
  }
  termios settings{};
  if (tcgetattr(fd_, &settings) != 0) {
    close(fd_);
    throw std::runtime_error("can't read settings of " + path);
  }
  cfmakeraw(&settings);
  cfsetspeed(&settings, B115200);  // ignored by USB serial, but must be set
  settings.c_cflag |= CLOCAL | CREAD | HUPCL;
  if (tcsetattr(fd_, TCSANOW, &settings) != 0) {
    close(fd_);
    throw std::runtime_error("can't set up " + path);
  }
  tcflush(fd_, TCIOFLUSH);

  // Exclusive, so a second bridge can't split the data.
  if (ioctl(fd_, TIOCEXCL) != 0) {
    close(fd_);
    throw std::runtime_error("can't get exclusive use of " + path);
  }

  // The Pico only sends output once it sees DTR raised.
  int dtr = TIOCM_DTR;
  ioctl(fd_, TIOCMBIS, &dtr);
}

SerialPort::~SerialPort() { close(fd_); }

namespace {

bool is_would_block(int error) {
  return error == EAGAIN || error == EWOULDBLOCK;
}

// A hang-up usually shows first as EIO from write().
[[noreturn]] void throw_lost_link(int error) {
  throw std::runtime_error(
      std::string("lost the Pico (unplugged or reset?): ") +
      std::strerror(error));
}

}  // namespace

void SerialPort::write_text(const std::string& text) {
  // A full buffer (Pico not reading) isn't a lost link.
  if (write(fd_, text.data(), text.size()) < 0 && !is_would_block(errno)) {
    throw_lost_link(errno);
  }
}

void SerialPort::read_available(std::string& buffer) {
  char chunk[512];
  while (true) {
    const ssize_t count = read(fd_, chunk, sizeof(chunk));
    if (count > 0) {
      buffer.append(chunk, static_cast<size_t>(count));
    } else if (count == 0) {
      throw std::runtime_error("lost the Pico (unplugged or reset?): hang-up");
    } else if (is_would_block(errno)) {
      return;  // nothing more to read for now
    } else {
      throw_lost_link(errno);
    }
  }
}

}  // namespace pico_bridge
