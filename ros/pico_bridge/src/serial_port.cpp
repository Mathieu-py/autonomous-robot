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
  tcsetattr(fd_, TCSANOW, &settings);
  tcflush(fd_, TCIOFLUSH);

  // Exclusive, so a second bridge can't open the port and split the data.
  if (ioctl(fd_, TIOCEXCL) != 0) {
    close(fd_);
    throw std::runtime_error("can't get exclusive use of " + path);
  }

  // The Pico only sends output once it sees DTR raised.
  int dtr = TIOCM_DTR;
  ioctl(fd_, TIOCMBIS, &dtr);
}

SerialPort::~SerialPort() { close(fd_); }

bool SerialPort::write_text(const std::string& text) {
  return write(fd_, text.data(), text.size()) ==
         static_cast<ssize_t>(text.size());
}

bool SerialPort::read_available(std::string& buffer) {
  char chunk[512];
  while (true) {
    const ssize_t count = read(fd_, chunk, sizeof(chunk));
    if (count > 0) {
      buffer.append(chunk, static_cast<size_t>(count));
    } else if (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      return true;  // nothing more to read for now
    } else {
      return false;  // 0 is a hang-up: the Pico was unplugged or reset
    }
  }
}

}  // namespace pico_bridge
