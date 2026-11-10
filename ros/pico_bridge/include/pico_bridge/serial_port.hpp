#pragma once

#include <string>

namespace pico_bridge {

// A raw, non-blocking serial port (the Pico's USB serial, /dev/ttyACM*).
class SerialPort {
 public:
  explicit SerialPort(const std::string& path);  // throws on failure
  ~SerialPort();
  SerialPort(const SerialPort&) = delete;
  SerialPort& operator=(const SerialPort&) = delete;

  // Both return false once the port has failed.
  bool write_text(const std::string& text);
  bool read_available(std::string& buffer);  // appends what has arrived

 private:
  int fd_;
};

}  // namespace pico_bridge
