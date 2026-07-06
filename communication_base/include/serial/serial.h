#pragma once

#include <stdexcept>

#include <core/serial/serial.h>

namespace serial {
using ordlidar::core::serial::bytesize_t;
using ordlidar::core::serial::flowcontrol_t;
using ordlidar::core::serial::parity_t;
using ordlidar::core::serial::stopbits_t;
using ordlidar::core::serial::Timeout;
using ordlidar::core::serial::eightbits;
using ordlidar::core::serial::flowcontrol_none;
using ordlidar::core::serial::parity_none;
using ordlidar::core::serial::stopbits_one;

class Serial : public ordlidar::core::serial::Serial {
 public:
  using ordlidar::core::serial::Serial::Serial;

  void close() {
    closePort();
  }
};

class IOException : public std::runtime_error {
 public:
  explicit IOException(const std::string &message) : std::runtime_error(message) {}
};
}
