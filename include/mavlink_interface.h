#pragma once

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/shared_array.hpp>
#include <boost/system/system_error.hpp>
#include <thread>
#include <vector>

#include <math.h>
#include <stdio.h>
#include <iostream>
#include <string>

#include <ardupilotmega/mavlink.h>

static constexpr auto kDefaultDevice = "/dev/ttyTHS1";
static constexpr auto kDefaultBaudRate = 9600;
//! Maximum buffer size with padding for CRC bytes (280 + padding)
static constexpr ssize_t MAX_SIZE = MAVLINK_MAX_PACKET_LEN + 16;
static constexpr size_t MAX_TXQ_SIZE = 1000;

class MavlinkInterface {
 public:
MavlinkInterface();
~MavlinkInterface();

 // void send_mavlink_message(const mavlink_message_t *message);
  void open();
  void Load();
  void close();
  float getIMU();

  bool armed_;

 // void handle_message(mavlink_message_t *msg, bool &received_actuator);
  // Serial interface
private:
//  MavlinkInterface();
// ~MavlinkInterface();
  void do_read();
  void parse_buffer(const boost::system::error_code &err, std::size_t bytes_t);
  boost::asio::io_service io_service;
  boost::asio::serial_port serial_dev;

  bool enable_lockstep_;
  float armack;

  // Serial interface
  mavlink_status_t m_status;
  mavlink_message_t m_buffer;
  bool serial_enabled_ = true;
  std::thread io_thread;
  std::string device_;

  std::array<uint8_t, MAX_SIZE> rx_buf;
  unsigned int baudrate_;

};
