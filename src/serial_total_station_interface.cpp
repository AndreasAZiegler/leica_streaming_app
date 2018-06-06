/**
 * \file tcp_total_station_interface.cpp
 * \author Andreas Ziegler
 * \date 31.05.2018
 * \brief Implementation of the tcp total station interface
 */

#include <iostream>
#include <string>
#include <thread>

#include <boost/date_time/posix_time/posix_time.hpp>

#include "leica_streaming_app/tcp_total_station_interface.h"

SerialTSInterface::SerialTSInterface(std::function<void(const double, const double, const double)> locationCallback)
  : io_context_(new boost::asio::io_service()),
    serial_port_(*io_context_),
    locationCallback_(locationCallback),
    timer_(*io_context_, boost::posix_time::seconds(2)),
    timerStartedFlag_(false),
    searchingPrismFlag_(false),
    tsState_(TSState::on),
    prismPosition_(3),
    TSInterface() {}

SerialTSInterface::~SerialTSInterface() {
  serial_port_.cancel();
  serial_port_.close();
  contextThread_.join();
}

void SerialTSInterface::connect(std::string comport) {
  try {
    // Connect to total station and call startReader and startTimer
    // if successfull
    boost::system::error_code ec;

    // what baud rate do we communicate at
    serial_port_base::baud_rate BAUD(151200);
    // how big is each "packet" of data (default is 8 bits)
    serial_port_base::character_size C_SIZE( 8 );
    // what flow control is used (default is none)
    serial_port_base::flow_control FLOW( serial_port_base::flow_control::none );
    // what parity is used (default is none)
    serial_port_base::parity PARITY( serial_port_base::parity::none );
    // how many stop bits are used (default is one)
    serial_port_base::stop_bits STOP( serial_port_base::stop_bits::one );

    serial_port_.set_option(BAUD);
    serial_port_.set_option(C_SIZE);
    serial_port_.set_option(FLOW);
    serial_port_.set_option(PARITY);
    serial_port_.set_option(STOP);

    serial_port_.open(comport, &ec);

    if (!ec) {
      startReader();
    }

    // Start io_context in separate thread
    contextThread_ = std::thread([this](){ io_context_->run(); });
  } catch (std::exception& e) {
    std::cerr << "Exception: " << e.what() << "\n";
  }
}

void SerialTSInterface::start() {
  std::vector<char> command{'%', 'R', '8', 'Q', ',', '1', ':', 0x0d/*CR*/, 0x0a/*LF*/};
  write(command);
  tsState_ = TSState::on;
}

void SerialTSInterface::end() {
  std::vector<char> command {'%', 'R', '8', 'Q', ',', '2', ':', 0x0d/*CR*/, 0x0a/*LF*/};
  write(command);
  tsState_ = TSState::off;
}

void SerialTSInterface::setPrismPosition(double x, double y, double z) {
  prismPosition_[0] = x;
  prismPosition_[1] = y;
  prismPosition_[2] = z;
}

void SerialTSInterface::startReader() {
  boost::asio::async_read_until(serial_port_,
                                readData_,
                                "\r\n",
                                std::bind(&TCPTSInterface::readHandler,
                                          this,
                                          std::placeholders::_1,
                                          std::placeholders::_2)
                                );
}

void SerialTSInterface::write(std::vector<char> command) {
  boost::asio::async_write(serial_port_,
                           boost::asio::buffer(command),
                           std::bind(&TCPTSInterface::writeHandler,
                                     this,
                                     std::placeholders::_1,
                                     std::placeholders::_2)
                          );
}

void SerialTSInterface::startTimer() {
  std::cout << "Start timer" << std::endl;
  timer_.expires_at(timer_.expires_at() + boost::posix_time::seconds(2));
  timer_.async_wait(std::bind(&TCPTSInterface::timerHandler, this));
}

void SerialTSInterface::readHandler(const boost::system::error_code& ec,
                                 std::size_t bytes_transferred) {
  if (!ec) {
    // Convert streambuf to std::string
    boost::asio::streambuf::const_buffers_type bufs = readData_.data();
    std::string data(boost::asio::buffers_begin(bufs),
                     boost::asio::buffers_begin(bufs) + readData_.size());

    readData_.consume(bytes_transferred);

    // Print received message
    //std::cout << data << std::endl;

    
    // Check for responses if the total station searches the prism
    if (searchingPrismFlag_) {
      // Catch the response
      if (data.find("%R8P,0,0:") != std::string::npos) {
        std::cout << "Got an answer." << std::endl;
        
        // Catch the negative response
        if (data.find(":31") != std::string::npos) {
          std::cout << "Prism not found!" << std::endl;
          searchPrism();
        } else if (data.find(":0") != std::string::npos) { // Catch the positive response
          std::cout << "Prism found" << std::endl;
          {
            std::lock_guard<std::mutex> guard(searchingPrismMutex_);
            searchingPrismFlag_ = false;
          }
          {
            std::lock_guard<std::mutex> guard1(messageReceivedMutex_);
            messagesReceivedFlag_ = true;
          }
        }
      }
    } else if (data[0] == 'T') { // Forward x, y and z coordinate if location was received
      // Split the received message to access the coordinates
      std::vector<std::string> results;
      boost::split(results, data, [](char c){return c == ',';});

      double x = std::stod(results[1]);
      double y = std::stod(results[2]);
      double z = std::stod(results[3]);

      locationCallback_(x, y, z);

      // Indicate that a message was received
      std::lock_guard<std::mutex> guard(messageReceivedMutex_);
      messagesReceivedFlag_ = true;

      // Start timer if it is not yet started
      if (!timerStartedFlag_) {
        startTimer();
        timerStartedFlag_ = true;
      }
    }

    // Restart reading
    boost::asio::async_read_until(serial_port_,
                                  readData_,
                                  "\r\n",
                                  std::bind(&TCPTSInterface::readHandler,
                                            this,
                                            std::placeholders::_1,
                                            std::placeholders::_2)
                                 );
  }
}

void SerialTSInterface::writeHandler(const boost::system::error_code& ec,
                                  std::size_t bytes_transferred) {
  if (!ec) {
    std::cout << "Command sent." << std::endl;
  }
}

void SerialTSInterface::timerHandler() {
  {
    if (TSState::on == tsState_) {
      // Check if a message was received since last time.
      std::lock_guard<std::mutex> guard1(messageReceivedMutex_);
      std::lock_guard<std::mutex> guard2(searchingPrismMutex_);

      // Start to search the prism if no message was received
      if (!messagesReceivedFlag_ && !searchingPrismFlag_) {
        std::cout << "Prism lost!" << std::endl;
        searchPrism();
      }

      // Reset flag
      messagesReceivedFlag_ = false;
    }
  }

  // Restart timer
  timer_.expires_at(timer_.expires_at() + boost::posix_time::milliseconds(800));
  timer_.async_wait(std::bind(&TCPTSInterface::timerHandler, this));
}

void SerialTSInterface::searchPrism(void) {
  searchingPrismFlag_ = true;
  std::vector<char> command {'%', 'R', '8', 'Q', ',', '6', ':', '1', 0x0d/*CR*/, 0x0a/*LF*/};
  write(command);

  std::cout << "Search prism" << std::endl;
}
