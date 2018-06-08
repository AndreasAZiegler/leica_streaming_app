/**
 * \file tcp_total_station_interface.cpp
 * \author Andreas Ziegler
 * \date 31.05.2018
 * \brief Implementation of the tcp total station interface
 */

#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/algorithm/string.hpp>

#include "leica_streaming_app/tcp_total_station_interface.h"

TCPTSInterface::TCPTSInterface(std::function<void(const double, const double, const double)> locationCallback)
  : socket_(new boost::asio::ip::tcp::socket(*io_context_)),
    TSInterface(locationCallback) {}

TCPTSInterface::~TCPTSInterface() {
  socket_->shutdown(boost::asio::ip::tcp::socket::shutdown_both);
  socket_->close();
  contextThread_.join();
}

void TCPTSInterface::connect(std::string ip, int port) {
  boost::asio::ip::tcp::endpoint endpoint(boost::asio::ip::address::from_string(ip),
                                          port);
  try {
    // Connect to total station and call startReader and startTimer
    // if successfull
    socket_->async_connect(endpoint,
          [this](const boost::system::error_code& ec) {
            if (!ec) {
              startReader();
            }
          });

    // Start io_context in separate thread
    contextThread_ = std::thread([this](){ io_context_->run(); });
  } catch (std::exception& e) {
    std::cerr << "Exception: " << e.what() << "\n";
  }
}

void TCPTSInterface::startReader() {
  boost::asio::async_read_until(*socket_,
                                readData_,
                                "\r\n",
                                std::bind(&TCPTSInterface::readHandler,
                                          this,
                                          std::placeholders::_1,
                                          std::placeholders::_2)
                                );
}

void TCPTSInterface::write(std::vector<char> command) {
  boost::asio::async_write(*socket_,
                           boost::asio::buffer(command),
                           std::bind(&TCPTSInterface::writeHandler,
                                     this,
                                     std::placeholders::_1,
                                     std::placeholders::_2)
                          );
}

void TCPTSInterface::writeHandler(const boost::system::error_code& ec,
                                  std::size_t bytes_transferred) {
  if (!ec) {
    std::cout << "Command sent." << std::endl;
  }
}

void TCPTSInterface::readHandler(const boost::system::error_code& ec,
                                 std::size_t bytes_transferred) {
  if (!ec) {
    // Convert streambuf to std::string
    boost::asio::streambuf::const_buffers_type bufs = readData_.data();
    std::string data(boost::asio::buffers_begin(bufs),
                     boost::asio::buffers_begin(bufs) + readData_.size());

    readData_.consume(bytes_transferred);

    // Print received message
    // std::cout << data << std::endl;

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
    boost::asio::async_read_until(*socket_,
                                  readData_,
                                  "\r\n",
                                  std::bind(&TCPTSInterface::readHandler,
                                            this,
                                            std::placeholders::_1,
                                            std::placeholders::_2)
                                 );
  }
}
