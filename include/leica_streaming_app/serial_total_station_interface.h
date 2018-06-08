/**
 * \file tcp_total_station_interface.h
 * \author Andreas Ziegler
 * \date 31.05.2018
 * \brief Header file containing the required defintion for the tcp total station interface
 */

#pragma once

#include <vector>
#include <string>
#include <memory>
#include <thread>
#include <mutex>

#include <boost/asio.hpp>
#include <boost/algorithm/string.hpp>

#include "total_station_interface.h"

/**
 * @brief TCP interface for the Leica total station
 */
class SerialTSInterface: public TSInterface {
 public:
  /**
   * @brief Constructor
   *
   * @param f Callback function to receive the x, y and z location of the tracked prism.
   */
  explicit SerialTSInterface(std::function<void(const double, const double, const double)> locationCallback);

  /**
   * @brief Close socket and join io_context thread.
   */
  ~SerialTSInterface();

  /**
   * @brief Tries to open a tcp connection to the total station
   *        Initializes the tcp connection to the total station
   *        and the timer to detect if no message are received anymore.
   *
   * @param ip The ip address as std::string
   * @param port The port number as int
   */
  void connect(std::string comport);

 private:
  /**
   * @brief Starts the tcp reader.
   */
  void startReader();

  /**
   * @brief Sends the total station a command.
   *
   * @param command std::vector<char> with the command.
   */
  void write(std::vector<char> str) override;

  /**
   * @brief Callback method when a message was sent
   *
   * @param ec Error code
   * @param bytes_transferred Amount of bytes received
   */
  void writeHandler(const boost::system::error_code& ec,
                    std::size_t bytes_transferred);

  /**
   * @brief Callback method when a message was received.
   *        Calls the registered callback function with the
   *        x, y and z coordinates of the prism. Also sets flag
   *        to indicate that a message was received.
   *
   * @param ec error code
   * @param bytes_transferred Amount of bytes received
   */
  void readHandler(const boost::system::error_code& ec,
                   std::size_t bytes_transferred);

  /**
   * @brief Starts the prism search on the total station.
   */
  void searchPrism(void);

  boost::asio::serial_port serial_port_;                  /**< Serial port */

  std::thread contextThread_;                             /**< Thread in which the io context object runs */
};
