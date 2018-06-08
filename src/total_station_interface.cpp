/**
 * \file total_station_interface.cpp
 * \author Andreas Ziegler
 * \date 08.06.2018
 * \brief Implementation of the total station interface base class
 */

#include <iostream>
#include <vector>

#include <boost/lexical_cast.hpp>

#include "leica_streaming_app/total_station_interface.h"

TSInterface::TSInterface(std::function<void(const double, const double, const double)> locationCallback)
  : io_context_(new boost::asio::io_service()),
    timer_(*io_context_, boost::posix_time::seconds(2)),
    timerStartedFlag_(false),
    searchingPrismFlag_(false),
    externalPositionReceivedFlag_(false),
    prismPosition_(3),
    tsState_(TSState::on),
    locationCallback_(locationCallback)
{}

void TSInterface::startTimer() {
  std::cout << "Start timer" << std::endl;
  timer_.expires_at(timer_.expires_at() + boost::posix_time::seconds(2));
  timer_.async_wait(std::bind(&TSInterface::timerHandler, this));
}

void TSInterface::start() {
  std::vector<char> command{'%', 'R', '8', 'Q', ',', '1', ':', 0x0d/*CR*/, 0x0a/*LF*/};
  write(command);
  tsState_ = TSState::on;
}

void TSInterface::end() {
  std::vector<char> command {'%', 'R', '8', 'Q', ',', '2', ':', 0x0d/*CR*/, 0x0a/*LF*/};
  write(command);
  tsState_ = TSState::off;
}

void TSInterface::setPrismPosition(double x, double y, double z) {
  prismPosition_[0] = x;
  prismPosition_[1] = y;
  prismPosition_[2] = z;

  // Set external position
  std::lock_guard<std::mutex> guard(externalPositionReceivedMutex_);
  externalPositionReceivedFlag_ = true;
}

void TSInterface::timerHandler() {
  {
    if (TSState::on == tsState_) {
      // Check if a message was received since last time.
      {
        std::lock_guard<std::mutex> guard1(messageReceivedMutex_);
        std::lock_guard<std::mutex> guard2(searchingPrismMutex_);

        // Start to search the prism if no message was received
        if (!messagesReceivedFlag_ && !searchingPrismFlag_) {
          std::cout << "Prism lost!" << std::endl;

          // Turn total station to prism if a recent external position was received
          {
            std::lock_guard<std::mutex> guard(externalPositionReceivedMutex_);
            if (externalPositionReceivedFlag_) {
                turnTelescope();
            }
          }

          // Turn total station to prism if a recent external position was received
          {
            std::lock_guard<std::mutex> guard(externalPositionReceivedMutex_);
            if (externalPositionReceivedFlag_) {
                turnTelescope();
            }
          }
          searchPrism();
        }

        // Reset flag
        messagesReceivedFlag_ = false;
      }

      // Reset external position
      {
        std::lock_guard<std::mutex> guard(externalPositionReceivedMutex_);
        externalPositionReceivedFlag_ = false;
      }
    }
  }

  // Restart timer
  timer_.expires_at(timer_.expires_at() + boost::posix_time::milliseconds(800));
  timer_.async_wait(std::bind(&TSInterface::timerHandler, this));
}

void TSInterface::searchPrism(void) {
  searchingPrismFlag_ = true;
  std::vector<char> command {'%', 'R', '8', 'Q', ',', '6', ':', '1', 0x0d/*CR*/, 0x0a/*LF*/};
  write(command);

  std::cout << "Search prism" << std::endl;
}

void TSInterface::turnTelescope(void) {
  searchingPrismFlag_ = true;
  std::vector<char> command {'%', 'R', '8', 'Q', ',', '7', ':', '1'};
  std::string y = boost::lexical_cast<std::string>(prismPosition_[1]);
  for (char c : y) {
    command.emplace_back(c);
  }
  std::string x = boost::lexical_cast<std::string>(prismPosition_[0]);
  for (char c : x) {
    command.emplace_back(c);
  }
  std::string z = boost::lexical_cast<std::string>(prismPosition_[2]);
  for (char c : z) {
    command.emplace_back(c);
  }
  command.emplace_back(0x0d/*CR*/);
  command.emplace_back(0x0a/*LF*/);

  write(command);
}
