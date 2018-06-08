#pragma once

#include <vector>
#include <mutex>
#include <string>
#include <memory>
#include <functional>

#include <boost/asio.hpp>

/**
 * @brief Possible states of the total station.
 */
enum class TSState { off, on };

class TSInterface {
 public:
  explicit TSInterface(std::function<void(const double, const double, const double)> locationCallback);

  /**
   * @brief Sends the start command to the total station.
   */
  void start(void);

  /**
   * @brief Sends the end command to the total station.
   */
  void end(void);

  /**
   * @brief Set the position of the prism to track externally
   *
   * @param x X coordinate
   * @param y Y coordinate
   * @param z Z coordinate
   */
  void setPrismPosition(double x, double y, double z);

 protected:
  /**
   * @brief Starts the timer to detect if no messages are received anymore.
   */
  void startTimer();

  /**
   * @brief Sends the total station a command.
   *
   * @param command std::vector<char> with the command.
   */
  virtual void write(std::vector<char> str) = 0;

  /**
   * @brief Callback method for the timer.
   *        Starts searching for the prism if no message was received since last time.
   *        Restarts timer at the end.
   */
  void timerHandler();

  /**
   * @brief Starts the prism search on the total station.
   */
  void searchPrism(void);

  /**
   * @brief Turn total station to the position received externally.
   */
  void turnTelescope(void);

  TSState tsState_;                                       /**< State of the total station */

  std::vector<double> prismPosition_;                     /**< Position of the prism given bei Rovio or Vicon */

  std::unique_ptr<boost::asio::io_service> io_context_;   /**< io context object  */

  boost::asio::streambuf readData_;                       /**< Streambuffer in which the incomming messages is stored */

  bool timerStartedFlag_;                                 /**< Flag indicating if the timer started or not */
  boost::asio::deadline_timer timer_;                     /**< Deadline timer */
  bool messagesReceivedFlag_;                             /**< Flag indicating if a message was received */
  std::mutex messageReceivedMutex_;                       /**< Mutex for the corresponding flag */

  bool searchingPrismFlag_;                               /**< Flag indicating that the total station searches the prism */
  std::mutex searchingPrismMutex_;                        /**< Mutex for the corresponding flag */

  bool externalPositionReceivedFlag_;                     /**< Flag indicating if a recent position was received externally */
  std::mutex externalPositionReceivedMutex_;              /**< Mutex for the corresponding flag */

  std::function<void(const double, const double, const double)> locationCallback_; /**< Function pointer for the callback function */
};
