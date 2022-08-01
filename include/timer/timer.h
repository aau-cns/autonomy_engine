// Copyright (C) 2021 Alessandro Fornasier,
// Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <alessandro.fornasier@ieee.org>

#ifndef TIMER_H
#define TIMER_H

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/signals2.hpp>
#include <thread>

class Timer
{
public:
  /**
   * @brief Timer default constructur
   */
  Timer();

  /**
   * @brief Timer constructur
   * @param reference to int (timeout in milliseconds)
   */
  Timer(const int& ms);

  /**
   * @brief Timer destructor
   */
  ~Timer();

  /**
   * @brief Get timeout
   * @return timeout
   */
  const int& getTimeout() const;

  /**
   * @brief Set timeout
   * @param reference to int milliseconds
   */
  void setTimeout(const int& ms);

  /**
   * @brief Start or restart timer, this function call will deleate any pending asynch wait
   */
  void resetTimer();

  /**
   * @brief Stop timer, this function call will deleate any pending asynch wait
   */
  void stopTimer();

  /**
   * @brief Get active_ flag
   */
  [[nodiscard]] inline bool isActive() const
  {
    return active_;
  }

  /// Signal handler
  boost::signals2::signal<void()> sh_;

private:
  /**
   * @brief Timeout handler function
   */
  void timeoutHandler(const boost::system::error_code& error);

  /// Thread to run io service
  std::thread th_;

  /// Boost io service
  boost::asio::io_service io_;

  /// Deadline_timer
  std::unique_ptr<boost::asio::deadline_timer> timer_;

  /// Timeout in milliseconds
  int timeout_ = 0;

  /// timer active flag
  bool active_ = false;
};

#endif  // TIMER_H
