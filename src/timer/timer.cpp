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

#include "timer/timer.h"
#include "utils/colors.h"
#include "utils/except.h"

Timer::Timer()
{
  // Make timer
  timer_ = std::make_unique<boost::asio::deadline_timer>(io_);
}

Timer::Timer(const int& ms)
{
  // Make timer
  timer_ = std::make_unique<boost::asio::deadline_timer>(io_);

  // Set timeout
  setTimeout(ms);
}

Timer::~Timer()
{
  // Check if thread joinable, then wait for it to complete its execution (join)
  if (th_.joinable())
  {
    th_.join();
  }
}

void Timer::setTimeout(const int& ms)
{
  // Set timeout in milliseconds if valid
  if (ms > 0)
  {
    timeout_ = ms;
  }
  else
  {
    std::cout << BOLD(YELLOW("Invalid timer timout\n")) << std::endl;
  }
}

const int& Timer::getTimeout() const
{
  return timeout_;
}

void Timer::resetTimer()
{
  // Start asynchronous waiting
  timer_->expires_from_now(boost::posix_time::millisec(timeout_));
  timer_->async_wait(boost::bind(&Timer::timeoutHandler, this, boost::asio::placeholders::error));

  // if not active run io service in a thread, call signal handler if exception is cought
  if (!active_)
  {
    th_ = std::thread([this]() {
      try
      {
        io_.run();
      }
      catch (const TimerOverflowException&)
      {
        sh_();
      }
    });
    active_ = true;
  }
}

void Timer::stopTimer()
{
  // If not active cancel any pending asynch wait
  if (active_)
  {
    timer_->cancel();
    active_ = false;
  }
}

void Timer::timeoutHandler(const boost::system::error_code& error)
{
  // Skip errors raised by resetting timer, if timeout then raise an exception
  if (error != boost::asio::error::operation_aborted)
  {
    throw TimerOverflowException();
  }
}
