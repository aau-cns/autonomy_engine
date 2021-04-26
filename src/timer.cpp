// Copyright (C) 2021 Christian Brommer and Alessandro Fornasier,
// Control of Networked Systems, Universitaet Klagenfurt, Austria
//
// You can contact the author at <christian.brommer@ieee.org>
// and <alessandro.fornasier@ieee.org>
//
// All rights reserved.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
// DEALINGS IN THE SOFTWARE.

#include "timer.h"
#include "colors.h"

Timer::Timer(int &ms) {

  // Make timer
  timer_ = std::make_shared<boost::asio::deadline_timer>(io_);

  // Set timeout
  setTimeout(ms);
}

Timer::~Timer() {

  // Check if thread joinable, then wait for it to complete its execution (join)
  if (th_.joinable()) {
    th_.join();
  }
};

void Timer::setTimeout(const int &ms) {

  // Set timeout in milliseconds if valid
  if (ms > 0) {
    timeout_ = ms;
  } else {
    std::cout << std::endl << BOLD(YELLOW("Invalid timer timout")) << std::endl;
  }
}

const int& Timer::getTimeout() const {
  return timeout_;
}

void Timer::resetTimer() {

  // Start asynchronous waiting
  timer_->expires_from_now(boost::posix_time::millisec(timeout_));
  timer_->async_wait(boost::bind(&Timer::timeoutHandler, this, boost::asio::placeholders::error));

  // if not init run io service in a thread, call signal handler if exception is cought
  if(!init_) {
    th_ = std::thread([this](){
      try {
        io_.run();
      } catch (const std::exception&) {
        sh_();
      }

    });
    init_ = true;
  }
}

void Timer::timeoutHandler(const boost::system::error_code& error) {

  // Skip errors raised by resetting timer, if timeout then raise an exception
  if(error != boost::asio::error::operation_aborted) {
    throw std::exception();
  }
}
