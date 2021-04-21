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

Timer::Timer(boost::asio::io_service &io, int ms) :
  timer_(io), timeout_(boost::posix_time::millisec(ms)) {}

void Timer::setTimeout(const int ms)
{
  // Set timeout in milliseconds and
  timeout_ = boost::posix_time::millisec(ms);
}

void Timer::restartTimer()
{
  // Start asynchronous waiting
  timer_.expires_from_now(timeout_);
  timer_.async_wait(boost::bind(&Timer::timeoutHandler, this));
}

void Timer::timeoutHandler()
{
  // [Alessandro] just for testing
  throw std::exception();
}
