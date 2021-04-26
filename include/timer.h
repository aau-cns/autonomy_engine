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

#ifndef TIMER_H
#define TIMER_H

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <thread>
#include <boost/signals2.hpp>

class Timer
{

  public:

    /**
     * @brief Timer constructur
     * @param reference to int (timeout in milliseconds)
     */
    Timer(int &ms);

    /**
     * @brief Timer destructor
     */
    ~Timer();

    /**
     * @brief Set timeout
     * @param reference to int milliseconds
     */
    void setTimeout(const int &ms);

    /**
     * @brief Get timeout
     * @return timeout
     */
    const int& getTimeout() const;

    /**
     * @brief Start or restart timer, this function call will deleate any pending asynch wait
     */
    void resetTimer();

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
    std::shared_ptr<boost::asio::deadline_timer> timer_;

    /// Timeout in milliseconds
    int timeout_ = 0;

    /// timer init flag
    bool init_ = false;

};

#endif  // TIMER_H
