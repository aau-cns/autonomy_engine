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

class Timer
{

  public:

    /**
     * @brief Timer constructur
     * @param reference to io service
     * @param timeout in milliseconds
     */
    Timer(boost::asio::io_service &io, int ms);

    /**
     * @brief Set timeout
     * @param double milliseconds
     */
    void setTimeout(const int ms);

    /**
     * @brief Start or restart timer, this function call
     *        will deleate any pending asynch wait
     */
    void restartTimer();

  private:

    /**
     * @brief Deadline_timer
     */
    boost::asio::deadline_timer timer_;

    /**
     * @brief Timeout in milliseconds
     */
    boost::posix_time::millisec timeout_;

    /**
     * @brief Timeout handler function
     */
    void timeoutHandler();

};

#endif  // TIMER_H
