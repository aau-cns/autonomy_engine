// Copyright (C) 2021 and Alessandro Fornasier,
// Control of Networked Systems, Universitaet Klagenfurt, Austria
//
// You can contact the author at <alessandro.fornasier@ieee.org>
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

#ifndef EXCEPT_H
#define EXCEPT_H

#include <iostream>
#include <exception>

struct FailureException : public std::exception
{
  const char* what () const noexcept {
    return "General failure";
  }
};

struct TimerOverflowException : public std::exception
{
  const char* what () const noexcept {
    return "Timer overflow";
  }
};

struct DataOverflowException : public std::exception
{
  const char* what () const noexcept {
    return "Data overflow";
  }
};

#endif  // EXCEPT_H
