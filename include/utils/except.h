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

#ifndef EXCEPT_H
#define EXCEPT_H

#include <exception>
#include <iostream>

struct FailureException : public std::exception
{
  const char* what() const noexcept
  {
    return "General failure";
  }
};

struct TimerOverflowException : public std::exception
{
  const char* what() const noexcept
  {
    return "Timer overflow";
  }
};

struct DataOverflowException : public std::exception
{
  const char* what() const noexcept
  {
    return "Data overflow";
  }
};

#endif  // EXCEPT_H
