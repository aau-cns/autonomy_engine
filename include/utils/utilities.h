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

#ifndef UTILITIES_H
#define UTILITIES_H

#define VARIABLE_TO_STRING(Variable) (void(Variable), #Variable)
#define FUNCTION_TO_STRING(Function) (void(&Function), #Function)
#define METHOD_TO_STRING(ClassName, Method) (void(&ClassName::Method), #Method)
#define TYPE_TO_STRING(Type) (void(sizeof(Type)), #Type)

#include <exception>
#include <iostream>
#include <iterator>
#include <vector>

/**
 * @brief Function to print a vector
 * @param output stream (std::cout usually)
 * @param vector to be printed (elements of vector must be printable)
 */
template <typename T>
std::ostream& operator<<(std::ostream& out, const std::vector<T>& v)
{
  // Check vector is not empty
  if (!v.empty())
  {
    // Copy element of vector into output stream
    std::copy(v.begin(), v.end(), std::ostream_iterator<T>(out, ", "));
  }
  return out;
}

#endif  // UTILITIES_H
